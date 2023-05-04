/*
FILE: main.c
PROJECT: PCB MUX
AUTHOR: R Ellingham
DATE MODIFIED: May 2023
PROGRAM DESC: Code to mux a current source and voltage measurements for 
Electrical Impedance Tomography using an adjacent electrode current injection pattern


*/

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "driver/gpio.h"
#include <driver/spi_master.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ESP_INR_FLAG_DEFAULT 0
#define LED_PIN  27
#define PUSH_BUTTON_PIN  33

#define NUM_ELECS 16 // Number of electrodes in ERT setup
static uint8_t isnk_elec = 1;
static uint8_t isrc_elec = 0;
static uint8_t vn_elec = 0;
static uint8_t vp_elec = 1;
static uint8_t reading_count = 0;

// MUX GPIO
#define EN_MUX_ALL 4
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<EN_MUX_ALL))

#define MUX_CS 2
#define MUX_CLK 14
#define MUX_COPI 13
#define MUX_CIPO 12

TaskHandle_t ISR = NULL;

void setup_all_gpio() {
    // output pins
    gpio_config_t o_conf;
    //disable interrupt
    o_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    o_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    o_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    o_conf.pull_down_en = 0;
    //disable pull-up mode
    o_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&o_conf);
}

void send_spi_cmd_init(void) {
	spi_bus_config_t bus_config;
    memset(&bus_config, 0, sizeof(spi_bus_config_t));
	bus_config.sclk_io_num = MUX_CLK;
	bus_config.mosi_io_num = MUX_COPI;
	bus_config.miso_io_num = MUX_CIPO;
	bus_config.quadwp_io_num = -1; // Not used
	bus_config.quadhd_io_num = -1; // Not used
    spi_bus_initialize(HSPI_HOST, &bus_config, 1);
}

void send_spi_cmd(uint8_t data[], uint8_t CS_pin) {
    uint8_t data_len = sizeof(data);

	spi_device_handle_t handle;

	spi_device_interface_config_t dev_config;
	dev_config.address_bits = 0;
	dev_config.command_bits = 0;
	dev_config.dummy_bits = 0;
	dev_config.mode = 0;
	dev_config.duty_cycle_pos = 0;
	dev_config.cs_ena_posttrans = 0;
	dev_config.cs_ena_pretrans = 0;
	dev_config.clock_speed_hz = 700000;
    dev_config.input_delay_ns = 0;
	dev_config.spics_io_num = CS_pin;
	dev_config.flags = 0;
	dev_config.queue_size = 1;
	dev_config.pre_cb = 0;
	dev_config.post_cb = 0;

    spi_bus_add_device(HSPI_HOST, &dev_config, &handle);

	spi_transaction_t trans_desc;
	trans_desc.addr = 0;
	trans_desc.cmd = 0;
	trans_desc.flags = 0;
	trans_desc.length = data_len/2 * 8;
	trans_desc.rxlength = data_len/2 * 8;
	trans_desc.tx_buffer = data;
	trans_desc.rx_buffer = data;

    spi_device_transmit(handle, &trans_desc);

    spi_bus_remove_device(handle);

    // spi_bus_free(HSPI_HOST);
}

uint8_t sel_mux_frmt(uint8_t elec_1, uint8_t elec_2) {
    // Inputs elec_1 and elec2 get turned into a value in the format 0x(elec_1)(elec_2)
    if ((elec_1 > 15) || (elec_2 > 15)){
        printf("MUX array index out of bounds (i.e. %d or %d)\n", elec_1, elec_2);
    }
    uint8_t data;
    data = (elec_1 << 4) + elec_2;
    return data;
}

uint8_t iter_elec(int8_t increment, uint8_t elec_val, uint8_t num_elecs) {
    // Rolls up or down through electrode values. Increment = 1, decrement = 0, none = -1
    if (increment > 0) {
        return elec_val++ < num_elecs-1? elec_val : 0;
    }
    if (increment == 0) {
        return elec_val-- < (num_elecs+1) ? elec_val : num_elecs-1;
    }
    else {
        return elec_val;
    }
}

void IRAM_ATTR button_isr_handler(void *arg){
  xTaskResumeFromISR(ISR);
}

void interrupt_task(void *arg){
  while(1){
    vTaskSuspend(NULL);
    printf("You interrupted me!\n");
    reading_count++;
    isnk_elec = iter_elec(1,isnk_elec,NUM_ELECS);
    isrc_elec = iter_elec(1,isrc_elec,NUM_ELECS);
    if (reading_count == NUM_ELECS-1) {
        reading_count = 0;
        vn_elec = iter_elec(1,vn_elec,NUM_ELECS);
        vp_elec = iter_elec(1,vp_elec,NUM_ELECS);
    }
  }
}

void app_main(void)
{
    esp_rom_gpio_pad_select_gpio(PUSH_BUTTON_PIN);
    esp_rom_gpio_pad_select_gpio(LED_PIN);

    gpio_set_direction(PUSH_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(LED_PIN ,GPIO_MODE_OUTPUT);

    gpio_set_intr_type(PUSH_BUTTON_PIN, GPIO_INTR_ANYEDGE); // no other option for INTR_TYPE works
    gpio_install_isr_service(ESP_INR_FLAG_DEFAULT);
    gpio_isr_handler_add(PUSH_BUTTON_PIN, button_isr_handler, NULL);

    esp_log_level_set("*", ESP_LOG_ERROR);
    printf("here");
    // setup GPIO
    setup_all_gpio();
    printf("here");
    uint8_t EN_ALL_MUX = 1;
    gpio_set_level(EN_MUX_ALL, EN_ALL_MUX);

    // setup SPI
    send_spi_cmd_init();
    
    // electrode setup
    uint8_t i_elecs = sel_mux_frmt(isnk_elec, isrc_elec);
    uint8_t v_elecs = sel_mux_frmt(vn_elec, vp_elec);

    xTaskCreate(interrupt_task, "interrupt_task", 4096, NULL, 10, &ISR);

	while(1) {
        vTaskDelay(100);
        printf("isrc%d isnk%d vn%d vp%d\n", isnk_elec, isrc_elec, vn_elec, vp_elec);
	}
	vTaskDelete(NULL);
}
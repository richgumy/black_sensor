/*

***UNSTABLE VERSION***
***COMPLIES WITH THIS VERSION***

Just code checking the mucks and currant sauce is were-king

*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "esp_adc_cal.h"
#include <sys/time.h>


#include <driver/spi_master.h>
#include <esp_log.h>

#include "sdkconfig.h"

#define V_OFFSET 2429 // Virtual ground for ADC. (Not in ERT PCB v1.2) 
#define NUM_ELECS 16 // Number of electrodes in ERT setup

// GPIO
    // MUX
#define EN_MUX_ISRC 5
#define EN_MUX_VGND 17
#define EN_MUX_VMEASP 4
#define EN_MUX_VMEASN 16
#define MUX_CS_PIN 2
    // LED
#define EN_LED 19
#define SW_LED 21
// ADC
#define ADC_CS_PIN 15

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<SW_LED) |(1ULL<<EN_MUX_ISRC) | (1ULL<<EN_MUX_VGND) | (1ULL<<EN_MUX_VMEASP) | (1ULL<<EN_MUX_VMEASN) | (1ULL<<EN_LED))
#define GPIO_INPUT_PIN_SEL  (1ULL<<SW_LED)


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

    // input pins
    gpio_config_t i_conf;
    //disable interrupt
    i_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    i_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set
    i_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //disable pull-down mode
    i_conf.pull_down_en = 0;
    //disable pull-up mode
    i_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&i_conf);
}

void send_spi_cmd_init(void) {
	spi_bus_config_t bus_config;
    memset(&bus_config, 0, sizeof(spi_bus_config_t));
	bus_config.sclk_io_num = 14; // CLK
	bus_config.mosi_io_num = 13; // MOSI
	bus_config.miso_io_num = 12; // MISO
	bus_config.quadwp_io_num = -1; // Not used
	bus_config.quadhd_io_num = -1; // Not used

    spi_bus_initialize(HSPI_HOST, &bus_config, 1);
}

void send_spi_cmd(uint8_t data[], uint8_t CS_pin) {
         
    // ////// TIMING CODE START ///////
    // struct timeval tv_I;
    // gettimeofday(&tv_I, NULL);
    // int64_t time_us_I = (int64_t)tv_I.tv_sec * 1000000L + (int64_t)tv_I.tv_usec;
    // ////////////////////////////////

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
    // ////// TIMING CODE FINISH //////
    // struct timeval tv_F;
    // gettimeofday(&tv_F, NULL);
    // int64_t time_us_F = (int64_t)tv_F.tv_sec * 1000000L + (int64_t)tv_F.tv_usec;
    // int64_t tv_D = time_us_F - time_us_I;
    // printf("%lld\n", tv_D);
    // ////////////////////////////////
    // spi_bus_free(HSPI_HOST);
}

uint32_t bytes16TOuint32(uint8_t data[]) {
    uint32_t conv_data = 0;
    for (int i=0;i<2;i++){
        conv_data = (conv_data << 8) + data[i];      
    }
    return conv_data;
}

uint32_t adc_get_avg(uint8_t adc_data[], int num_samples) {
    uint32_t spi_read_avg = 0;
    send_spi_cmd(adc_data, ADC_CS_PIN); // get rid of first ADC reading from previous cycle
    for (int i = 0; i < num_samples; i++) {
        send_spi_cmd(adc_data, ADC_CS_PIN); // Send SPI cmd to read ADC and store value in 'spi_read_ADC'
        spi_read_avg += bytes16TOuint32(adc_data);
    }
    return spi_read_avg /= num_samples;
}

uint32_t adcbits_to_uV(uint32_t adc_data_16b, int Vcc_V, int bits) {
    uint32_t out_uV = (Vcc_V*1000000/pow(2,bits))* adc_data_16b;
    return out_uV;
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


void app_main(void)
{   
    esp_log_level_set("*", ESP_LOG_ERROR);
    
    setup_all_gpio();
    uint8_t EN_ALL_MUX = 1;
    gpio_set_level(EN_MUX_ISRC, EN_ALL_MUX);
    gpio_set_level(EN_MUX_VGND, EN_ALL_MUX);
    gpio_set_level(EN_MUX_VMEASP, EN_ALL_MUX);
    gpio_set_level(EN_MUX_VMEASN, EN_ALL_MUX);
    send_spi_cmd_init();

    // setup ADC params/consts
    uint8_t spi_read_ADC[2]; // [2] bytes for 16bit ADC
    int num_samples = 200;
    uint32_t adc_out;
    int Vcc = 5;
    int adcbits = 16;

    // Electrode setup
    int num_elecs = 16;
    uint8_t isnk_elec = 1;
    uint8_t isrc_elec = 0;
    uint8_t i_elecs = sel_mux_frmt(isnk_elec, isrc_elec);

    uint8_t vn_elec = 0;
    uint8_t vp_elec = 1;
    uint8_t v_elecs = sel_mux_frmt(vn_elec, vp_elec);
    
    while (1) {

        for (int j = 0; j < num_elecs; j++) { 
            // iterate current source
            for (int i = 0; i < num_elecs; i++) { 
                // iterate ADC voltage measure electrodes
                vTaskDelay(10 / portTICK_PERIOD_MS); // pause for XXs while switching MUX!!
                uint8_t elec_sel_V[2] = {i_elecs, v_elecs};
                send_spi_cmd(elec_sel_V, MUX_CS_PIN); 
                adc_out = adc_get_avg(spi_read_ADC, num_samples);
                // printf("ADC:%u uV\n",adcbits_to_uV(adc_out, Vcc, adcbits));
                printf("%u,\n",adcbits_to_uV(adc_out, Vcc, adcbits));
                vn_elec = iter_elec(1, vn_elec, num_elecs);
                vp_elec = iter_elec(1, vp_elec, num_elecs);
                v_elecs = sel_mux_frmt(vn_elec, vp_elec);
            }
            // printf("Iter i: %d\n",j);
            isnk_elec = iter_elec(1, isnk_elec, num_elecs);
            isrc_elec = iter_elec(1, isrc_elec, num_elecs);
            i_elecs = sel_mux_frmt(isnk_elec, isrc_elec);
            uint8_t elec_sel_I[2] = {i_elecs, v_elecs};
            // printf("Currently: %x %x\n",isnk_elec,isrc_elec);
            send_spi_cmd(elec_sel_I, MUX_CS_PIN);
        }
    }
}


// // LED Demo with LEDs across 1,2,3
// vTaskDelay(2000 / portTICK_PERIOD_MS); // pause for XXs while switching MUX!!
// uint8_t elec_index1[2] = {0x21,0x21};
// send_spi_cmd(elec_index1, MUX_CS_PIN); // Send SPI cmd to read ADC and store value in 'spi_read_ADC'
// printf("RED LED\n");
// adc_out = adc_get_avg(spi_read_ADC, num_samples);
// printf("ADC:%u uV\n",adcbits_to_uV(adc_out, Vcc, adcbits));

// vTaskDelay(2000 / portTICK_PERIOD_MS); // pause for XXs while switching MUX!!
// uint8_t elec_index2[2] = {0x23,0x23};
// send_spi_cmd(elec_index2, MUX_CS_PIN); // Send SPI cmd to read ADC and store value in 'spi_read_ADC'
// printf("BLUE LED\n");
// adc_out = adc_get_avg(spi_read_ADC, num_samples);
// printf("ADC:%u uV\n",adcbits_to_uV(adc_out, Vcc, adcbits));

// vTaskDelay(2000 / portTICK_PERIOD_MS); // pause for XXs while switching MUX!!
// uint8_t elec_index3[2] = {0x45,0x45};
// send_spi_cmd(elec_index3, MUX_CS_PIN); // Send SPI cmd to read ADC and store value in 'spi_read_ADC'
// printf("NO LED\n");
// adc_out = adc_get_avg(spi_read_ADC, num_samples);
// printf("ADC:%u uV\n",adcbits_to_uV(adc_out, Vcc, adcbits));
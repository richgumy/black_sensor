/*
FILE: main.c
PROJECT: PCB MUX
AUTHOR: R Ellingham
DATE MODIFIED: May 2023
PROGRAM DESC: Code to multiplex a current source and voltage measurements for 
Electrical Impedance Tomography using an adjacent electrode current injection pattern. 
Using an ESP32 to send SPI commands to four multiplexers.
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
#include "driver/uart.h"


// Define all UART intr realted params
static const char *TAG = "uart_events";

#define EX_UART_NUM UART_NUM_0
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

static QueueHandle_t uart0_queue;

// Electrode globals
#define NUM_ELECS 16 // Number of electrodes in ERT setup
static uint8_t isnk_elec = 1;
static uint8_t isrc_elec = 0;
static uint8_t vn_elec = 0;
static uint8_t vp_elec = 1;
static uint8_t reading_count = 0;

// MUX GPIO
#define EN_MUX_ALL 4
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<EN_MUX_ALL))

#ifdef CONFIG_IDF_TARGET_ESP32
#   define SPI_HOST_ID    HSPI_HOST
#   define MUX_CIPO 18
#    define MUX_COPI 23
#    define MUX_CLK  19
#    define MUX_CS   13

// NOT TESTED WITH THE FOLLOWING ESP VARIANTS!!
#elif defined CONFIG_IDF_TARGET_ESP32S2
#  define SPI_HOST_ID    SPI2_HOST

#  define MUX_CIPO 37
#  define MUX_COPI 35
#  define MUX_CLK  36
#  define MUX_CS   34
#elif defined CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C2
#  define SPI_HOST_ID    SPI2_HOST

#  define MUX_CIPO 2
#  define MUX_COPI 7
#  define MUX_CLK  6
#  define MUX_CS   10

#elif CONFIG_IDF_TARGET_ESP32S3
#  define SPI_HOST_ID    SPI2_HOST

#  define MUX_CIPO 13
#  define MUX_COPI 11
#  define MUX_CLK  12
#  define MUX_CS   10
#endif

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

void spi_config_init(void) {
	spi_bus_config_t bus_config;
    memset(&bus_config, 0, sizeof(spi_bus_config_t));
	bus_config.sclk_io_num = MUX_CLK;
	bus_config.mosi_io_num = MUX_COPI;
	bus_config.miso_io_num = MUX_CIPO; // Not used
	bus_config.quadwp_io_num = -1; // Not used
	bus_config.quadhd_io_num = -1; // Not used
    spi_bus_initialize(SPI_HOST_ID, &bus_config, 1);
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

    spi_bus_add_device(SPI_HOST_ID, &dev_config, &handle);

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

    // spi_bus_free(SPI_HOST_ID);
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

void iter_elecs_adj(void){
    // Iterates electrode pattern for adjacent EIT electrode drive sequence
    reading_count++;
    isnk_elec = iter_elec(1,isnk_elec,NUM_ELECS);
    isrc_elec = iter_elec(1,isrc_elec,NUM_ELECS);
    if (reading_count == NUM_ELECS) {
        reading_count = 0;
        vn_elec = iter_elec(1,vn_elec,NUM_ELECS);
        vp_elec = iter_elec(1,vp_elec,NUM_ELECS);
    }
    uint8_t data[2] = {sel_mux_frmt(isnk_elec, isrc_elec),sel_mux_frmt(vn_elec,vp_elec)};
    send_spi_cmd(data, MUX_CS);
}

static void uart_event_task(void *pvParameters)
{
    /*
    Command prompt for the PCB_MUX setup.
    Cmds sent over UART:
    i = iterate electrodes
    g = get current electrode state
    c = get iteration (reading) count
    */ 
    uart_event_t event;
    // size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGI(TAG, "\n[UART DATA SZ]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "[DATA EVT]: %c", (char)(dtmp[0]));
                    uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                    if (dtmp[0] == (uint8_t)('i')) {
                        // ITERATE ELECTRODE PATTERN
                        iter_elecs_adj();    
                        ESP_LOGI(TAG, "[ITERATION COUNT]: %d", reading_count);
                    }
                    else if (dtmp[0] == (uint8_t)('g')) {
                        // GET ELECTRODE STATE
                        printf("isrc%d,isnk%d,vn%d,vp%d\n", isnk_elec, isrc_elec, vn_elec, vp_elec);
                    }
                    else if (dtmp[0] == (uint8_t)('c')) {
                        // GET ITERATION COUNT
                        ESP_LOGI(TAG, "[ITERATION COUNT]: %d", reading_count);
                        printf("%d", reading_count);
                    }
                    else {
                        ESP_LOGI(TAG, "[UNKNOWN DATA]: %c", dtmp[0]);
                    }
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}


void app_main(void)
{
    // setup UART interrupt
    esp_log_level_set(TAG, ESP_LOG_NONE); // en/disable log interrupt related messages

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // setup GPIO
    setup_all_gpio();
    uint8_t EN_ALL_MUX = 1;
    gpio_set_level(EN_MUX_ALL, EN_ALL_MUX);

    // setup SPI for MUX coms
    spi_config_init();
    
    // electrode setup
    uint8_t i_elecs = sel_mux_frmt(isnk_elec, isrc_elec);
    uint8_t v_elecs = sel_mux_frmt(vn_elec, vp_elec);
    uint8_t elec_data[2] = {i_elecs, v_elecs};
    send_spi_cmd(elec_data, MUX_CS);

    // create a UART triggered task
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);

	while(1) {
        vTaskDelay(100);
        ESP_LOGI(TAG, "loop");
	}
}
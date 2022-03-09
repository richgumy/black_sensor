/*
ERT program!
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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

#define V_OFFSET 2429 // Virtual ground for ADC.
#define NUM_ELECS 16 // Number of electrodes in ERT setup

// ERT modes
#define STANDBY -1
#define CALIBRATE 0
#define ADJACENT 1
#define PSEUDO_POLAR 2
#define PP_PP 3 // See paper "A Quantitative Evaluation of Drive Pattern Selection for Optimizing EIT-Based Stretchable Sensors - Russo et al."

static const uint8_t ert_mode = ADJACENT; // <- SET ELECTRODE DRIVE PATTERN MODE HERE //

// GPIO
    // MUX
#define EN_MUX_ISRC 5
#define EN_MUX_VGND 17
#define EN_MUX_VMEASP 4
#define EN_MUX_VMEASN 16
#define MUX_CS_PIN 15
    // LED
#define EN_LED 18
#define SW_LED 21

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<SW_LED) |(1ULL<<EN_MUX_ISRC) | (1ULL<<EN_MUX_VGND) | (1ULL<<EN_MUX_VMEASP) | (1ULL<<EN_MUX_VMEASN) | (1ULL<<EN_LED))
#define GPIO_INPUT_PIN_SEL  (1ULL<<SW_LED)

// I2C
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

// ADC
#define ADC_CS_PIN 19
#define MAX_16BIT_VAL   65536
#define NO_ADC_SAMPLES   1         //Multisampling

// Electrodes
static const uint8_t max_elecs = 128; // Maximum number of electrodes
static int8_t cycle_dir = 1; // Increment through electrode values
static uint8_t isrc_elec;
static uint8_t isnk_elec;
static uint8_t vp_elec;
static uint8_t vn_elec;


// Electrical measurement structures
struct Vmeas {
    uint8_t vp_elec;
    uint8_t vn_elec;
    uint32_t voltage;
};
struct Cycle_meas {
    uint8_t isrc_elec;
    uint8_t isnk_elec;
    struct Vmeas vm[NUM_ELECS];
};

// Error log tag
// static const char TAG[] = "main";

void init_ERT_mode (void) {
    #if (ert_mode == STANDBY)
        printf("MODE = STANDBY\n");
    #elif (ert_mode == CALIBRATE)
        isrc_elec = 1;
        isnk_elec = 0;
        vp_elec = 1;
        vn_elec = 0;
        printf("MODE = CALIBRATE\n");
    #elif ert_mode == ADJACENT)
        isrc_elec = 1;
        isnk_elec = 0;
        vp_elec = 1;
        vn_elec = 0;
        printf("MODE = ADJACENT\n");
    #elif (ert_mode == PSEUDO_POLAR)
        isrc_elec = 0;
        isnk_elec = 7;
        vp_elec = 0;
        vn_elec = 1;
        printf("MODE = PSEUDO_POLAR\n");
    #elif (ert_mode == PP_PP)
        isrc_elec = 0;
        isnk_elec = 7;
        vp_elec = 0;
        vn_elec = 7;
        printf("MODE = PP_PP\n");
    #else
        #error "INVALID ELECTRODE DRIVE MODE"
    #endif
}

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

void send_spi_read_cmd(uint8_t data[], uint8_t CS_pin) {
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
	dev_config.clock_speed_hz = 10000;
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
	trans_desc.length = data_len * 8; // Unsure if this is bits or bytes? test this...?
	trans_desc.rxlength = data_len * 8;
	trans_desc.tx_buffer = data;
	trans_desc.rx_buffer = data;

    spi_device_transmit(handle, &trans_desc);

    spi_bus_remove_device(handle);

    // spi_bus_free(HSPI_HOST);
}

int32_t conv_adc_readingLTC1864L(uint8_t data[]) {
    // long long conv_factor = 4865000; // Calibrate for each board's 5v supply (add V buffer to stabilise)
    int32_t conv_offset = 31360; // 32533; // Calibrate for each board (i.e. V_GND)

    int32_t conv_data = 0;
    for (int i=0;i<2;i++){
        // printf("%x\n",data[i]);
        conv_data = (conv_data << 8) + data[i];      
    }
    // if (conv_data >= 0xffff)
    // {
    //     conv_data = -1;
    //     conv_offset = 0;
    //     ESP_LOGE("*","ERROR 0x69 : ADC INPUT IS OUT OF RANGE");
    //     // printf("0x%x%x%x%x\n",data[0],data[1]);
    // }
    // printf("%lld\n",conv_data);

    conv_data = (conv_data - conv_offset)*5000; // Arb value with ref to VGND
    conv_data = conv_data / 65536;

    // printf("%lld\n",conv_data);

    return conv_data;
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
        return elec_val-- < (max_elecs+1) ? elec_val : num_elecs-1;
    }
    else {
        return elec_val;
    }
}

void print_vmeas_csv(struct Cycle_meas measurements, uint8_t num_elecs)
// Prints out csv formatted measurements
{
    for (int i=0; i<num_elecs; i++){
        printf(",%d", measurements.vm[i].voltage);
    }
    printf("\n");
}

static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Efuse is basically a register with permanent values 'burned' into it.
    
    //Check if TP is burned into eFuse
    //TP can be manually 'burned' into the MCU by the user
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}


void app_main(void)
{   
    /*
    SETUP
    */
    esp_log_level_set("*", ESP_LOG_ERROR); 

    init_ERT_mode();

    //Check if Two Point or Vref are burned into eFuse for ADC
    check_efuse();

    //Configure GPIO
    setup_all_gpio();
    uint8_t EN_ALL_MUX;
    EN_ALL_MUX = ert_mode == STANDBY? 1 : 0;
    gpio_set_level(EN_MUX_ISRC, EN_ALL_MUX);
    gpio_set_level(EN_MUX_VGND, EN_ALL_MUX);
    gpio_set_level(EN_MUX_VMEASP, EN_ALL_MUX);
    gpio_set_level(EN_MUX_VMEASN, EN_ALL_MUX);

    // Assign initial mux electrode values
    uint8_t i_elecs = sel_mux_frmt(isnk_elec, isrc_elec);
    uint8_t v_elecs = sel_mux_frmt(vn_elec, vp_elec);
    uint8_t elec_index[2] = {i_elecs, v_elecs};

    // SPI read ADC
    send_spi_cmd_init();
    uint8_t spi_read_ADC[2]; // [2] for 16bit ADC
    int spi_read_ADC_arr[16];

    /*
    MAIN LOOP
    Continuously cycles through electrodes pattern
    */
    while (1) {
        while (ert_mode != STANDBY){
            // Store all voltage measurements and electrodes used in cycle_read
            struct Cycle_meas cycle_read;
            cycle_read.isrc_elec = isrc_elec;
            cycle_read.isnk_elec = isnk_elec;

            if (!cycle_read.isnk_elec){
                printf("A\n"); // Break in between full ERT measurement
            }
         
            // ////// TIMING CODE START //////
            // struct timeval tv_I;
            // gettimeofday(&tv_I, NULL);
            // int64_t time_us_I = (int64_t)tv_I.tv_sec * 1000000L + (int64_t)tv_I.tv_usec;

            // Cycle through each voltage measurement
            for (int elec_iter=0 ; elec_iter<NUM_ELECS ; elec_iter++)
            {
                // Store data in v_read
                struct Vmeas v_read;
                v_read.vp_elec = vp_elec;
                v_read.vn_elec = vn_elec;

                 // Send SPI mux cmd
                send_spi_read_cmd(elec_index, MUX_CS_PIN);                                   
            
                // ADC SPI read
                int32_t spi_read = 0;
                int32_t spi_read_avg = 0;
                for (int i = 0; i < NO_ADC_SAMPLES; i++) {
                    
                    send_spi_read_cmd(spi_read_ADC, ADC_CS_PIN);

                    spi_read = conv_adc_readingLTC1864L(spi_read_ADC);

                    spi_read_avg += spi_read;
                }
                spi_read_avg /= NO_ADC_SAMPLES;

                spi_read_ADC_arr[vp_elec] = spi_read_avg;

                // Cycle through voltage measurement electrodes
                vp_elec = iter_elec(cycle_dir, vp_elec, NUM_ELECS);
                vn_elec = iter_elec(cycle_dir, vn_elec, NUM_ELECS);
                v_elecs = sel_mux_frmt(vn_elec, vp_elec);
                elec_index[1] = v_elecs;

                if (ert_mode == CALIBRATE) {
                    // Calibration mode: measure impedance between each electrode
                    isnk_elec = iter_elec(cycle_dir, isnk_elec, NUM_ELECS);
                    isrc_elec = iter_elec(cycle_dir, isrc_elec, NUM_ELECS);
                }
                i_elecs = sel_mux_frmt(isnk_elec, isrc_elec);
                elec_index[0] = i_elecs;


            }

            // ////// TIMING CODE FINISH //////
            // struct timeval tv_F;
            // gettimeofday(&tv_F, NULL);
            // int64_t time_us_F = (int64_t)tv_F.tv_sec * 1000000L + (int64_t)tv_F.tv_usec;
            // int64_t tv_D = time_us_F - time_us_I;
            // printf("%lld\n", tv_D);

            // Print electrode measurements           
            for (int i=0; i<16; i++){
                printf("%hd,",spi_read_ADC_arr[i]);
            }
            printf("\n");
            
            if (ert_mode == ADJACENT) {
                // Adjacent mode: cycle through current source electrodes
                isnk_elec = iter_elec(cycle_dir, isnk_elec, NUM_ELECS);
                isrc_elec = iter_elec(cycle_dir, isrc_elec, NUM_ELECS);
                i_elecs = sel_mux_frmt(isnk_elec, isrc_elec);
                elec_index[0] = i_elecs;
            }           
        }
        gpio_set_level(EN_LED, 1);
    }
}

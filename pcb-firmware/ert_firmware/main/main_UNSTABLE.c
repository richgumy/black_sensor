/*

***UNSTABLE VERSION***
***COMPLIES WITH THIS VERSION***

ERT program!

The PCB firmware is all written in C for the ESP32-WROOM32E SoC. The firmware applies an electrode pattern to the electrodes and sends measurement data via the USB-UART serial connection. The basic electrode drive process is:

1. Apply current to 2 electrodes
2. Measure voltage across 16 electrode pairs
3. Send voltage measurement data via serial
4. Iterate to next set of current electrodes.
5. Back to step 1.

*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

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
#define MUX_CS_PIN 2
    // LED
#define EN_LED 19
#define SW_LED 21

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<SW_LED) |(1ULL<<EN_MUX_ISRC) | (1ULL<<EN_MUX_VGND) | (1ULL<<EN_MUX_VMEASP) | (1ULL<<EN_MUX_VMEASN) | (1ULL<<EN_LED))
#define GPIO_INPUT_PIN_SEL  (1ULL<<SW_LED)

// ADC
#define ADC_CS_PIN 15
#define MAX_16BIT_VAL   65536
#define NO_ADC_SAMPLES   1         // Multisampling

// Electrodes
static const uint8_t max_elecs = 128; // Maximum number of electrodes
static int8_t cycle_dir = 1; // Increment through electrode values (0 is decrement)
static uint8_t isrc_elec;
static uint8_t isnk_elec;
static uint8_t vp_elec;
static uint8_t vn_elec;


// Electrical measurement structures
struct Vmeas {
    uint8_t vp_elec;
    uint8_t vn_elec;
    uint16_t voltage;
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
    #elif (ert_mode == ADJACENT)
        isrc_elec = 1;
        isnk_elec = 0;
            vp_elec = 0;
        vn_elec = 1;
        printf("MODE = ADJACENT\n");
    #elif (ert_mode == CALIBRATE)
        isrc_elec = 1;
        isnk_elec = 0;
        vp_elec = 0;
        vn_elec = 1;
        printf("MODE = CALIBRATE\n");
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
        #error "INVALID ERT MODE!\n"
    #endif
    
    // printf("MODE = ")
    // switch(ert_mode) {

    // case STANDBY  :
    //     printf("STANDBY\n");
    //     break;
    // case CALIBRATE  :
    //     printf("CALIBRATE\n");
    //     break;
    // case ADJACENT  :
    //     printf("ADJACENT\n");
    //     break;
    // case PSEUDO_POLAR  :
    //     printf("PSEUDO_POLAR\n");
    //     break;
    // case PP_PP  :
    //     printf("PP_PP\n");
    //     break;
    // default :
    //     printf("INVALID ERT MODE!\n");
    // }
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

uint32_t conv_adc_readingLTC1864L(uint8_t data[]) {
    // Converts arbitrary analogue unit to readable mV value
    uint32_t conv_offset = 0;//32533; // Calibrate for each board (i.e. V_GND)
    uint32_t conv_scale = 65536;
    uint32_t conv_data = 0;

    for (int i=0;i<2;i++){
        conv_data = (conv_data << 8) + data[i];      
    }

    // conv_data = conv_data * 5 * 10000 / conv_scale ; // mV conversion

    return conv_data;
}

uint32_t to_resistance(uint32_t current_src_uA, uint32_t v_data_mV) {
    return v_data_mV * 1000 / current_src_uA;
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

void print_vmeas_csv_frmt(struct Cycle_meas measurements, uint8_t num_elecs)
// Prints out csv formatted measurements
{
    // Print electrode measurements           
    for (int i=0; i<num_elecs; i++){
        printf("%u,",measurements.vm[i].voltage);
    }
    printf("\n");
}

void app_main(void)
{   
    /*
    SETUP
    */
    esp_log_level_set("*", ESP_LOG_ERROR); 

    init_ERT_mode();

    //Configure GPIO
    setup_all_gpio();
    uint8_t EN_ALL_MUX = ert_mode == STANDBY? 1 : 0;
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
    uint8_t spi_read_ADC[2]; // [2] bytes for 16bit ADC
    uint16_t spi_read_ADC_arr[16];

    /*
    MAIN LOOP
    Continuously cycles through electrode pattern mode
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

            // Cycle through each voltage measurement
            for (int elec_iter=0 ; elec_iter<NUM_ELECS ; elec_iter++)
            {
                // Store data in v_read
                struct Vmeas v_read;
                v_read.vp_elec = vp_elec;
                v_read.vn_elec = vn_elec;

                 // Send SPI mux cmd
                send_spi_cmd(elec_index, MUX_CS_PIN);

                // vTaskDelay(100 / portTICK_PERIOD_MS); // pause for .1s while switching of MUXs happens!!!
            
                // ADC SPI read
                uint32_t spi_read = 0;
                uint32_t spi_read_avg = 0;
                send_spi_cmd(spi_read_ADC, ADC_CS_PIN); // Discard ADC reading from previous cycle
                for (int i = 0; i < NO_ADC_SAMPLES; i++) {
                    
                    send_spi_cmd(spi_read_ADC, ADC_CS_PIN); // Send SPI cmd to read ADC and store value in 'spi_read_ADC'

                    spi_read = conv_adc_readingLTC1864L(spi_read_ADC);

                    spi_read_avg += spi_read;
                }
                spi_read_avg /= NO_ADC_SAMPLES;

                // printf("%"PRIu32"\n",spi_read_avg);

                // printf("Isrc,Isnk:%d,%d.Vp,Vn:%d,%d\n", isrc_elec, isnk_elec, vp_elec, vn_elec);

                uint16_t spi_read_avg_16b = spi_read_avg;
                v_read.voltage = spi_read_avg_16b;
                // spi_read_ADC_arr[vp_elec] = spi_read_avg_16b;

                cycle_read.vm[elec_iter] = v_read;

                // Cycle through voltage measurement electrodes
                vp_elec = iter_elec(cycle_dir, vp_elec, NUM_ELECS);
                vn_elec = iter_elec(cycle_dir, vn_elec, NUM_ELECS);
                v_elecs = sel_mux_frmt(vn_elec, vp_elec);
                elec_index[1] = v_elecs;

                if (ert_mode == CALIBRATE) {
                    // printf("%u\n",isrc_elec);
                    for (int i=0; i < 0; i++){
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        printf("..%u\n",10-i);
                    }
                    // Calibration mode: measure impedance between each electrode
                    isnk_elec = iter_elec(cycle_dir, isnk_elec, NUM_ELECS);
                    isrc_elec = iter_elec(cycle_dir, isrc_elec, NUM_ELECS);
                    // vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                i_elecs = sel_mux_frmt(isnk_elec, isrc_elec);
                elec_index[0] = i_elecs;

                // vTaskDelay(1000 / portTICK_PERIOD_MS);
            }

            print_vmeas_csv_frmt(cycle_read, NUM_ELECS);
            
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

/* Adapted from esp-idf examples.
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

static const uint8_t ert_mode = CALIBRATE; // <- SET ELECTRODE DRIVE PATTERN MODE HERE //

// GPIO
    // MUX
#define EN_MUX_ISRC 5
#define EN_MUX_VGND 17
#define EN_MUX_VMEASP 4
#define EN_MUX_VMEASN 16
    // LED
#define EN_LED 18

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<EN_MUX_ISRC) | (1ULL<<EN_MUX_VGND) | (1ULL<<EN_MUX_VMEASP) | (1ULL<<EN_MUX_VMEASN) | (1ULL<<EN_LED))

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
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate?
#define NO_OF_SAMPLES   16          //Multisampling
    // Internal ADC params
static esp_adc_cal_characteristics_t *adc_chars;
#if CONFIG_IDF_TARGET_ESP32
static const adc_channel_t channel = ADC_CHANNEL_0;     //GPIO34 if ADC1, GPIO14 if ADC2 for CHANNEL6
static const adc_bits_width_t width = ADC_WIDTH_BIT_12; 
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_0;     // GPIO7 if ADC1, GPIO17 if ADC2 for CHANNEL6
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
    // I2C params
static gpio_num_t i2c_gpio_sda = 22;
static gpio_num_t i2c_gpio_scl = 23;
static uint32_t i2c_frequency = 400000;
static i2c_port_t i2c_port = I2C_NUM_0;
static uint8_t *ADC_chip_address = 0x4F;


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

void init_ERT_mode (void) {
    #if (ert_mode == CALIBRATE || ert_mode == ADJACENT)
        isrc_elec = 0;
        isnk_elec = 1;
        vp_elec = 0;
        vn_elec = 1;
    #elif (ert_mode == PSEUDO_POLAR)
        isrc_elec = 0;
        isnk_elec = 7;
        vp_elec = 0;
        vn_elec = 1;
    #elif (ert_mode == PP_PP)
        isrc_elec = 0;
        isnk_elec = 7;
        vp_elec = 0;
        vn_elec = 7;
    #else
        #error "INVALID ELECTRODE DRIVE MODE"
    #endif
}

static esp_err_t i2c_master_driver_initialize(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_gpio_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = i2c_gpio_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_frequency,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    return i2c_param_config(i2c_port, &conf);
}

void setup_all_gpio() {
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

int do_i2cget_cmd(uint8_t *chipaddr)
{
    /* chip address pointer */
    uint8_t chip_addr = *chipaddr;
    int len = 2;

    uint8_t *data = malloc(len);
    uint16_t output_data;

    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, len - 1, ACK_VAL);
    i2c_master_read_byte(cmd, data + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    // if (ret == ESP_OK) {
    //     for (int i = 0; i < len; i++) {
    //         printf("0x%02x ", data[i]);
    //         if ((i + 1) % 16 == 0) {
    //             printf("\r\n");
    //         }
    //     }
    //     if (len % 16) {
    //         printf("\r\n");
    //     }
    // }
    // } else if (ret == ESP_ERR_TIMEOUT) {
    //     ESP_LOGW(TAG, "Bus is busy");
    // } else {
    //     ESP_LOGW(TAG, "Read failed");
    // }
    output_data = data[1] + (data[0] << 8); 
    // printf("Test:0x%x\n", output_data);
    free(data);
    i2c_driver_delete(i2c_port);
    
    return output_data;
}

void send_spi_cmd(uint8_t data[]) {
	// ESP_LOGD(tag, ">> test_spi_task");

	spi_bus_config_t bus_config;
    memset(&bus_config, 0, sizeof(spi_bus_config_t));
	bus_config.sclk_io_num = 14; // CLK
	bus_config.mosi_io_num = 13; // MOSI
	bus_config.miso_io_num = -1; // MISO
	bus_config.quadwp_io_num = -1; // Not used
	bus_config.quadhd_io_num = -1; // Not used
	// ESP_LOGI(tag, "... Initializing bus.");
	// ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &bus_config, 1));
    spi_bus_initialize(HSPI_HOST, &bus_config, 1);

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
	dev_config.spics_io_num = 15; // CS pin = io15
	dev_config.flags = 0;
	dev_config.queue_size = 1;
	dev_config.pre_cb = 0;
	dev_config.post_cb = 0;
	// ESP_LOGI(tag, "... Adding device bus.");
	// ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &dev_config, &handle));
    spi_bus_add_device(HSPI_HOST, &dev_config, &handle);


	// char data[2];
	spi_transaction_t trans_desc;
	trans_desc.addr = 0;
	trans_desc.cmd = 0;
	trans_desc.flags = 0;
	trans_desc.length = 2 * 8;
	trans_desc.rxlength = 0;
	trans_desc.tx_buffer = data;
	trans_desc.rx_buffer = data;

    // The following messages map to s3,s2,s1,s0
    // Mapping to PCB MUX component ref: U9(Vm+) = 4, U6(Vm-) = 3, U10(Isrc) = 2, U7(V_gnd) = 1
	// data[0] = 0xFE; // 0xVGND,ISRC
	// data[1] = 0x12; // 0xVm-,Vm+

    // 
    //s3,s2,s1,s0
    // U9/ 0010 = 2 con-e1(6.6kohm) (should be electrode 2 (e2)??)
    // U7
    // 0001 = 1 con-e1(142ohm)

	// ESP_LOGI(tag, "... Transmitting.");
	// ESP_ERROR_CHECK(spi_device_transmit(handle, &trans_desc));
    spi_device_transmit(handle, &trans_desc);

	// ESP_LOGI(tag, "... Removing device.");
	// ESP_ERROR_CHECK(spi_bus_remove_device(handle));
    spi_bus_remove_device(handle);

	// ESP_LOGI(tag, "... Freeing bus.");
	// ESP_ERROR_CHECK(spi_bus_free(HSPI_HOST));
    spi_bus_free(HSPI_HOST);

	// ESP_LOGD(tag, "<< test_spi_task");
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

// uint8_t calibrate_elec(uint8_t elec_vn, uint8_t elec_vp, uint8_t elec_vn, uint8_t elec_vp...)

void printline_csv(struct Cycle_meas measurements, uint8_t num_elecs)
// Prints out csv formatted measurements
{
    printf("%d,%d", measurements.isrc_elec, measurements.isnk_elec);
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

    //Configure ADC
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);
    // adc_vref_to_gpio(ADC_UNIT_1, GPIO_NUM_34);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars); // Characterised using DEFAULT_VREF (factory?)

    // Assign initial mux electrode values
    uint8_t i_elecs = sel_mux_frmt(isnk_elec, isrc_elec);
    uint8_t v_elecs = sel_mux_frmt(vn_elec, vp_elec);
    uint8_t elec_index[2] = {i_elecs, v_elecs};

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

            uint8_t led_state = 0;

            // Cycle through each voltage measurement
            for (int elec_iter=0 ; elec_iter<NUM_ELECS ; elec_iter++)
            {
                // Store data in v_read
                struct Vmeas v_read;
                v_read.vp_elec = vp_elec;
                v_read.vn_elec = vn_elec;
                
                // Send SPI mux cmd
                send_spi_cmd(elec_index);

                //Multisampling
                uint32_t adc_reading = 0;
                for (int i = 0; i < NO_OF_SAMPLES; i++) {
                    // // Read internal ADC:
                    // adc_reading += adc1_get_raw((adc1_channel_t)channel); // ADC operates @ 6kHz
                    // Read external i2c ADC
                    adc_reading += do_i2cget_cmd(&ADC_chip_address);
                }
                adc_reading /= NO_OF_SAMPLES;

                //Convert adc_reading to voltage in mV
                // // for internal adc...  
                // v_read.voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars) - V_OFFSET;
                // for external adc...
                v_read.voltage = adc_reading * 1.255 - V_OFFSET;
                cycle_read.vm[vp_elec] = v_read;

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
                
                // Clear screen
                // for (int i=0 ; i<64 ; i++){
                //     printf("\b"); 
                // }

                // Print buffer
                // for (int i=0 ; i < NUM_ELECS ; i++) {
                //     printf(" v%X%X:%d",cycle_read.vm[i].vn_elec, cycle_read.vm[i].vp_elec, cycle_read.vm[i].voltage);
                // }
                // printf("\n");

                // printf("Raw: %d\tVoltage(%d-%d): %dmV\n", adc_reading, vn_elec, vp_elec, voltage - V_OFFSET);
                led_state = !led_state; // Toggle LED for fun
                gpio_set_level(EN_LED, led_state);
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            printline_csv(cycle_read, NUM_ELECS);
            
            if (ert_mode == ADJACENT) {
                // Adjacent mode: cycle through current source electrodes
                isnk_elec = iter_elec(cycle_dir, isnk_elec, NUM_ELECS);
                isrc_elec = iter_elec(cycle_dir, isrc_elec, NUM_ELECS);
                i_elecs = sel_mux_frmt(isnk_elec, isrc_elec);
                elec_index[0] = i_elecs;
                // send_spi_cmd(elec_index);
            }
        }
        gpio_set_level(EN_LED, 1);
    }
}

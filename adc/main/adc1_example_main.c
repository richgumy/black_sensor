/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include <driver/spi_master.h>
#include <esp_log.h>

#include "sdkconfig.h"

// MUX
#define EN_MUX_ISRC 5
#define EN_MUX_VGND 17
#define EN_MUX_VMEASP 4
#define EN_MUX_VMEASN 16
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<EN_MUX_ISRC) | (1ULL<<EN_MUX_VGND) | (1ULL<<EN_MUX_VMEASP) | (1ULL<<EN_MUX_VMEASN))

// ADC
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate?
#define NO_OF_SAMPLES   64          //Multisampling

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

// Electrodes
static uint8_t isrc_elec = 0;
static uint8_t isnk_elec = 4;
static uint8_t vp_elec = 1;
static uint8_t vn_elec = 2;
static const uint8_t num_elecs = 5;


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

void send_spi_cmd(uint8_t data[]) {
	// ESP_LOGD(tag, ">> test_spi_task");

	spi_bus_config_t bus_config;
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
	dev_config.spics_io_num = 15;
	dev_config.flags = 0;
	dev_config.queue_size = 1;
	dev_config.pre_cb = NULL;
	dev_config.post_cb = NULL;
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

// uint8_t* sel_mux(uint8_t isnk_elec, uint8_t isrc_elec, uint8_t vn_elec, uint8_t vp_elec) {
//     // input:a,b,c,d get turned into a hex value in the format 0xab and 0xcd
//     if ((isnk_elec > 15) || (isrc_elec > 15) || (vn_elec > 15) || (vp_elec > 15)){
//         printf("MUX array index out of bounds\n");
//     }
//     uint8_t data[2];
//     data[0] = (isnk_elec << 4) + isrc_elec;
//     data[1] = (vn_elec << 4) + vp_elec;
//     printf("0xab = 0x%X, 0xcd = 0x%X\n",data[0],data[1]);
//     return data;
// }

uint8_t sel_mux_frmt(uint8_t elec_1, uint8_t elec_2) {
    // Inputs elec_1 and elec2 get turned into a hex value in the format 0x(elec_1)(elec_2)
    if ((elec_1 > 15) || (elec_2 > 15)){
        printf("MUX array index out of bounds\n");
    }
    uint8_t data;
    data = (elec_1 << 4) + elec_2;
    return data;
}

uint8_t iter_elec(uint8_t increment, uint8_t elec_val, uint8_t num_elecs) {
    // Rolls up or down through electrode values
    if (increment) {
        return elec_val++ < num_elecs? elec_val : 0;
    }
    else {
        return elec_val-- >= 0? elec_val : num_elecs;
    }
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
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure GPIO
    setup_all_gpio();
    gpio_set_level(EN_MUX_ISRC, 0);
    gpio_set_level(EN_MUX_VGND, 0);
    gpio_set_level(EN_MUX_VMEASP, 0);
    gpio_set_level(EN_MUX_VMEASN, 0);

    //Configure ADC
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);
    // adc_vref_to_gpio(ADC_UNIT_1, GPIO_NUM_34);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars); // Characterised using DEFAULT_VREF (factory?)


    //Continuously sample ADC1
    while (1) {
        // Assign mux'd electrode values
        uint8_t i_elecs = sel_mux_frmt(isnk_elec, isrc_elec);
        uint8_t v_elecs = sel_mux_frmt(vn_elec, vp_elec);
        uint8_t data[2] = {i_elecs, v_elecs};
        printf("check1");
        send_spi_cmd(data);

        // Cycle through electrodes
        vp_elec = iter_elec(1, vp_elec, num_elecs);
        vn_elec = iter_elec(1, vn_elec, num_elecs);

        uint32_t adc_reading = 0;

        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        }
        adc_reading /= NO_OF_SAMPLES;

        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        uint32_t v_offset = 1663;
        printf("Raw: %d\tVoltage(%d-%d): %dmV\n", adc_reading, vn_elec, vp_elec, voltage - v_offset);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
}

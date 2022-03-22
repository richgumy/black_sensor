#include "esp32_spi.h"

int init_spi_channel( int spiCLK, int spiMOSI, int spiMISO) {
  esp_err_t err;
	spi_bus_config_t bus_config;
  memset(&bus_config, 0, sizeof(spi_bus_config_t)); // zero out bus_config;
	bus_config.sclk_io_num = spiCLK; // CLK
	bus_config.mosi_io_num = spiMOSI; // MOSI
	bus_config.miso_io_num = spiMISO; // MISO
	bus_config.quadwp_io_num = -1; // Not used
	bus_config.quadhd_io_num = -1; // Not used
  bus_config.max_transfer_sz = 4094;

  err = spi_bus_initialize(HSPI_HOST, &bus_config, 1);

  return err;
}

int init_spi_device(spi_device_handle_t &handle, uint8_t CS_pin) {
	esp_err_t err;
	spi_device_interface_config_t dev_config={};
	memset(&dev_config, 0, sizeof(dev_config)); // zero out dev_config;
	dev_config.address_bits = 0;
	dev_config.command_bits = 0;
	dev_config.dummy_bits = 0;
	dev_config.mode = 0;
	dev_config.cs_ena_pretrans = 0;
	dev_config.duty_cycle_pos = 0;
	dev_config.cs_ena_posttrans = 0;
	dev_config.clock_speed_hz = 600000;
	dev_config.input_delay_ns = 0;
	dev_config.spics_io_num = CS_pin;
	dev_config.flags = 0;
	dev_config.queue_size = 1;
	dev_config.pre_cb = NULL;
	dev_config.post_cb = NULL;

	err = spi_bus_add_device(HSPI_HOST, &dev_config, &handle);

	return err;
}

// int read_spi_data() {
//   static uint8_t data_in;
//   static uint8_t data_out;
// }

// int write_spi_data() {

// }

int send_spi_cmd(uint8_t data_in[], uint8_t data_out[], spi_device_handle_t handle) {
	esp_err_t err;
	spi_transaction_t trans_desc;
	memset(&trans_desc, 0, sizeof(trans_desc)); // zero out spi_trans;
	trans_desc.length = 2 * 8;
	trans_desc.rxlength = 2 * 8;
	trans_desc.tx_buffer = data_out;
	trans_desc.rx_buffer = data_in;
	trans_desc.user = (void*)0;
	err = spi_device_transmit(handle, &trans_desc);
	return err;
}

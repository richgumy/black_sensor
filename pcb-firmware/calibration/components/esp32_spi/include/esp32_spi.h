#include <stdio.h>
#include <string.h>
#include <driver/spi_master.h>
#include "sdkconfig.h"
#include "esp_system.h" // config ESP peripherals

#ifndef ESP32_SPI_H
    #define ESP32_SPI_H
    
    int init_spi_channel( int spiCLK, int spiMOSI, int spiMISO);
    
    
    int init_spi_device(spi_device_handle_t handle, uint8_t CS_pin);
    
    
    int send_spi_cmd(uint8_t data_in[], uint8_t data_out[], spi_device_handle_t handle);


    #endif /* ESP32_SPI_H */

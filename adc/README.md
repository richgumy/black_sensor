| Supported Targets | ESP32 |
| ----------------- | ----- |

# ADC1 Example

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This example shows how to configure ADC1 and read the voltage connected to GPIO pin.

## How to use example

### Hardware Required

* A development board with ESP32 SoC (e.g., ESP32-DevKitC, ESP-WROVER-KIT, etc.)
* A USB cable for power supply and programming

In this example, we use `ADC_UNIT_1` by default, we need to connect a voltage source (0 ~ 3.3v) to GPIO34. If another ADC unit is selected in your application, you need to change the GPIO pin (please refer to Chapter 4.11 of the `ESP32 Technical Reference Manual`).

### Configure the project

```
idf.py menuconfig
```

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.



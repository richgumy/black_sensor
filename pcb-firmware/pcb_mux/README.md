# _PCB MUX with SMU_

Code to multiplex a current source and voltage measurements for Electrical Impedance Tomography using an adjacent electrode current injection pattern using an ESP32 microcontroller. We are using a Keithely 2634b SMU, but this can easily be changed out for any programmable current source and voltage measuring unit. Use the pcb_mux directory above for programming the ESP32 using ESP-IDF. 

## Run an ERT collection cycle 

In a terminal in the pcb_mux directory run:

`> python eit_reader.py <filename> <format> <num_cycles> <Isrc_A> <nplc>`

filename = file name, not inlc. '.csv'

format = 'r' for raw data. Else output EIT reconstruction-friendly format

All other are optional and otherwise default to num_cycles = 15, Isrc_A = 1 mA, nplc = 0.01

<img src="https://github.com/richgumy/black_sensor/assets/14900898/cdfc24ae-b968-4236-8506-cd4593f8a3f5" width="500"/>

*Force applicator load order and locations*

TODO: Run a set of experiments with batch_run_exp.sh


## SMU-PCB-ESP Pin Connections

| **SMU Signal** | **MUX PCB Pin** | **Signal** | **ESP32 Pin** |
|:--------------:|:---------------:|:----------:|:-------------:|
|     smub hi    |      Out2 B     |    Isrc    |      N/A      |
|     smub lo    |      Out2 A     |    Isnk    |      N/A      |
|     smua hi    |      Out1 B     |  $\mathrm{Vmeas_p}$ |      N/A      |
|     smua lo    |      Out1 A     |  $\mathrm{Vmeas_n}$ |      N/A      |
|     smu gnd    |     GND & VSS   |     GND    |      GND      |
|       N/A      |       VDD       |  V_EXT/5V  |    V_EXT/5V   |
|       N/A      |      MUX_EN     |   MUX_EN   |      IO4      |
|       N/A      |      SPI_CS     |   MUX_CS   |      IO13     |
|       N/A      |     SPI_COPI    |  MUX_COPI  |      IO23     |
|       N/A      |     SPI_CLK     |   MUX_CLK  |      IO19     |
|       N/A      |       3V3       |     VCC    |      3V3      |

*Upload to an ESP32 microcontroller using ESP-IDF or similar*

## Command set
To change the state of the PCB MUX circuit it must receive certain characters over serial UART. 

Command prompts for the PCB_MUX setup sent over UART:

    i = iterate electrodes
using [typical EIT adjacent pattern](https://hal.science/hal-03370772/document)

    g = get current electrode state
responding via serial, the electrode number of current source and voltage measure electrodes

    c = get iteration count
responding via serial, the number of iterations since program start

## Intended Software Flow

![Pool lane diagram showing the parallel workflow of a PC, SMU, and ESP programs](/pcb-firmware/pcb_mux/PCB_MUX_SW_flow.jpg)


# _PCB MUX with SMU_

Code to multiplex a current source and voltage measurements for Electrical Impedance Tomography using an adjacent electrode current injection pattern. 

| **Signal** | **ESP-WROOM32 Pin** | **ESP32-C3 Pin** |
|------------|---------------------|------------------|
| MUX_EN     |          4          |         TBD        |
| MUX_CS     |          2          |         TBD        |
| MUX_COPI   |          13         |        TBD        |
| MUX_CLK    |          14         |        TBD        |
 
## Command set
To change the state of the PCB MUX circuit it must receive certain characters over serial UART. 

Command prompts for the PCB_MUX setup sent over UART:

    i = iterate electrodes (in typical EIT adjacent pattern)
    g = get current electrode state
    c = get iteration (reading) count

*Upload using ESP-IDF or similar*

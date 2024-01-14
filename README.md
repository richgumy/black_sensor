# Soft Surface Pressure Sensor using EIT
Black sensor is a project aimed at characterising carbon black silicone composites (and other piezoresistive conductive particle elastomer composites). This material can also be used as a pressure sensitive skin. The sensor system uses the principle of electrical impedance tomography (EIT) to reconstruct a resisitivty map of a flat sheet of piezoresisitive composite (characteristic limits of material yet to be determined. Such as min/max inter-electrode resisitvity and gauge factor) We will be only using DC as we are solely interested in the resisitive qualities of the composite for now, hence we're using electrical resistivity tomography (ERT). The sensor has been created that contains 4 major parts:

1) [ERT PCB hardware](/README.md#1---ert-pcb-hardware)
2) [ERT PCB firmware](/README.md#2---ert-pcb-firmware)
3) [Reconstruction software](/README.md#3---reconstruction-software)
4) [Sensor domain parts](/README.md#4---sensor-domain-parts)

A low cost PCB has been developed to capture the ERT data in reatime, which is streamed to a another computer to run a resistivity image reconstruction algorithm.


<img src="/media/architecture.png" width="600">

## 1 - ERT PCB hardware
The hardware driving the ERT sensor consist of either one custom ERT PCB or a MUX PCB used with an ESP32 development board and source measurement unit (SMU). All of the files to create the hardware are given as raw KiCad files. All of the component libraries may not exist so will need to made manually or imported.
[SMU PCB pinout here](/pcb-firmware/pcb_mux/README.md)
### 2.1 - ERT PCB
This board is for ERT data collection limited to sensor domains which have lower resisitances throughout the entirety of its intended use. The adjacent electrode resistance must not exceeed approx. 5 $k\Omega$.
#### Features:
- 5v USB power supply
- Onboard USB-UART programming
- Four 16:1 SPI MUXs
- Trimpot controlled current source
- Trimpot controlled voltage measurement gain
- 16 bit SPI ADC
- WiFi/BT capability
#### Future Features:
- Impedance measurements using network analyser
- Smaller PCB size
- Lower power MCU
### 2.2 - MUX PCB
This system consists of a SPI controlled 4:16 multiplexer (MUX) PCB used controlled with an ESP32 development board, PC and driven by a Keithley 2634b SMU and Keithley 2230G-30-1 DC power supply.
#### Features:
- Onboard USB-UART programming
- Four 16:1 SPI MUXs
- WiFi/BT capability
#### Future Features:
- Miniatruise and move all this system's functionality to the ERT PCB

## 2 - ERT PCB firmware
To run a test with the Cartesian force applicator see instructions in directory [here](pcb-firmware/pcb_mux/README.md)

### 2.1 - ERT PCB
The PCB firmware is all written in C for the ESP32-WROOM SoC connected to a custom 'ERT PCB'. The firmware applies an electrode pattern to the electrodes and sends measurement data via the USB UART serial connection.
The basic electrode drive process is:
1. Apply current to 2 electrodes
2. Measure voltage across 16 electrode pairs
3. Send voltage measurement data via serial
4. Iterate to next set of current electrodes.
5. Back to step 1.
#### Features:
- Drive modes: Standby, Calibrate, Adjacent
- Measurement averaging
- 16bit analog reading
#### Future Features:
- Bluetooth transmission of serial data
- More drive modes: Pseudo polar, PP-PP
- Nonense data checker (Throw error if data is very noisy or ADC saturated etc.)
- Speed up measurement rate from 8Hz to 40Hz

### 2.2 - MUX PCB - ESP32/PC
This is a more modular approach so that we are not limited by the ERT PCB's current source and power supply values. This system has five main components: A Keithley 2634b SMU, a MUX PCB, an ESP32 WROOM32 development board, a Keithley 2230G-30-1 DC power supply, and a piezoresistive sensor domain. The whole system is controlled via the PCB connected to the ESP32 and SMU over UART. 
The basic electrode drive process is:
1. Apply current to 2 electrodes
2. Measure voltage across 16 electrode pairs
3. Send voltage measurement data via serial
4. Iterate to next set of current electrodes.
5. Back to step 1.

These steps are done using a PC to send commands for switching the MUC PCB and query messages sent to the SMU to get voltage and current values.

#### Features:
- Simple serial control
- Voltage measurement time integration
- Nonense data checker (Throw error if SMU Isrc voltage saturating or DC power supply saturating)
- Logs the electrode resistance at beginning of cycle

#### Future Features:
- More drive modes: Pseudo polar, PP-PP
- Speed up measurement rate from 0.3Hz to ??Hz

## 3 - Reconstruction software
EIDORS was used as the library for image reconstruction using the data gathered from the PCB. A reference measurement is taken from the material on start-up of the reconstruction program. The reference measurement is compared to a current measurement for change in resisitvity (\Delta R) image reconstruction.

<img src="/media/testing.png" width="400">

### Features:
- Reconstructs image in realtime using serial data.
- Save images for video generation
- See EIDORS documentation for an extensive feature list: http://eidors3d.sourceforge.net/index.shtml
- Timestamped FEM and measurement data exported to .pkl files.

### Future Features:
- Optimise reconstruction by porting all functionality to C++ or for use with a TPU/GPU.

## 4 - Sensor domain mechanical parts
The rest of the physical system consists of:
1. Composite material under test (MUT)
2. MUT holder
3. MUT electrodes
The material we are using is a carbon black nanoparticle silicone rubber composite, with customisable properties depending on the fabrication process. The MUT holder is a 3D printed part that provides an ideal platform for the material to be tested on. The MUT electrodes maintain a reliable connection between the MUT and the ERT PCB.
### Features:
- 100mm diameter pressure test area
- Easily customisable for other MUT sizes and MUT materials within measurement range.
### Future Features:
- Standard test domains and MUT holder/electrode configurations


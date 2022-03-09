# Black Flat Sensor
Black flat sensor is a project aimed at characterising carbon black silicone composites (and other piezoresistive conductive elastomers). The sensor system uses the principle of electrical impedance tomography (EIT) to recreate a resisitivty map of flat shape of piezoresisitive composite. We will be only using DC as we are solely interested in the resisitive qualities of the composite for now, hence we're using electrical resistivity tomography (ERT). The sensor has been created that contains 4 major parts:

[1) ERT PCB hardware](/ 1 - ERT PCB hardware)
2) ERT PCB firmware
3) Reconstruction software
4) Sensor domain parts

A low cost PCB has been developed to capture the ERT data in reatime, which is streamed to a another computer to run a resistivity image reconstruction algorithm.

## 1 - ERT PCB hardware
All of the files to create the hardware are given as raw kicad files. All of the component libraries may not exist so will need to made manually or imported.
### Features:
- 5v USB power supply
- Onboard USB-UART programming
- Four 16:1 SPI MUXs
- Trimpot controlled current source
- Trimpot controlled voltage measurement gain
- 16 bit SPI ADC
- WiFi/BT capability
### Future Features:
- Impedance measurements using network analyser
- Smaller PCB size
- Lower power MCU

## 2 - ERT PCB firmware
The PCB firmware is all written in C for the ESP32-WROOM SoC. The firmware applies an electrode pattern to the electrodes and sends measurement data via the USB UART serial connection.
The basic electrode drive process is:
1. Apply current to 2 electrodes
2. Measure voltage across 16 electrode pairs
3. Send voltage measurement data via serial
4. Iterate to next set of current electrodes.
5. Back to step 1.
### Features:
- Drive modes: Standby, Calibrate, Adjacent
- Measurement averaging
- 16bit analog reading
### Future Features:
- Bluetooth transmission of serial data
- More drive modes: Pseudo polar, PP-PP
- Nonense data checker (Throw error if data is very noisy or ADC saturated etc.)
- Speed up measurement rate from 2Hz to 40Hz

## 3 - Reconstruction software
EIDORS was used as the library for image reconstruction using the data gathered from the PCB. A reference measurement is taken from the material on start-up of the reconstruction program. The reference measurement is compared to a current measurement for change in resisitvity (\Delta R) image reconstruction.
### Features:
- Reconstructs image in realtime using serial data.
- Save images for gif generation
- See EIDORS documentation for extensive feature list: http://eidors3d.sourceforge.net/index.shtml
### Future Features:
- Timestamped FEM and measurement data exported to CSV files.
- Optimise reconstruction by porting all functionality to C++ or for use with a TPU/GPU.

## 4 - Sensor domain parts
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

With all of this sensor in place and ready to obtain data the next step is to apply it to something!




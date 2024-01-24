# Force applicator EIT Experiment Operation Guide
This is a rough guide on how to set up a multi-load (nine-load) EIT experiment with the below setup.

<img src="https://github.com/richgumy/black_sensor/assets/14900898/7f64d337-f059-4093-b3b1-f14167e13c85" width="400"/>
<img src="https://github.com/richgumy/black_sensor/assets/14900898/e8aeedb7-f97e-46c6-aa1b-f5f7d5d6639e" width="250"/>

## 1 - Check Hardware
The hardware required for this experiment includes: a Keithley 2634b SMU, DC power supply ($\pm20 V$), ESP32 dev board, custom MUX PCB, sensing domain under test (DUT), 8-16 electrode harness, and the cartesian force applicator (CFA).

Connect the SMU, ESP32, DC power supply, and DUT as shown in the [SMU-PCB-ESP pin connection table](https://github.com/richgumy/black_sensor/tree/main/pcb-firmware/pcb_mux#smu-pcb-esp-pin-connections)

Connect the USB serial cables to the SMU, ESP32, and CFA (loadcell and 3d printer frame).

Attach each of the electrodes of the DUT to the MUX PCB cable harness. Ensure that the DUT is centered and the electrodes are aligned with the markings. Once all electrodes are connected you can test the inter-electrode resistance on the 16-pin header on the MUX PCB connected directly to each electrode or run ```eit_reader.py``` and check the output file using ```eit_reader_checker.py```.

<img src="https://github.com/richgumy/black_sensor/assets/14900898/9dc37324-ac0d-482a-8dc1-cd9f36d1c197" width="200"/>


## 2 - Edit Batch Script
To run a batch of experiments iterating through different strain values and repeating trials a bash script is used.

1. Open ```batch_run_exp.sh``` code

2. Edit the ```filepath``` and ```filename``` using descriptive naming. E.g. "DEA1_CBSR_8p_1_9push_strain_120s_1mA" tells us it's a 1 mm thick sample, is made from CBSR, is sample #1, the experiment is 9 pushes, the strain hold time is 120 s, and the current source is set to a constant 1 mA. **Please check you're not overwriting any existing file, there is no warning for this.**

3. If creating a new directory for ```filepath``` make an ```images``` sub-directory.

4. Set the strain string position index. I.e. where shall we put the strain string variable in the ```filename```

5. Edit the following line ```python eit_reader.py <*filename> <*Isrc_A> <Vmax> <*sample_name> <*date_fabricated> <*load_time_s> $i``` changing all asterisked values if necessary.


## 3 - Run Batch Script

1. Open git bash terminal
  
2. Enter ```./batch_run_exp.sh```

3. Watch the calibration sequence to check for crashes or wires snagging.

4. Leave it and relax :)

# Notes

## Force applicator looking like it will crash?
It has crash detection however this only kicks in after a reasonable amount of force has been applied. **Just power off the 3D printer frame immediately if needed.**

## Setting up a new computer?

If setting up a new control computer several things need to be done. 

You will need to obtain all of the python dependencies from the ```eit_cfa_reader.py``` code.

You will need to manually redefine the COM ports in the ```eit_cfa_reader.py``` code.

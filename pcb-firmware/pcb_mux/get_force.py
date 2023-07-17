"""
FILE: make_ert_data_csv.py
AUTHOR: R Ellingham
PROGRAM DESC: Gather serial data from ERT sensor in real time. The parameters measured are: 1. Voltage measurements in order from ERT system and 2. Time stamped the data when recieved by the program (Recieving timestamp from ERT sensor would slow measurement too much, but possible)

USAGE: python make_ert_data_csv.py csv_filename save_location_dir
E.G. "python make_ert_data_csv.py data_capture ../../data_folder/eit"

"""

import csv
from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *

from datetime import datetime
import time

## loadcell function parameters
scale_loadcell_G = 4.94462e+6
offset_loadcell_G = 70.13

def onVoltageRatioChange(self, voltageRatio):
    # print("VoltageRatio: " + str(voltageRatio))
    mass_g = voltageRatio*scale_loadcell_G + offset_loadcell_G
    self.voltage_V = voltageRatio
    self.mass_g = mass_g
    self.force_N = mass_g * 9.805 / 1000

def cal_loadcell(loadcell_handle):
    global offset_loadcell_G
    offsetv_buf = []
    buf_len = 30
    for i in range(buf_len):
        time.sleep(0.01)
        offsetv_buf.append(loadcell_handle.mass_g)
    offset_loadcell_G = offset_loadcell_G - sum(offsetv_buf)/buf_len
    print("loadcell cal'd")
    return offset_loadcell_G

def loadcell_init(loadcell_handle):
    ## Setup loadcell(bridge) device
    loadcell_handle.setOnVoltageRatioChangeHandler(onVoltageRatioChange)
    loadcell_handle.openWaitForAttachment(5000)
    loadcell_handle.setDataRate(8)
    print("loadcell init'd!")

def main(filename, save_loc):
    ## Setup loadcell(bridge) device
    loadcell = VoltageRatioInput()
    loadcell.setOnVoltageRatioChangeHandler(onVoltageRatioChange)
    loadcell.openWaitForAttachment(5000)
    loadcell.setDataRate(8)

    ## Setup CSV file
    if (len(filename) == 0): filename = input("Filename? ")
    
    filename = "force_" + filename # prepend 'force_'

    if (filename[-4:] != ".csv"): filename = filename+".csv"
    if (save_loc[-1] != '\\'): save_loc = save_loc+'\\'

    file_dir = save_loc + filename

    with open(file_dir, 'a', newline='') as csvfile:
        csv_data = csv.writer(csvfile, delimiter=',')
        csv_data.writerow(["UTC:", str(datetime.utcnow())])
        csv_data.writerow(["time_pc [s]", "Mass [g]"])

        # Zero time
        t_s = time.time()

        ## Gather data
        print("Gathering force data... push Ctrl+C to stop")
        while(1):
                time.sleep(0.08)
                # print(loadcell.mass_g)
                raw_data_timed = [time.time()-t_s] + [loadcell.mass_g]
                csv_data.writerow(raw_data_timed)

if __name__ == "__main__":
    import sys
    try:
        # Input parameters e.g. run >>python get_force.py expABCD_strain0.1 
        if len(sys.argv)>1: 
            input_filename = (sys.argv[1])
        else: 
            input_filename = ""
        if len(sys.argv)>2: 
            input_save_loc = (sys.argv[2])
        else: 
            input_save_loc = "."
        main(input_filename, input_save_loc)
    except Exception:
        print(traceback.format_exc())
    finally:
        sys.exit()

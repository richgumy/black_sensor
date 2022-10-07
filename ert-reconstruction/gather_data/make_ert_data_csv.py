"""
FILE: make_ert_data_csv.py
AUTHOR: R Ellingham
PROGRAM DESC: Gather serial data from ERT sensor in real time. The parameters measured are: 1. Voltage measurements in order from ERT system and 2. Time stamped the data when recieved by the program (Recieving timestamp from ERT sensor would slow measurement too much, but possible)

USAGE: python make_ert_data_csv.py csv_filename ERT_xcomport
E.G. "python make_ert_data_csv.py data_capture COM6"

"""

import csv
import serial
import serial.tools.list_ports as list_serial_ports
from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *

from datetime import datetime
import time

def onVoltageRatioChange(self, voltageRatio):
    # print("VoltageRatio: " + str(voltageRatio))
    mass_g = voltageRatio*4.94462e+6 + 70.13
    self.voltage_V = voltageRatio
    self.mass_g = mass_g

def list_serial_devices():
    """
    DESCR: Creates list of the names of all currrently connected serial comport devices
    IN_PARAMS: N/A
    OUTPUT: List of available comports
    NOTES: Requires serial.tools.list_ports library
    """
    print("Fetching list of devices...")
    avail_devs = list_serial_ports.comports()
    devs_list = []
    for i in range(len(avail_devs)):
        devs_list.append(avail_devs[i].device)
    return devs_list

def main(filename, comport):
    ## Setup loadcell(bridge) device
    loadcell = VoltageRatioInput()
    loadcell.setOnVoltageRatioChangeHandler(onVoltageRatioChange)
    loadcell.openWaitForAttachment(5000)
    loadcell.setDataRate(8)

    ## Setup serial connection
    if (len(comport) == 0):
        # get list of comports
        print("No comport specified. Select from list of comports below:")
        print(list_serial_devices())
        comport = "COM"+input("Comport num? COM")

    if (len(comport) == 4 or len(comport) == 5):
        print("{} chosen".format(comport))

    srl_dev = serial.Serial(comport,115200,timeout=2)
    print("Connecting to {}".format(comport)) 


    ## Setup CSV file
    if (len(filename) == 0): filename = input("Filename? ")
    
    if (filename[-4:] != ".csv"): filename = filename+".csv"

    with open(filename, 'a', newline='') as csvfile:
        csv_data = csv.writer(csvfile, delimiter=',')
        csv_data.writerow(["UTC:", str(datetime.utcnow())])
        csv_data.writerow(["time_pc [s]", "Mass [g]", "time_mcu[ms]", "data [mV]:"])

        # Zero time
        t_s = time.time()

        ## Gather data
        print("Gathering data... push Ctrl+C to stop")
        while(1):
                raw_data = srl_dev.readline().decode()[:-2].split(',')
                # print(loadcell.mass_g)
                raw_data_timed = [time.time()-t_s] + [loadcell.mass_g] + raw_data
                # raw_data_timed = [time.time()-t_s] + [0] + raw_data
                # print(loadcell.mass_g)
                csv_data.writerow(raw_data_timed)


if __name__ == "__main__":
    import sys
    try:
        # Input parameters
        if len(sys.argv)>1: 
            input_filename = (sys.argv[1])
        else: 
            input_filename = ""
        if len(sys.argv)>2: 
            input_comport = (sys.argv[2])
        else: 
            input_comport = ""
        main(input_filename, input_comport)
    except Exception:
        print(traceback.format_exc())
    finally:
        sys.exit()

"""
FILE: make_ert_data_csv_v2.py
AUTHOR: R Ellingham
PROGRAM DESC: Gather serial data from ERT sensor in real time. The parameters measured are: 1. Voltage measurements in order from ERT system and 2. Time stamped the data when recieved by the program (Recieving timestamp from ERT sensor would slow measurement too much, but possible)

USAGE: python make_ert_data_csv_v2.py csv_filename loadcell_comport ert_comport
E.G. "python make_ert_data_csv_v2.py data_capture COM11 COM6"

"""

import csv
import serial
import serial.tools.list_ports as list_serial_ports

from datetime import datetime
import time

def read_load_gf(serial_loadcell_handle):
    serial_loadcell_handle.write([1]) # Send flag for loadcell data request
    time.sleep(0.1)
    force_g = serial_loadcell_handle.readline()
    
    return force_g

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

def main(filename, comportERT, comportLoad):
    ## Setup loadcell(bridge) device serial connection
    if (len(comportLoad) == 0):
        # get list of comports
        print("No comport specified for the loadcell. Select from list of comports below:")
        print(list_serial_devices())
        comportLoad = "COM"+input("Loadcell comport num? COM")
    if (len(comportLoad) == 4 or len(comportLoad) == 5):
        print("{} chosen".format(comportLoad))
    else:
        print("INVALID COMPORT CHOSEN BEWARE")
    srl_dev_load = serial.Serial(comportLoad,115200,timeout=2)
    print("Connecting to {}".format(comportLoad)) 

    ## Setup ERT sensor serial connection
    if (len(comportERT) == 0):
        # get list of comports
        print("No comport specified for the ERT sensor. Select from list of comports below:")
        print(list_serial_devices())
        comportERT = "COM"+input("ERT comport num? COM")
    if (len(comportERT) == 4 or len(comportERT) == 5):
        print("{} chosen".format(comportERT))
    else:
        print("INVALID COMPORT CHOSEN BEWARE")
    srl_dev_ERT = serial.Serial(comportERT,115200,timeout=2)
    print("Connecting to {}".format(comportERT)) 


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
                # Read timestamped ERT data line by line
                raw_data_ERT = srl_dev_ERT.readline().decode()[:-2].split(',')
                
                ## Make serial read function for loadcell so serial buffer doesn't lag behind.
                if (raw_data_ERT[0] == 'A'):
                    raw_data_load = read_load_gf(srl_dev_load).decode()[:-2] # Do once per full ERT cycle read?
                else:
                    raw_data_load = 0
                
                # Aggregate data to write to CSV file
                raw_data_ERT_timed = [time.time()-t_s] + [raw_data_load] + raw_data_ERT
                csv_data.writerow(raw_data_ERT_timed)


if __name__ == "__main__":
    import sys
    print("Usage >>python make_ert_data_csv_v2.py <csv data filename> <loadcell comport> <ert comport>")
    try:
        # Input parameters
        if len(sys.argv)>1: 
            input_filename = (sys.argv[1])
        else: 
            input_filename = ""
        if len(sys.argv)>2: 
            input_comportERT = (sys.argv[2])
        else: 
            input_comportERT = ""
        if len(sys.argv)>3: 
            input_comportL = (sys.argv[3])
        else: 
            input_comportL = ""
        main(input_filename, input_comportERT, input_comportL)
    except Exception:
        print(traceback.format_exc())
    finally:
        sys.exit()

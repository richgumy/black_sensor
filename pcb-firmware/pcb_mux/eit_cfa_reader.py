"""
FILE: eit_cfa_reader.py
AUTHOR: R Ellingham
DATE CREATED: July 2023
DATE MODIFIED: July 2023
PROGRAM DESC: Gather EIT measurement then prompt via serial a mux device to iterate to the next 
step in an EIT read pattern. The program simultaneously reads the force being applied from the cartesian force applicator loadcell for each cycle.
Writes all data to a CSV file ready for analysis.

Use case: 
1) Enter in program directory cmd prompt:
    > python eit_reader.py <filename> <r=raw_data or leave blank for 16x16 format>
    * Additional command line arguments can be added to include measure cycles, current source and integration time using
    > python eit_reader.py <filename> <r=raw_data> <num_cycles> <Isrc_A> <nplc>
        otherwise default values of cycles=15, i_src_A=1e-3, nplc=0.01 will be used.
2) Push Ctrl+C to stop EIT reading, then filename.csv will be saved and plot full electrode cycle
    # Note - for this the SMU outputs will remain on
3) Push Ctrl+C again to close plot

To change SMU params go to '## Init smu parameters ##' in code.
"""

import csv
from datetime import datetime
from k2600 import K2600 # see k2600.py for usage
import k2600
import matplotlib
import matplotlib.pyplot as plt 
import numpy as np
import pyvisa
import serial
import serial.tools.list_ports
import time
from threading import Thread
import traceback

import eit_reader_checker
import get_force

def init_smu(smu_handle, i_src_A, v_meas_max_V=20, nplc=1, f_baud=115200):
    # init smu parameters
    smu_handle.serial.baud(smu_handle, f_baud)
    smu_handle.display.screen(smu_handle,smu_handle.display.SMUA_SMUB) # display both SMUs on screen
        # SMU b (Func:Isrc)
    smu_handle.smub.source.func(smu_handle,smu_handle.smub.OUTPUT_DCAMPS)
    smu_handle.smub.source.leveli(smu_handle,i_src_A)
    smu_handle.smub.source.limitv(smu_handle,v_meas_max_V)
    smu_handle.smub.measure.nplc(smu_handle,nplc)
        # SMU a (Func:Vmeas) 
    # smu_handle.smua.measure.func(smu_handle,smu_handle.smua.MEASURE_DCVOLTS)
    # smu_handle.smua.measure.nplc(smu_handle,nplc)
            # OR --> set as a 0A i_src for 4-wire setup??
    smu_handle.smua.source.func(smu_handle,smu_handle.smua.OUTPUT_DCAMPS)
    smu_handle.smua.source.leveli(smu_handle,0)
    smu_handle.smua.source.limitv(smu_handle,v_meas_max_V)
    smu_handle.smua.measure.nplc(smu_handle,nplc)

    smu_handle.smub.source.output(smu_handle,1) # turn on channel b
    smu_handle.smua.source.output(smu_handle,1) # turn on channel a

    smu_handle.beeper.enable(smu_handle,0) # turn off beeper 

    print("SMU init'd")

def query_pcbmux(serial_handle, cmd):
    # send one of the compatible commands: 'ITER', 'GET_STATE', or 'GET_ITER'
    # and print out response
    pcbmux_elec_cmds = {'ITER':'i','GET_STATE':'g','GET_ITER':'i'}
    msg = bytes(pcbmux_elec_cmds[cmd],'utf-8')
    error = serial_handle.write(msg)
    reply = serial_handle.read() # takes 0.0140s for 115200 baud
    if reply == b'i':
        return error
    while (reply != b''):
        print(reply)
        reply = serial_handle.readline()
    return error

def write_pcbmux(serial_handle, cmd):
    # send one of the compatible commands: 'ITER', 'GET_STATE', or 'GET_ITER'
    pcbmux_elec_cmds = {'ITER':'i','GET_STATE':'g','GET_ITER':'i'}
    msg = bytes(pcbmux_elec_cmds[cmd],'utf-8')
    error = serial_handle.write(msg)
    return error

def main(t_buf, v_buf, i_buf, cycles=15, i_src_A=1e-3, nplc=0.01):
    ## 0. setup system
    eit_count = 0 # EIT iteration count
    fs_mx = 1000 # max vmeas frequency (limited by how fast the PCB MUX can be MUX'd via serial comms)

    # setup SMU connection
    rm = pyvisa.ResourceManager()
    available_devs = rm.list_resources()
    smu = K2600(available_devs[0])
    ## Init smu parameters ## 
    v_meas_max_V = 20
    init_smu(smu, i_src_A, v_meas_max_V, nplc, uart_baud)

    # setup PCB serial connection
    ser = serial.Serial(comport,uart_baud,timeout=0.01)
    ser.set_buffer_size(rx_size=256, tx_size=256)
    query_pcbmux(ser,'GET_STATE')
    query_pcbmux(ser,'GET_ITER')

    # start of measurements time
    ti = time.time()
    tsamp = 1/fs_mx
    for i in range(cycles*256):
        if (tsamp < 1/fs_mx):
            time.sleep(1/fs_mx - tsamp)
        ## 1. take voltage reading
        t_si = time.time()
        v_meas_V = smu.smua.measure.v(smu)
        if abs(float(v_meas_V)) > 0.9*v_meas_max_V:
            v_meas_V = f"{v_meas_V} MAX VOLTAGE ERROR"
            print(v_meas_V)
            input("Press enter to continue")
        if i % 16 == 0:
            i_src_act_A = smu.smub.measure.i(smu)
        else:
            i_src_act_A = 0
        i_buf.append(i_src_act_A)
        v_buf.append(v_meas_V)
        ts = time.time()
        tsamp = ts - t_si
        tstamp = ts - ti # sample time stamp
        
        t_buf.append(tstamp)

        ## 2. iterate mux
        query_pcbmux(ser,'ITER')
        eit_count = eit_count + 1
        tmuxf = time.time()
        tdmux = tmuxf - ts
        if not i % 256:
            print(f"cycle {i//256}")
            print("tsamp="+str(tsamp))
            print("tdmux="+str(tdmux))
        
    # close smu connection
    smu.disconnect()

if __name__ == "__main__":
    import sys
    input_filename = ""
    csv_frmt = 0
    eit_pattern_len = 256
    
    # set pcbmux serial params
    comport = 'COM3'
    uart_baud = 115200

    # set buffers for storing all recorded data
    v_buf = [] # EIT voltage readings
    i_buf = [] # EIT actual Isrc current
    t_buf = [] # EIT PC timestamps
    # default vals
    cycles = 10 
    i_src_A = 1e-3
    nplc = 0.01

    try:
        # Input parameters e.g. run >>python eit_reader.py expABCD_strain0.1 r
        if len(sys.argv)>1: 
            input_filename = sys.argv[1] + '.csv'

        else: 
            input_filename = ""
        if len(sys.argv)>2: 
            csv_frmt = sys.argv[2] # raw voltage and timestamps
        else: 
            input_filename = input_filename + '_eitfmt.csv' # EIT reconstruction friendly format
        
        # Important! the following must be in order  
        if len(sys.argv)>3:
            cycles = int(sys.argv[3])
        if len(sys.argv)>4:
            i_src_A = float(sys.argv[4])
        if len(sys.argv)>5:
            nplc = float(sys.argv[5])
        # Run programs!
        Experimental threading below!
        thread1 = Thread(target=main(t_buf, v_buf, i_buf, cycles, i_src_A, nplc))
        thread2 = Thread(target=get_force.main(f"{input_filename}",''))
        thread1.start()
        thread2.start()
        thread1.join()
        thread2.join()


    except Exception:
        print(traceback.format_exc())

    finally:
        print("CSV file saving...")
        with open(input_filename, 'a', newline='') as csvfile:
            csv_data = csv.writer(csvfile, delimiter=',')
            csv_data.writerow(["UTC:", str(datetime.utcnow())])
            # likely that t_buf and v_buf will not by the same size so ensure they are.
            max_len = 0
            if len(v_buf) > len(t_buf):
                max_len = len(t_buf)
            else:
                max_len = len(v_buf)
                
            if (csv_frmt == 'r'):
                # raw timestamped voltage stream data
                csv_data.writerow(["time_pc [s]", "voltage [V]", "i_src [A]"])
                csv_data.writerows(np.transpose([t_buf[0:max_len],v_buf[0:max_len],i_buf[0:max_len]]))
                # plt.plot(list(map(float,v_buf[0:eit_pattern_len-1])))
                # plt.show()                
            else:
                # reformat CSV file to be readable by EIDORS
                n_cycles = max_len // eit_pattern_len
                data_formatd = np.zeros((eit_pattern_len+1, n_cycles))
                for i in range(n_cycles):
                    data_formatd[0,i] = t_buf[i*eit_pattern_len + int(eit_pattern_len/2)]
                    data_formatd[1:eit_pattern_len+1,i] = v_buf[i*eit_pattern_len:(i+1)*eit_pattern_len]
                csv_data.writerows(data_formatd)
                # plt.plot(data_formatd[1:,0])
                # plt.show()
        eit_reader_checker.main(input_filename,i_src_A) # print out report
        sys.exit()
        



"""
FILE: eit_cfa_reader.py
AUTHOR: R Ellingham
DATE CREATED: May 2023
DATE MODIFIED: July 2023
PROGRAM DESC: Gather EIT and force measurements. Writes the data to a CSV file ready for analysis.

USE CASE: 
1) Enter in program directory cmd prompt:
    > python eit_reader.py <filename>
    
    * Additional command line arguments can be added to include measure cycles, current source and integration time using
        > python eit_reader.py <filename> <num_cycles> <Isrc_A> <nplc>
    otherwise default values of cycles=15, i_src_A=1e-3, nplc=0.01 will be used.

2) Push Ctrl+C to stop EIT reading, then filename.csv will be saved and plot full electrode cycle
    # Note - for this the SMU outputs will remain on
3) Push Ctrl+C again to close plot
"""

import csv
from datetime import datetime
import matplotlib
import matplotlib.pyplot as plt 
import numpy as np
from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *
import pickle as pkl
import pyvisa
import serial
import serial.tools.list_ports
from threading import Thread
import time
import traceback

import eit_reader_checker
from eitf_dataframe import *
from k2600 import K2600 # see k2600.py for usage
import k2600

## loadcell function parameters
scale_loadcell_G = 4.94462e+6
offset_loadcell_G = 70.13

## Test start ref
start_time_G = time.time()

def onVoltageRatioChange(self, voltageRatio):
    # print("VoltageRatio: " + str(voltageRatio))
    mass_g = voltageRatio*scale_loadcell_G + offset_loadcell_G
    self.voltage_V = voltageRatio
    self.mass_g = mass_g
    self.force_N = mass_g * 9.805

def cal_cfa(loadcell_handle):
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

def get_force(loadcell_handle):
    # re-define stop_flag
    global stop_flag
    # re-define data buffers
    global f_buf_N
    global tf_buf_s
    print('may the force be ... measured')
    while(not stop_flag):
        tf_buf_s.append(time.time()-start_time_G)
        f_buf_N.append(loadcell_handle.force_N)
        time.sleep(0.001)
    print(f"final force={loadcell_handle.force_N:.2e}N")

## smu functions
def init_smu(smu_handle, i_src_A, v_meas_max_V=20, nplc=0.01, f_baud=115200):
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

def read_eit_data(smu_handle,ser_handle,fs_max,v_meas_max_V,num_elecs=16):
    eit_count = 0 # EIT iteration count    # re-define stop flag
    global stop_flag
    # define global buffers
    global v_buf_V
    global tv_buf_s
    tsamp = 1/fs_max
    for i in range(cycles*num_elecs**2):
        if (tsamp < 1/fs_max):
            time.sleep(1/fs_max - tsamp)
        ## 1. take voltage reading
        t_si = time.time()
        v_meas_V = smu_handle.smua.measure.v(smu_handle)
        if abs(float(v_meas_V)) > 0.9*v_meas_max_V:
            v_meas_V = f"{v_meas_V} MAX VOLTAGE ERROR"
            print(v_meas_V)
            input("Press enter to continue")
        if i % num_elecs == 0:
            i_src_act_A = smu_handle.smub.measure.i(smu_handle)
        else:
            i_src_act_A = np.nan
        i_buf_A.append(i_src_act_A)
        v_buf_V.append(v_meas_V)
        ts = time.time()
        tsamp = ts - t_si # sample time
        tstamp = ts - start_time_G # sample timestamp
        
        tv_buf_s.append(tstamp)

        ## 2. iterate mux
        query_pcbmux(ser_handle,'ITER')
        eit_count = eit_count + 1
        tmuxf = time.time()
        tdmux = tmuxf - ts
        if not i % num_elecs**2:
            print(f"cycle {i//num_elecs**2}")
            print("t_samp="+str(tsamp))
            print("t_mux="+str(tdmux))
    stop_flag = True

def main(t_buf, v_buf, i_buf, cycles, i_src_A, nplc, v_max_V, num_elecs=16):
    ## 0. setup system
    fs_max = 1000 # max Vmeas sample frequency (limited by how fast the PCB MUX can be MUX'd via serial SPI comms)

    # setup SMU connection
    rm = pyvisa.ResourceManager()
    available_devs = rm.list_resources()
    smu = K2600(available_devs[0])
    v_meas_max_V = 20
    init_smu(smu, i_src_A, v_meas_max_V, nplc, uart_baud)

    # setup mux PCB serial connection
    ser = serial.Serial(comport,uart_baud,timeout=0.1)
    ser.set_buffer_size(rx_size=256, tx_size=256)
    query_pcbmux(ser,'GET_STATE')
    query_pcbmux(ser,'GET_ITER')

    # setup force measurement
    loadcell = VoltageRatioInput()
    loadcell_init(loadcell)
    cal_cfa(loadcell)

    # COMPLETE EIT & FORCE MEASUREMENTS CONCURRENTLY
    start_time_G = time.time() # global reference start time
    eit_thread = Thread(target=read_eit_data, args=(smu, ser, fs_max, v_meas_max_V))
    force_thread = Thread(target=get_force, args=(loadcell,))
    eit_thread.start()
    force_thread.start()
    eit_thread.join()
    force_thread.join()
        
    # close smu connection
    smu.disconnect()



if __name__ == "__main__":
    import sys
    # set default data collection params
    cycles = 10
    i_src_A = 1e-3
    nplc = 0.01
    v_max_V = 20

    stop_flag = False
    
    # set pcbmux serial params
    comport = 'COM8'
    uart_baud = 115200

    # set buffers for storing all recorded data
        # EIT
    v_buf_V = [] # EIT voltage readings
    i_buf_A = [] # EIT actual Isrc current
    tv_buf_s = [] # EIT PC timestamps
        # force
    f_buf_N = []
    f_buf_intp_N = []
    tf_buf_s = []
        # tool center point location xyz
    tcp_buf_mm = []
    ttcp_buf_s = []

    try:
        if len(sys.argv)==1:
            raise Exception("\n\nPlease retry using the terminal format:\n\t>python eit_reader.py <filename> optional:<num_cycles> <Isrc_A> <nplc>\n") 

        # set input sample details
        sample_name = input("what is your sample name? (e.g. CBSR_9p_1 or rGOSR_pcb_1) ")
        if sample_name[0:4] == 'CBSR':
            th_dim_mm = 4
            dia_dim_mm = 100
            fab_date = input("Input sample fabrication date if known? (in format DD-MM-YY): ")
        elif sample_name[0:5] == 'rGOSR':
            th_dim_mm = 3
            d_dim_mm = 100
            fab_date = '27-05-22'
        else:
            th_dim_mm = input("Unknown sample disk dimensions. What thickness in mm? ")
            dia_dim_mm = input("What diameter in mm? ")
            fab_date = input("Input fabrication date if known? (in format DD-MM-YY):")
        
        # Run program with cmd line arguments
        if len(sys.argv)>1: 
            input_filename = sys.argv[1] + '.csv'
        if len(sys.argv)>2:
            cycles = int(sys.argv[2])
        if len(sys.argv)>3:
            i_src_A = float(sys.argv[3])
        if len(sys.argv)>4:
            nplc = float(sys.argv[4])
        if len(sys.argv)>5:
            v_max_V = float(sys.argv[5])

        date_time_start = str(datetime.utcnow())

        main(tv_buf_s, v_buf_V, i_buf_A, cycles, i_src_A, nplc, v_max_V)

    except Exception:
        print(traceback.format_exc())

    finally:
        # interpolate force data 
        if len(tf_buf_s) > len(tv_buf_s):
            f_buf_intp_N = np.interp(tv_buf_s, tf_buf_s, f_buf_N)
        else:
            f_buf_intp_N = np.interp(tv_buf_s, tf_buf_s, f_buf_N)
            print('force reading too slow and will be downsampled!')
            
        print("CSV file saving...")
        with open(input_filename, 'a', newline='') as csvfile:
            csv_data = csv.writer(csvfile, delimiter=',')

            csv_data.writerow(["UTC:",date_time_start])
            
            # cut off unsync'd data
            max_len = 0
            if len(v_buf_V) > len(tv_buf_s):
                max_len = len(tv_buf_s)
            else:
                max_len = len(v_buf_V)

            # write data to .csv file
            csv_data.writerow(["time_pc [s]", "voltage [V]", "i_src [A]","f [N]"])
            csv_data.writerows(np.transpose([tv_buf_s[0:max_len],v_buf_V[0:max_len],i_buf_A[0:max_len],f_buf_intp_N[0:max_len]])) 
        
        # print out results report
        r_flag, vmax_flag = eit_reader_checker.report(input_filename,i_src_A) 
        _, r_adj_mean, _ = eit_reader_checker.get_inter_elec_res(v_buf_V, i_src_A)
        
        # save all params to .pkl file of same name
        eit_sample = PiezoResSample(sample_name, th_dim_mm, dia_dim_mm, fab_date)
        eit_test = EITFDataFrame(input_filename, date_time_start, v_buf_V, i_buf_A, tv_buf_s, i_src_A, v_max_V, 
                                 nplc, cycles, r_adj_mean, PiezoResSample, f_data_N=f_buf_intp_N, r_adj_error=r_flag, 
                                 v_max_error=vmax_flag)
        with open(input_filename[0:-4]+".pkl","wb") as fp:
            pkl.dump(eit_test,fp)

        sys.exit()
        



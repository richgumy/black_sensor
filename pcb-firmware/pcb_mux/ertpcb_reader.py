"""
FILE: ertpcb_reader.py
AUTHOR: R Ellingham
DATE CREATED: May 2024
DATE MODIFIED: May 2024
PROGRAM DESC: Gather ERT voltage measurements using PCB. Writes the data to a CSV file ready for analysis.

Use case: 
1) Enter in program directory cmd prompt:
    > python ertpcb_reader.py <filename>
    
    * Additional command line arguments can be added to include measure cycles, current source and integration time using
        > python ertpcb_reader.py <filename> <num_cycles> <Isrc_A> <nplc>
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
import pickle as pkl
import pyvisa
import serial
import serial.tools.list_ports
import time
import traceback

import eit_reader_checker
from eitf_dataframe import *


## eit data functions
def Vadc2Vreal(Vadc,gain=0.232,Vref=5,n_adc=16):
    '''
    Description
    -----------
    Converts raw n-bit ADC value to input voltage for ERT PCB v1.3    

    Parameters
    ----------
    Vadc : int
        raw bit voltage data
    gain: float
        expected total gain of opamp circuit
    Vref: float
        ADC voltage reference
    n_adc: int
        ADC resolution (n bits)

    Returns
    -------
    None.

    '''
    return Vadc * Vref / (2**n_adc * gain)

def vmeas_data_error(v_meas, v_meas_max_V):
    '''
    Description
    -----------
    Checks for serial data errors.

    Parameters
    ----------
    v_meas: serial data

    v_meas_max_V: maximum expected v_meas value

    Returns
    -------
    1 : error
    0 : data ok

    '''
    if v_meas.isdigit() and (Vadc2Vreal(float(v_meas)) > 0.999*v_meas_max_V):
        v_meas = f"{int(v_meas)} MAX VOLTAGE ERROR"
    elif v_meas.isdigit() and (int(v_meas) > 2**16):
        v_meas = f"{int(v_meas)} ABOVE 16BIT ERROR"
    elif not v_meas.isdigit():
        v_meas = f"{v_meas} NOT DIGIT ERROR"
    else:
        return 0
    # print(v_meas)
    return 1

def log_eit_data(ser_handle, tot_cycles=15, v_meas_max_V=20, i_src_A=1e-3, num_elecs=16):
    '''
    Description
    -----------
    Writes voltage, and timestamp values to global buffers v_buf,voltages have units 'bits'
    Execution is dis/enabled by an external thread raising the stop_flag. 

    Parameters
    ----------
    serial_handle : obj
        Serial connection object.

    Returns
    -------
    None.

    '''
    eit_cycles = 0 # EIT iteration count
    # define global buffers
    global i_buf_A
    global v_buf_bit
    global tv_buf_s
    # i = 0
    # i_frame = 0
    v_meas = ''

    print("LISTENING FOR ERT PCB...")
    while v_meas != b'A':
        if ser_handle.in_waiting > 0:
            v_meas = ser_handle.read_until(b'\r')[0:-1]
            print(v_meas)
    
    print("BEGINNING ERT DATA CAPTURE!")
    while eit_cycles < tot_cycles:
        ## 1. take voltage reading
        t_start = time.time()
        # get a frame
        i_meas = 0
        bad_data_flag = 0
        v_buf_frame_bit = []
        tv_buf_frame_s = []
        v_meas = ''
        while v_meas != b'A' and not bad_data_flag:
            if ser_handle.in_waiting > 0:
                v_meas = ser_handle.read_until(b'\r')[0:-1]
                # print(v_meas)
                tv = time.time()
                tv_stamp = tv - t_start # sample timestamp
            if v_meas == b'A':
                _ = 0
            elif vmeas_data_error(v_meas, v_meas_max_V) or (i_meas > num_elecs**2):
                bad_data_flag = 1
            else:
                v_buf_frame_bit.append(v_meas)
                tv_buf_frame_s.append(tv_stamp)
            i_meas += 1
        print(len(v_buf_frame_bit),bad_data_flag)
        if len(v_buf_frame_bit) == 257:
            print(v_buf_frame_bit)
            time.sleep(0.5)
        if (not bad_data_flag) and (len(v_buf_frame_bit) == num_elecs**2):
            for v in range(len(v_buf_frame_bit)):
                v_buf_bit.append((Vadc2Vreal(float(v_buf_frame_bit[v]))))
                tv_buf_s.append(float(tv_buf_frame_s[v]))
                i_buf_A.append(i_src_A)
            eit_cycles += 1
            print(f"ERT cycle {eit_cycles}")
        else:
            _ = 0
            # print(f"bad_data_flag={bad_data_flag} len(v_buf_frame_bit)={len(v_buf_frame_bit)}")

        # if ser_handle.in_waiting > 0:
        #   v_meas = ser_handle.read_until(b'\r')[0:-1]
        #   i_frame += 1

        #   if v_meas == b'A': # 'A' signals a new frame
        #     # print(f"{i_frame} i_frame")
        #     if i_frame != num_elecs**2: # check for any dropped error messages
        #         # remove partial frames
        #         tv_buf_s = tv_buf_s[0:-i_frame]
        #         frame_offset_err = len(tv_buf_s) % num_elecs**2

        #         if frame_offset_err != 0:
        #             # raise TypeError("Oops, ERT serial dropout detector not working!")
        #             print("ERT serial dropout detector not working!")
        #             print(f"frame offset error: {frame_offset_err}")
        #             print(f"i_frame {i_frame}")
        #             print(f"length of v_buf = {len(tv_buf_s)}")
        #             print("attempting to fix...")
                    
        #         tv_buf_s = tv_buf_s[0:-(i_frame+frame_offset_err)]
        #         i_buf_A = i_buf_A[0:-(i_frame+frame_offset_err)]
        #         v_buf_bit = v_buf_bit[0:-(i_frame+frame_offset_err)]
        #         eit_cycles -= (i_frame+frame_offset_err // num_elecs**2) + 1
        #         v_meas = b'' 
        #         while v_meas != b'A':
        #             if ser_handle.in_waiting > 0:
        #                 v_meas = ser_handle.read_until(b'\r')[0:-1]
        #     i_frame = 0
        #     v_meas = ser_handle.read_until(b'\r')[0:-1]

        # if v_meas.isdigit() and (Vadc2Vreal(float(v_meas)) > 0.995*v_meas_max_V):
        #     v_meas = f"{int(v_meas)} MAX VOLTAGE ERROR"
        #     print(v_meas)
        # elif not v_meas.isdigit():
        #     v_meas = f"{v_meas} NOT DIGIT ERROR"
        #     print(v_meas)

        # tv_buf_s.append(float(tstamp))
        # i_src_act_A = i_src_A
        # i_buf_A.append(i_src_act_A)
        # v_buf_bit.append((Vadc2Vreal(float(v_meas))))

        # if not i % num_elecs**2:
        #     # print(f"ERT cycle {i//num_elecs**2}")
        #     eit_cycles += 1
        # i += 1

def main(tv_buf_s, v_buf_bit, i_buf_A, tot_cycles, i_src_A, nplc, v_max_V, num_elecs=16):
    # setup mux PCB serial connection
    ser = serial.Serial(comport,uart_baud,timeout=0.1)
    ser.set_buffer_size(rx_size=1024, tx_size=1024)

    # start ert logging
    log_eit_data(ser,tot_cycles)
        


if __name__ == "__main__":
    import sys
    # set default data collection params
    tot_cycles = 15
    i_src_A = 1e-3
    nplc = 0.01
    v_max_V = 22
   
    # set pcbmux serial params
    comport = 'COM5'
    uart_baud = 115200

    # set buffers for storing all recorded data
    v_buf_bit = [] # EIT voltage readings
    i_buf_A = [] # EIT actual Isrc current
    tv_buf_s = [] # EIT PC timestamps
    
    try:
        if len(sys.argv)==1:
            raise Exception("\n\nPlease retry using the terminal format:\n\t>python eit_reader.py <filename> optional:<num_cycles> <Isrc_A> <nplc>\n") 
        date_time_start = str(datetime.now())

        # set input sample details
        sample_name = input("what is your sample name? (e.g. CBSR_9p_1 or rGOSR_pcb_1) ")
        if sample_name[0:4] == 'CBSR':
            th_dim_mm = 4
            dia_dim_mm = 100
            fab_date = input("Input fabrication date if known? (in format DD-MM-YY): ")
        elif sample_name[0:5] == 'rGOSR':
            th_dim_mm = 3
            d_dim_mm = 100
            fab_date = '27-05-22'
        else:
            th_dim_mm = input("Unknown sample disk dimensions. What thickness in mm? ")
            dia_dim_mm = input("What diameter in mm? ")
            fab_date = input("Input sample fabrication date if known? (in format DD-MM-YY):")
        
        # Run program with cmd line arguments
        if len(sys.argv)>1: 
            input_filename = sys.argv[1] + '.csv'
        if len(sys.argv)>2:
            tot_cycles = int(sys.argv[2])
        if len(sys.argv)>3:
            i_src_A = float(sys.argv[3])
        if len(sys.argv)>4:
            nplc = float(sys.argv[4])
        if len(sys.argv)>5:
            v_max_V = float(sys.argv[5])
        main(tv_buf_s, v_buf_bit, i_buf_A, tot_cycles, i_src_A, nplc, v_max_V)
        
    except Exception:
        print(traceback.format_exc())

    finally:
        print("CSV file saving...")
        with open(input_filename, 'a', newline='') as csvfile:
            csv_data = csv.writer(csvfile, delimiter=',')

            csv_data.writerow(["UTC:",date_time_start])
            
            # cut off unsync'd data
            max_len = 0
            if len(v_buf_bit) > len(tv_buf_s):
                max_len = len(tv_buf_s)
            else:
                max_len = len(v_buf_bit)

            # write data to .csv file
            csv_data.writerow(["time_pc [s]", "voltage [V]", "i_src [A]"])
            csv_data.writerows(np.transpose([tv_buf_s[0:max_len],v_buf_bit[0:max_len],i_buf_A[0:max_len]])) 
        
        # print out results report
        r_flag, vmax_flag = eit_reader_checker.report(input_filename,i_src_A) 
        _, r_adj_mean, _ = eit_reader_checker.get_inter_elec_res(v_buf_bit, i_src_A)
        
        # save all params to .pkl file of same name
        eit_sample = PiezoResSample(sample_name, th_dim_mm, dia_dim_mm, fab_date)
        eit_test = EITFDataFrame(input_filename, date_time_start, v_buf_bit, i_buf_A, tv_buf_s, i_src_A, v_max_V, 
                                 nplc, tot_cycles, r_adj_mean, PiezoResSample, r_adj_error=r_flag, v_max_error=vmax_flag)
        with open(input_filename[0:-4]+".pkl","wb") as fp:
            pkl.dump(eit_test,fp)

        sys.exit()
        



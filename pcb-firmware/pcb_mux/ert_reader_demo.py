import csv
from datetime import datetime
import matplotlib.pyplot as plt 
import serial
import sys
import time


eit_cycles = 0 # EIT iteration count
v_meas = 0
v_meas_prev = 0

tot_cycles = 100
i_src_A = 1e-3

# set pcbmux serial params
comport = 'COM3'
# uart_baud = 115200
uart_baud = 1400000
ser_handle = serial.Serial(comport, uart_baud, timeout=0.1)

# set buffers for storing all recorded data
v_buf_V = [] # EIT voltage readings
i_buf_A = [] # EIT actual Isrc current
tv_buf_s = [] # EIT PC timestamps

tv_stamp_prev = 0

with open('test.csv', 'a', newline='') as csvfile:
    csv_data = csv.writer(csvfile, delimiter=',')
    csv_data.writerow(["UTC:", str(datetime.now())])
    csv_data.writerow(["time_pc [s]", "voltage [V]", "i_src [A]"])

    print("LISTENING FOR ERT PCB...")
    while v_meas != b'A':
        if ser_handle.in_waiting > 0:
            v_meas = ser_handle.read_until(b'\r')[0:-1]
            v_meas_int = int.from_bytes(v_meas, byteorder=sys.byteorder)
            print(v_meas_int)
    
    t_start = time.time()
    while eit_cycles/256 < tot_cycles:
        # Get first V reading
        if ser_handle.in_waiting > 0:
            tsamp_s = time.time()
            v_meas_int_prev = v_meas_int
            v_meas = ser_handle.read_until(b'\r')[0:-1]
            v_meas_int = int.from_bytes(v_meas, byteorder=sys.byteorder)
            # print(v_meas, eit_cycles)
            if not (eit_cycles % 256):
                tv_stamp = time.time() - t_start # sample timestamp
                tv_stamp_prev = tv_stamp
            else:
                tv_stamp = tv_stamp_prev
            tv_stamp = 0
            
            eit_cycles += 1
        
            if v_meas != b'A' and ((v_meas_int != v_meas_int_prev) or (not v_meas_int)):
                # write data to .csv file
                csv_data.writerow([tv_stamp,float(v_meas_int),i_src_A])

print(100/(time.time() - t_start))




            




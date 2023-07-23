"""
FILE: eit_cfa_reader.py
AUTHOR: R Ellingham
DATE CREATED: May 2023
DATE MODIFIED: July 2023
PROGRAM DESC: Gather EIT and force measurements. Writes the data to a CSV file ready for analysis.

USE CASE: 
1) Enter in program directory cmd prompt:
    > python eit_reader.py <filename>
    
    * Additional command line arguments can be added to include current source and integration time using
        > python eit_reader.py <filename> <Isrc_A> <nplc>
    otherwise default values of i_src_A=1e-3, nplc=0.01 will be used.

2) Push Ctrl+C to stop EIT reading, then filename.csv will be saved and plot full electrode cycle
    # Note - for this the SMU outputs will remain on
3) Push Ctrl+C again to close plot
"""

import csv
from datetime import datetime
import matplotlib
import matplotlib.pyplot as plt 
from multiprocessing import Process, Queue # delete above line if using this works. Use queue to ensure variables can be shared between processes??
import numpy as np
from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *
import pickle as pkl
import pyvisa
import re
import serial
import serial.tools.list_ports
from threading import Thread
import time
import traceback

import eit_reader_checker
from eitf_dataframe import *
from get_force import *
from k2600 import K2600 # see k2600.py for usage
import k2600


# init globals
start_time_G = time.time() # reference time
max_accel_G = 200 # mm/s^2
    # global global coordinates
x_G = 0 # mm
y_G = 0
z_G = 0


## loadcell functions
def log_force_N(loadcell_handle):
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


## smu/eit functions
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

def log_eit_data(smu_handle,ser_handle,fs_max,v_meas_max_V,num_elecs=16):
    global eit_cycles # EIT iteration count
    global stop_flag
    # define global buffers
    global v_buf_V
    global tv_buf_s
    tsamp = 1/fs_max
    i = 0
    while not stop_flag:
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
        
        tmuxf = time.time()
        tdmux = tmuxf - ts
        if not i % num_elecs**2:
            print(f"cycle {i//num_elecs**2}")
            print("t_samp="+str(tsamp))
            print("t_mux="+str(tdmux))
            eit_cycles += 1
        i += 1


## cfa functions
def send_gcode(s_handle, data):
    '''
    Descr: ensures all gcode messages are correctly encoded and terminated
    '''
    print(f"sending: {data}")
    return s_handle.write((data+'\n').encode('ascii'))
    
def gcode_move_wait(s_handle,a=max_accel_G,x=None,y=None,z=None,f=600,gmove=1,with_offset=1,wait_off=0):
    '''
    Descr: Moves to an abs x,y,z [mm] coord at a set speed f [mm/min] using a gmove cmd(0=rapid,1=linear).
    By default the home+ref offsets are assumed
    '''
    global x_G
    global y_G
    global z_G
    if with_offset and not x==None:
        x = x + home_offset_mm[0]+ref_loc_mm[0]
    if with_offset and not y==None:
        y = y + home_offset_mm[1]+ref_loc_mm[1]
    if x == None:
        x = x_G
    if y == None:
        y = y_G
    if z == None:
        z = z_G
    len_move_mm = np.linalg.norm([x-x_G,y-y_G,z-z_G])
    v = f/60 # speed in mm/s
    t_move_s = 2*v/a + (len_move_mm - (v**2)/a)/v
    if t_move_s <= 2*v/a:
        t_move_s = np.sqrt(2*(len_move_mm/a))

    send_gcode(s_handle,f"G{gmove} X{x:.2f} Y{y:.2f} Z{z:.2f} F{f:.0f}")
    if not wait_off:
        ts = time.time()
        while (time.time()-ts) < t_move_s:
            # print(t_move_s, time.time()-ts)
            # get_pos(s_handle)
            _ = None

    x_G = x
    y_G = y
    z_G = z
    print(x,y,z)
    return t_move_s

def init_cfa(s_handle, max_accel, home_offset):
    # to 3d printer home
    send_gcode(s_handle,"G28 X Y Z") # CAN ALSO SEND 'G28 W'. DON'T SEND 'G28' ALONE AS THIS WILL COMPLETE MESH BED LEVELLING.
    print('homing now')
    send_gcode(s_handle,f"M201 X{max_accel} Y{max_accel} Z{max_accel}") # sets max accel in xyz
    send_gcode(s_handle,f"M204 P{max_accel} T{max_accel}") # sets max accel for print and travel moves
    gcode_move_wait(s_handle,z=5,with_offset=0)
    gcode_move_wait(s_handle,x=home_offset[0],y=home_offset[1],with_offset=0)
    print("please wait..")
    wait = 15
    for i in range(wait):
        print(wait-i)
        time.sleep(1)

def run_cfa_mesh_lvl(s_handle,load_handle,z_mesh_locs,hover_z_mm,dz=0.05,fz=100,f_thresh_N=1):
    """
    Descr: Runs a mesh bed leveling sequence on the DUT obtaining the offset of the DUT surface from the z hover point.
    The surface is determined to be touched when the force applied to the toolhead is > f_thresh_N.
    """
    global z_mesh
    # move on top of mid DUT
    gcode_move_wait(s_handle,z=hover_z_mm,with_offset=0)
    gcode_move_wait(s_handle,x=0,y=0)
    # go through mesh bed leveling pushes
    print(z_mesh_locs)
    for i in range(len(z_mesh_locs)):
        gcode_move_wait(s_handle,x=z_mesh_locs[i][0],y=z_mesh_locs[i][1])
        f_N = 0
        z_step = 0
        while f_N < f_thresh_N:
            # print(hover_z_mm,dz,z_step)
            gcode_move_wait(s_handle,z=hover_z_mm-dz*z_step, f=fz) # slow push
            z_step = z_step + 1
            f_N = load_handle.force_N
            print(f"{f_N:.2e}N")
        z_mesh.append(round(z_step*dz,2))
        gcode_move_wait(s_handle,z=hover_z_mm)
        print(f"x,y,z={get_pos(s_handle)}")
    print("z mesh array:")
    print(z_mesh)

def get_pos(s_handle):
    max_count = 10
    x_curr,y_curr,z_curr = 0,0,0
    s_handle.readlines()
    send_gcode(s_handle,"M114")
    time.sleep(0.001)
    rx_buf = s_handle.readline().decode()
    while((not 'X:' in rx_buf) and max_count):
        print(rx_buf)
        rx_buf = s_handle.readline().decode()
        max_count -= 1
        print(max_count)
        send_gcode(s_handle,"M114")
        
    if not max_count:
        input("Position not found. Printer spitting out rubbish. Continue?")
        return None
    pos_raw = m114_exp.findall(rx_buf)
    x_curr = pos_raw[3][1]
    y_curr = pos_raw[4][1]
    z_curr = pos_raw[5][1]
        
    return [x_curr,y_curr,z_curr]




## cfa gcode r/w fucntions
def writemove_gcode(f_handle,a=max_accel_G,x=None,y=None,z=None,f=600,gmove=1,with_offset=1):
    '''
    Descr: Moves to an abs x,y,z [mm] coord at a set speed f [mm/min] using a gmove cmd(0=rapid,1=linear)
    '''
    global x_G
    global y_G
    global z_G
    if with_offset and not x==None:
        x = x + home_offset_mm[0]+ref_loc_mm[0]
    if with_offset and not y==None:
        y = y + home_offset_mm[1]+ref_loc_mm[1]
    if x == None:
        x = x_G
    if y == None:
        y = y_G
    if z == None:
        z = z_G
    len_move_mm = np.linalg.norm([x-x_G,y-y_G,z-z_G])
    v = f/60 # speed in mm/s
    t_move_s = 2*v/a + (len_move_mm - (v**2)/a)/v
    if t_move_s <= 2*v/a:
        t_move_s = np.sqrt(2*(len_move_mm/a))
    f_handle.write(f"G{gmove} X{x:.2f} Y{y:.2f} Z{z:.2f} F{f:.0f};\n")
    f_handle.write(f"G04 P{t_move_s*1000:.2f};\n")
    x_G = x
    y_G = y
    z_G = z
    return t_move_s

def writepause_gcode(f_handle,t_ms):
    f_handle.write(f"G04 P{t_ms:.0f};\n")

def write_gcode_seq(gcode_file, push_points, strain, t_hold_s, thk, hover_z_mm, f_push=20):
    # OBSELETE #
    global x_G
    global y_G
    global z_G
    # calc strain distance Zs
    Zs = float(thk) * float(strain)
    if not (gcode_file[-5:] == ".gcode"):
        gcode_file = gcode_file + ".gcode"
    x_G_prev = x_G
    y_G_prev = y_G
    z_G_prev = z_G
    # open gcode file to write in
    with open(gcode_file, 'w') as f:
        print("Writing gcode file")
        f.write("G90;\n")
        for point in push_points:
            print(f"push point {point}")
            writemove_gcode(f,x=point[0],y=point[1])
            writemove_gcode(f,z=-(z_mesh[z_mesh_locs.index(point)]+Zs)+hover_z_mm,f=v_z_push)
            writepause_gcode(f,t_hold_s*1000)
            writemove_gcode(f,z=hover_z_mm,f=v_z_push)
            writepause_gcode(f,t_hold_s*1000)
        writemove_gcode(f,x=ref_loc_mm[0],y=ref_loc_mm[1],z=hover_z_mm,with_offset=0)
    x_G = x_G_prev
    y_G = y_G_prev
    z_G = z_G_prev
    f.close()
    return gcode_file

def send_gcode_seq(s_handle, gcode_file):
    with open(gcode_file,'r') as f:
        gcode = f.readlines()
    for line in gcode:
        send_gcode(s_handle, line[0:-2])
        print(line)
        time.sleep(0.001)
        # input("check gcode above?")
    print(s_handle.readlines())
    
def log_pos_wait(s_handle, t_wait):
    global pos_buf_mm
    global tpos_buf_s
    ts = time.time()
    while (time.time() - ts) < t_wait:
        pos_buf_mm.append(get_pos(s_handle))
        print(pos_buf_mm[-1])
        tpos_buf_s.append(time.time()-start_time_G)
        time.sleep(0.001)

def run_push_seq(s_handle,push_points,thk,strain,v_z_push):
    # main position logging function
    global stop_flag
    global x_G
    global y_G
    global z_G
    # define global buffers
    global pos_buf_mm
    global tpos_buf_s
    # calc strain distance Zs
    Zs = float(thk) * float(strain)
    # Start gcode sequence
    send_gcode(s_handle,"G90")
    for point in push_points:
        print(f"push point {point}")
        # move x,y to point
        t_wait = gcode_move_wait(s_handle,x=point[0],y=point[1],wait_off=1)
        log_pos_wait(s_handle, t_wait)
        # move z to push point
        t_wait = gcode_move_wait(s_handle,z=-(z_mesh[z_mesh_locs.index(point)]+Zs)+hover_z_mm,f=v_z_push,wait_off=1)
        log_pos_wait(s_handle, t_wait)
        # wait
        log_pos_wait(s_handle, t_hold_s)
        # move z to release push point
        t_wait = gcode_move_wait(s_handle,z=hover_z_mm,f=v_z_push,wait_off=1)
        log_pos_wait(s_handle, t_wait)
        # wait
        log_pos_wait(s_handle, t_hold_s)
    # move to reference location
    t_wait = gcode_move_wait(s_handle,x=home_offset_mm[0],y=home_offset_mm[1],z=hover_z_mm,with_offset=0)
    log_pos_wait(s_handle, t_wait)
    # wait at home
    log_pos_wait(s_handle, t_hold_s)

    stop_flag = 1



def main(t_buf, v_buf, i_buf, i_src_A, nplc, v_max_V, num_elecs=16):
    fs_max = 1000 # max Vmeas sample frequency (limited by how fast the PCB MUX can be MUX'd via serial SPI comms)

    # setup SMU connection
    rm = pyvisa.ResourceManager()
    available_devs = rm.list_resources()
    smu = K2600(available_devs[0])
    v_meas_max_V = 20
    init_smu(smu, i_src_A, v_meas_max_V, nplc, uart_baud)

    # setup mux PCB serial connection
    mux_s = serial.Serial(mux_com,uart_baud,timeout=0.1)
    mux_s.set_buffer_size(rx_size=256, tx_size=256)
    query_pcbmux(mux_s,'GET_STATE')
    query_pcbmux(mux_s,'GET_ITER')

    # setup force measurement
    loadcell = VoltageRatioInput()
    loadcell_init(loadcell)
    cal_loadcell(loadcell)

    # setup cfa (3d printer)
    cfa = serial.Serial('COM4', 115200, timeout=1)
    print("3d printer booting...")
    time.sleep(6)
    cfa.reset_input_buffer()
    time.sleep(0.1)
        # remove startup rubbish
    rx_buf = b'_'
    while((not 'ok' in rx_buf.decode()) and len(rx_buf)):
        print(rx_buf)
        rx_buf = cfa.readline()
    init_cfa(cfa, max_accel_G, home_offset_mm)
    run_cfa_mesh_lvl(cfa,loadcell,z_mesh_locs,hover_z_mm) # run mesh bed leveling
    z_mesh_datetime = str(datetime.utcnow()) # TODO Add to pkl file
        # send gcode to cfa
    gcode_file = write_gcode_seq(input_filename, push_points, strain, t_hold_s, th_dim_mm, hover_z_mm, v_z_push)
    gcode_move_wait(cfa,z=30)
    post_cal_t_relax = 240
    print(f"Material relaxing for {post_cal_t_relax}s ... :)")
    time.sleep(post_cal_t_relax)
    # send_gcode_seq(cfa, gcode_file)

    # COMPLETE EIT & FORCE & POS MEASUREMENTS CONCURRENTLY #
    start_time_G = time.time() # global reference start time
    eit_thread = Thread(target=log_eit_data, args=(smu, mux_s, fs_max, v_meas_max_V))
    force_thread = Thread(target=log_force_N, args=(loadcell,))
    pos_thread = Thread(target=run_push_seq, args=(cfa,push_points,th_dim_mm,strain,v_z_push))

    eit_thread.start()
    force_thread.start()
    pos_thread.start()
    eit_thread.join()
    force_thread.join()
    pos_thread.join()
        
    # close serial connections
    smu.disconnect()
    cfa.close()
    mux_s.close()



if __name__ == "__main__":
    import sys
    # data collection params
    i_src_A = 1e-3
    nplc = 0.01
    v_max_V = 20
    eit_cycles = 0
    num_elecs = 16

    # cfa params
    m114_exp = re.compile("\([^\(\)]*\)|[/\*].*\n|([XYZ]:\s|[XYZ]):?([-+]?[0-9]*\.?[0-9]*)") # M114 regex
    home_offset_mm = [57.8,14.5] 
    ref_loc_mm = [65,25] # home location (x,y)[mm] relative to the CoM of the DUT
    hover_z_mm = 22
    z_mesh_datetime = str(datetime.utcnow())
    
    # define thread stop flag
    stop_flag = False
    
    # set pcbmux serial params
    mux_com = 'COM8'
    uart_baud = 115200

    # set buffers for storing all recorded data
        # mesh leveling
    z_mesh = []
        # EIT
    v_buf_V = [] # EIT voltage readings
    i_buf_A = [] # EIT actual Isrc current
    tv_buf_s = [] # EIT PC timestamps
        # force
    f_buf_N = []
    f_buf_intp_N = []
    tf_buf_s = []
        # tool center point location xyz
    pos_buf_mm = []
    pos_buf_intp_mm = []
    tpos_buf_s = []

    try:
        if len(sys.argv)==1:
            raise Exception("\n\nPlease retry using the terminal format:\n\t>python eit_reader.py <filename> optional:<Isrc_A> <nplc>\n") 

        # set input sample details
        sample_name = input("what is your sample name? (e.g. CBSR_9p_1 or rGOSR_pcb_1) ")
        if sample_name[0:4] == 'CBSR':
            th_dim_mm = 4.0
            dia_dim_mm = 100.0
            fab_date = input("Input sample fabrication date if known? (in format DD-MM-YY): ")
        elif sample_name[0:5] == 'rGOSR':
            th_dim_mm = 3.0
            dia_dim_mm = 100.0
            fab_date = '27-05-22'
        else:
            th_dim_mm = float(input("Unknown sample disk dimensions. What thickness in mm? "))
            dia_dim_mm = float(input("What diameter in mm? "))
            fab_date = input("Input fabrication date if known? (in format DD-MM-YY):")

        # set experiment push parameters
        t_hold_s = float(input("Input sample push and hold time [s]:"))
        strain = float(input("Input sample strain [%]:"))/100

        # man_pts = input("Manually input push points?")
        # if (man_pts == ('y'or 'Y')):
        #     print(f"push points must be one of the following locs: \n {z_mesh_locs} \n as mesh bed interpolation hasn't been completed yet")
        #     push_points = []
        #     pt = '_'
        #     i = 0
        #     while len(pt):
        #         pt = input(f"Input x{i},y{i}:")
        #         push_points.append(list(pt.split(',')))
        #         i += 1

        push_points = [[0,0],[0.3*dia_dim_mm,0],[-0.3*dia_dim_mm,0], # default push points
            [0.15*dia_dim_mm,0.15*dia_dim_mm],[-0.15*dia_dim_mm,0.15*dia_dim_mm],
            [0.15*dia_dim_mm,-0.15*dia_dim_mm],[-0.15*dia_dim_mm,-0.15*dia_dim_mm],
            [0,0.3*dia_dim_mm],[0,-0.3*dia_dim_mm]]
        v_z_push = 40 # push speed mm/min
        z_mesh_locs = push_points 

        # Run program with cmd line arguments
        if len(sys.argv)>1: 
            input_filename = sys.argv[1]
        if len(sys.argv)>2:
            i_src_A = float(sys.argv[2])
        if len(sys.argv)>3:
            nplc = float(sys.argv[3])
        if len(sys.argv)>4:
            v_max_V = float(sys.argv[4])

        date_time_start = str(datetime.utcnow())

        main(tv_buf_s, v_buf_V, i_buf_A, i_src_A, nplc, v_max_V)

    except Exception:
        print(traceback.format_exc())

    finally:
        # close all connections
        # interpolate force data 
        if len(tf_buf_s) < len(tv_buf_s):
            print('Warning: force reading too slow and will be downsampled!')
        f_buf_intp_N = np.interp(tv_buf_s, tf_buf_s, f_buf_N)

        # interpolate pos data 
        if len(tpos_buf_s) < len(tv_buf_s):
            print('Warning: position reading too slow and will be downsampled!')
        pos_buf_mm = np.transpose(pos_buf_mm).astype('float64')
        xpos_buf_intp_mm = np.interp(tv_buf_s, tpos_buf_s, pos_buf_mm[0])
        ypos_buf_intp_mm = np.interp(tv_buf_s, tpos_buf_s, pos_buf_mm[1])
        zpos_buf_intp_mm = np.interp(tv_buf_s, tpos_buf_s, pos_buf_mm[2])
            
        print("CSV file saving...")
        with open(input_filename+'.csv', 'a', newline='') as csvfile:
            csv_data = csv.writer(csvfile, delimiter=',')
            csv_data.writerow(["UTC:",date_time_start])
            
            # cut off unsync'd data
            max_len = 0
            if len(v_buf_V) > len(tv_buf_s):
                max_len = len(tv_buf_s)
            else:
                max_len = len(v_buf_V)
            max_len -= max_len%num_elecs**2
            

            # write data to .csv file
            csv_data.writerow(["time_pc [s]", "voltage [V]", "i_src [A]","f [N]","x [mm]","y [mm]","z [mm]"])
            csv_data.writerows(np.transpose([tv_buf_s[0:max_len],v_buf_V[0:max_len],i_buf_A[0:max_len],f_buf_intp_N[0:max_len],
                                             xpos_buf_intp_mm[0:max_len],ypos_buf_intp_mm[0:max_len],zpos_buf_intp_mm[0:max_len]])) 
        print(f"csv file saved as: {input_filename}.csv")
        
        # # print out EIT voltage reading results report ## Doesn't work with when eit cycles > 16!!!
        # r_flag, vmax_flag = eit_reader_checker.report(input_filename+'.csv',i_src_A) 
        # _, r_adj_mean, _ = eit_reader_checker.get_inter_elec_res(v_buf_V, i_src_A)
        r_adj_mean = None # remove after fixed above functions
        
        # save all params to .pkl file of same name as .csv and .gcode
        eit_sample = PiezoResSample(sample_name, th_dim_mm, dia_dim_mm, fab_date)
        eit_test = EITFDataFrame(input_filename+'.csv', date_time_start, v_buf_V, i_buf_A, tv_buf_s, i_src_A, v_max_V, 
                                 nplc, eit_cycles, r_adj_mean, eit_sample, strain=strain, t_hold_s=t_hold_s, v_z_push=v_z_push,
                                 f_data_N=f_buf_intp_N, x_data_mm=xpos_buf_intp_mm, y_data_mm=ypos_buf_intp_mm, 
                                 z_data_mm=zpos_buf_intp_mm, z_mesh=z_mesh, z_mesh_locs=z_mesh_locs, 
                                 z_mesh_datetime=z_mesh_datetime)
        with open(input_filename+".pkl","wb") as fp:
            pkl.dump(eit_test,fp)
        print(f"pkl file saved as: {input_filename}.pkl")

        # print out results report
        r_flag, vmax_flag = eit_reader_checker.report(input_filename+'.csv',i_src_A) 
        _, r_adj_mean, _ = eit_reader_checker.get_inter_elec_res(v_buf_V, i_src_A)
        
        sys.exit()
        



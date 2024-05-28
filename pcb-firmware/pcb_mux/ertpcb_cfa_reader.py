"""
FILE: eitpcb_cfa_reader.py
AUTHOR: R Ellingham
DATE CREATED: May 2024
DATE MODIFIED: May 2024
PROGRAM DESC: Gather ERT PCB and force measurements. Writes the data to a CSV file ready for analysis.

USE CASE: 
1) Enter in program directory cmd prompt:
    > python ertpcb_cfa_reader.py <filename>
    
    * Additional command line arguments can be added to include current source and integration time using
        
        >> python ertpcb_cfa_reader.py <filename> <Isrc_A> <Vmax> <sample_name> <date_fabricated> <load_time_s> <trial_num>

    otherwise default values will be used (i.e. i_src_A=1e-3 ... see code)

2) Push Ctrl+C to stop EIT reading, then filename.csv will be saved
    # Note - for this the SMU outputs will remain on
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
    '''
    Description
    -----------
    Writes force and timestamp values to global buffers f_buf_N and tf_buf_s. 
    Execution is enabled by an external thread raising the stop_flag.

    Parameters
    ----------
    loadcell_handle : obj
        object for Phidget22 module loadcell.

    Returns
    -------
    None.

    '''
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
def Vadc2Vreal(Vadc,gain=0.232,Vref=5,n_adc=16):
    return Vadc * Vref / (2**n_adc * gain)

def log_eit_data(ser_handle,v_meas_max_V=20, num_elecs=16):
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
    global eit_cycles # EIT iteration count
    global stop_flag
    # define global buffers
    global v_buf_bit
    global tv_buf_s
    i = 0
    v_meas = ''

    print("LISTENING FOR ERT PCB...")
    while v_meas != b'A\r':
        if ser_handle.in_waiting > 0:
            v_meas = ser_handle.read_until(b'\r')
            print(v_meas)
        print("BEGINNING ERT DATA CAPTURE!")

    while not stop_flag:
        ## 1. take voltage reading
        t_si = time.time()
        if ser_handle.in_waiting > 0:
          v_meas = ser_handle.read_until(b'\r')
          if v_meas == b'A\r': # skip 'A\r' each frame
            v_meas = ser_handle.read_until(b'\r')
        if Vadc2Vreal(float(v_meas)) > 0.99*v_meas_max_V:
            v_meas = f"{int(v_meas)} MAX VOLTAGE ERROR"
            # print(v_meas_V)
            # input("Press enter to continue")
        i_src_act_A = np.nan # no Isrc measurement available on ERT PCB
        ts = time.time()
        tstamp = ts - start_time_G # sample timestamp

        tv_buf_s.append(tstamp)
        i_buf_A.append(i_src_act_A)
        v_buf_bit.append(v_meas)

        if not i % num_elecs**2:
            print(f"ERT cycle {i//num_elecs**2}")
            eit_cycles += 1
        i += 1


## cfa functions
def send_gcode(s_handle, data):
    '''
    Description
    -----------
    Ensures all gcode messages are encoded and terminated

    Parameters
    ----------
    s_handle : obj
        3d printer serial connection obj.
    data : str
        gcode string.

    Returns
    -------
    Serial error state

    '''
    print(f"sending: {data}")
    return s_handle.write((data+'\n').encode('ascii'))
    
def gcode_move_wait(s_handle,a=max_accel_G,x=None,y=None,z=None,f=600,gmove=1,with_offset=1,wait_off=0):
    '''
    Description
    -----------
    Moves to an abs x,y,z [mm] coord at a set speed f [mm/min] using a gmove 
    cmd(0=rapid,1=linear). Waits until movement is assumed finished.
    By default the home+ref offsets are assumed.

    Parameters
    ----------
    s_handle : obj
        3d printer serial connection obj.
    a : float, optional
        maximum acceleration. The default is max_accel_G.
    x : float, optional
        Desired x location. The default is None.
    y : float, optional
        Desired y location. The default is None.
    z : float, optional
        Desired z location. The default is None.
    f : int, optional
        Desired feed rate. The default is 600.
    gmove : bool, optional
        DESCRIPTION. The default is 1.
    with_offset : bool, optional
        DESCRIPTION. The default is 1.
    wait_off : bool, optional
        DESCRIPTION. The default is 0.

    Returns
    -------
    t_move_s : float
        predicted time taken to move.

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

def init_cfa(s_handle, home_offset, max_accel=max_accel_G):
    '''
    Description
    -----------
    Initialises cartesian force applicator (Prusa MK3s 3d printer) acceleration
    and moves to a known reference point

    Parameters
    ----------
    s_handle : obj
        3d printer serial connection obj.
    home_offset : float list
        x,y,z coordinates of home offset.
    max_accel : float, optional
        maximum acceleration. The default is max_accel_G. (global var)

    Returns
    -------
    None.

    '''
    
    # to 3d printer home
    send_gcode(s_handle,"G28 X Y Z") # CAN ALSO SEND 'G28 W'. DON'T SEND 'G28' ALONE AS THIS WILL COMPLETE MESH BED LEVELLING.
    print('homing now')
    send_gcode(s_handle,f"M201 X{max_accel} Y{max_accel} Z{max_accel}") # sets max accel in xyz
    send_gcode(s_handle,f"M204 P{max_accel} T{max_accel}") # sets max accel for print and travel moves
    gcode_move_wait(s_handle,z=15,with_offset=0)
    gcode_move_wait(s_handle,x=home_offset[0],y=home_offset[1],with_offset=0)
    print("please wait..")
    wait = 15
    for i in range(wait):
        print(wait-i)
        time.sleep(1)

def run_cfa_mesh_lvl(s_handle,load_handle,z_mesh_locs,hover_z_mm,dz=0.05,fz=100,f_thresh_N=0.2):
    '''
    Description
    -----------
    Run cartesian force aplicator mesh bed leveling which determines the 
    surface offset of the DUT at various locations
    
    Parameters
    ----------
    s_handle : obj
        3d printer serial connection obj.
    loadcell_handle : obj
        object for Phidget22 module loadcell.
    z_mesh_locs : float list
        offset locations.
    hover_z_mm : float
        height from home reference location.
    dz : float, optional
        z movement step size. The default is 0.05 mm.
    fz : int, optional
        z move feedrate. The default is 100 mm/min.
    f_thresh_N : float, optional
        force threshold for zero point touch. The default is 1 N.

    Returns
    -------
    None.

    '''
    
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
    '''
    Description
    -----------
    Gets current position from CFA

    Parameters
    ----------
    s_handle : obj
        3d printer serial connection obj.

    Returns
    -------
    list
        x,y,z locations.

    '''
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
    Description
    -----------
    Writes gcode cmds for moving to an abs x,y,z [mm] coord at a set speed f [mm/min] 
    using a gmove cmd(0=rapid,1=linear).
    By default the home+ref offsets are assumed.

    Parameters
    ----------
    f_handle : obj
        open file obj.
    a : float, optional
        maximum acceleration. The default is max_accel_G.
    x : float, optional
        Desired x location. The default is None.
    y : float, optional
        Desired y location. The default is None.
    z : float, optional
        Desired z location. The default is None.
    f : int, optional
        Desired feed rate. The default is 600.
    gmove : bool, optional
        DESCRIPTION. The default is 1.
    with_offset : bool, optional
        DESCRIPTION. The default is 1.

    Returns
    -------
    t_move_s : float
        predicted time taken to move.

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
    '''
    Description
    -----------
    write line of code in file for a pause

    Parameters
    ----------
    f_handle : obj
        open file obj.
    t_ms : float
        pause time in ms.

    Returns
    -------
    None.

    '''
    f_handle.write(f"G04 P{t_ms:.0f};\n")

def write_gcode_seq(gcode_file, push_points, strain, t_hold_s, thk, hover_z_mm):
    # OBSELETE FUNCTION #
    '''
    Description
    -----------
    Writes whole gcode file in a set sequence of push points

    Parameters
    ----------
    gcode_file : str
        gcode filename to save.
    push_points : float list
        list of [x,y] locations to push.
    strain : float (array?)
        compressive strain(s) to be used 
    t_hold_s : float
        DESCRIPTION.
    thk : float
        DUT thickness.
    hover_z_mm : float
        height from home reference location.

    Returns
    -------
    None.

    '''
    global x_G
    global y_G
    global z_G
    if not (gcode_file[-5:] == ".gcode"):
        gcode_file = gcode_file + ".gcode"
    x_G_prev = x_G
    y_G_prev = y_G
    z_G_prev = z_G
    # open gcode file to write in
    with open(gcode_file, 'w') as f:
        print("Writing gcode file")
        f.write("G90;\n")
        for ind, point in enumerate(push_points):
            Zs = float(thk) * float(strain[ind]) # calc strain distance Zs
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
    '''
    Description
    -----------
    Sends whole gcode file to a serially connected 3d printer

    Parameters
    ----------
    s_handle : obj
        3d printer serial connection obj.
    gcode_file : str
        file to run e.g. gcode_file.gcode.

    Returns
    -------
    None.

    '''
    
    with open(gcode_file,'r') as f:
        gcode = f.readlines()
    for line in gcode:
        send_gcode(s_handle, line[0:-2])
        print(line)
        time.sleep(0.001)
        # input("check gcode above?")
    print(s_handle.readlines())
    
def log_pos_wait(s_handle, t_wait):
    '''
    Description
    -----------
    Writes toolhead positions x,y,z and timestamp values to global buffers 
    pos_buf_mm and tpos_buf_s for a set amount of time
    Parameters
    ----------
    s_handle : obj
        3d printer serial connection obj.
    t_wait : float
        wait time in seconds.

    Returns
    -------
    None.

    '''
    global pos_buf_mm
    global tpos_buf_s
    ts = time.time()
    while (time.time() - ts) < t_wait:
        pos_buf_mm.append(get_pos(s_handle))
        print(pos_buf_mm[-1])
        tpos_buf_s.append(time.time()-start_time_G)
        time.sleep(0.001)

def run_push_seq(s_handle,push_points,thk,strain,v_z_push,t_hold_s):
    '''
    Main code driving sequence. Sends a sequence of gcode cmds to the 3d printer
    while logging toolhead positional data. Then returns to the home reference 
    location.

    Parameters
    ----------
    s_handle : obj
        3d printer serial connection o
    push_points : float list
        list of [x,y] locations to push.
    thk : float
        DUT thickness.
    strain : float (array?)
        compressive strain(s) to be used
    v_z_push : float
        push speed mm/min.

    Returns
    -------
    None.

    '''    
    # main position logging function
    global stop_flag
    global x_G
    global y_G
    global z_G
    # define global buffers
    global pos_buf_mm
    global tpos_buf_s

    # Start gcode sequence
    send_gcode(s_handle,"G90")
    for ind, point in enumerate(push_points):
        Zs = float(thk) * float(strain[ind]) # calc strain distance Zs
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



def main():
    '''
    Main loop - asking for input parameters running measurement threads.

    Parameters
    ----------
    None. But relies on many global variables. Watch out.

    Returns
    -------
    None.

    '''

    # setup ERT PCB serial connection
    ert_s = serial.Serial(ert_com,uart_baud,timeout=0.1)
    ert_s.set_buffer_size(rx_size=1024, tx_size=1024)

    # setup force measurement
    loadcell = VoltageRatioInput()
    loadcell_init(loadcell)
    cal_loadcell(loadcell)

    # setup cfa (3d printer)
    cfa = serial.Serial('COM7', 115200, timeout=1)
    print("3d printer booting...")
    time.sleep(6)
    cfa.reset_input_buffer()
    time.sleep(0.1)
        # remove 3d printer startup serial rubbish
    rx_buf = b'_'
    while((not 'ok' in rx_buf.decode()) and len(rx_buf)):
        print(rx_buf)
        rx_buf = cfa.readline()
    init_cfa(cfa, home_offset_mm)
    run_cfa_mesh_lvl(cfa,loadcell,z_mesh_locs,hover_z_mm) # run mesh bed leveling
    z_mesh_datetime = str(datetime.now()) # TODO Add to pkl file
    gcode_file = write_gcode_seq(input_filename, push_points, strain, t_hold_s, th_dim_mm, hover_z_mm) # make gcode
    gcode_move_wait(cfa,z=40)
    post_cal_t_relax = 240 # post calibration waiting time in secs
    print(f"Material relaxing for {post_cal_t_relax}s ... :)")
    time.sleep(post_cal_t_relax)
    # send_gcode_seq(cfa, gcode_file) # OBSELETE

    # COMPLETE EIT & FORCE & POS MEASUREMENTS CONCURRENTLY #
    start_time_G = time.time() # global reference start time
    eit_thread = Thread(target=log_eit_data, args=(ert_s, v_meas_max_V))
    force_thread = Thread(target=log_force_N, args=(loadcell,))
    pos_thread = Thread(target=run_push_seq, args=(cfa,push_points,th_dim_mm,strain,v_z_push,t_hold_s))

    eit_thread.start()
    force_thread.start()
    pos_thread.start()
    eit_thread.join()
    force_thread.join()
    pos_thread.join()
        
    # close serial connections
    cfa.close()
    ert_s.close()



if __name__ == "__main__":
    import sys
    # data collection params
    i_src_A = 0.9e-3
    nplc = 0.01
    v_meas_max_V = 22
    eit_cycles = 0
    num_elecs = 16
    fs_max = 1000 # max Vmeas sample frequency (limited by how fast the PCB MUX can be MUX'd via serial SPI comms)
    
    # init program arguments to zero
    sample_name = 0
    t_hold_s = 0
    strain = 0
    fab_date = 0

    # cfa params
    m114_exp = re.compile("\([^\(\)]*\)|[/\*].*\n|([XYZ]:\s|[XYZ]):?([-+]?[0-9]*\.?[0-9]*)") # M114 regex
    home_offset_mm = [57.8,14.5] # for original EIT test and regular DEA test
    # home_offset_mm = [62.8,16] # for altered DEA2 test (and accidentally for DEA1)
    # home_offset_mm = [59.3,13.0] # for altered DEA.5 test
    # ref_loc_mm = [65,40] # home location (x,y)[mm] relative to the CoM of the DUT for regular DEA test
    ref_loc_mm = [65,25] # home location (x,y)[mm] relative to the CoM of the DUT for original EIT test?
    hover_z_mm = 34
    v_z_push = 40 # push speed mm/min
    z_mesh_datetime = str(datetime.utcnow())
    strain_limit = 0.4 # max strain allowed (e.g. strain_limit = 0.4 = 40%)
    
    # define thread stop flag
    stop_flag = False
    
    # set pcbmux serial params
    ert_com = 'COM5'
    uart_baud = 115200

    # set buffers for storing all recorded data
        # mesh leveling
    z_mesh = []
        # EIT
    v_buf_bit = [] # EIT voltage readings
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
            raise Exception("\n\nPlease retry using the terminal format:\n\t>python eit_reader.py <filename> optional:<Isrc_A> <v_meas_max_V> <sample_name> <fab_date> <t_hold_s> <strain>(%)\n") 

        # Run program with cmd line arguments
        if len(sys.argv)>1: 
            input_filename = sys.argv[1]
        if len(sys.argv)>2:
            i_src_A = float(sys.argv[2])
        if len(sys.argv)>3:
            v_meas_max_V = float(sys.argv[3])
        if len(sys.argv)>4:
            sample_name = sys.argv[4]
        if len(sys.argv)>5:
            fab_date = sys.argv[5]
        if len(sys.argv)>6:
            t_hold_s = float(sys.argv[6])
        if len(sys.argv)>7:
            strain = float(sys.argv[7])/100

        # set input sample details
        if not sample_name:
            sample_name = input("what is your sample name? (e.g. DEA2_CBSR_8p_1, CBSR_9p_1, rGOSR_pcb_1, ...) ")
        if not fab_date:
            fab_date = input("Input fabrication date if known? (in format DD-MM-YY):")

        if sample_name[0:4] == 'CBSR':
            th_dim_mm = 4.0
            dia_dim_mm = 100.0
        elif sample_name[0:5] == 'rGOSR':
            th_dim_mm = 3.0
            dia_dim_mm = 60.0
            fab_date = '27-05-22'
        elif sample_name[0:4] == 'DEA2':
            th_dim_mm = 2.0
            dia_dim_mm = 100
            fab_date = '12-01-24'
        elif sample_name[0:4] == 'DEA1':
            th_dim_mm = 1.0
            dia_dim_mm = 100
            fab_date = '19-01-24'
        elif sample_name[0:5] == 'DEA.5':
            th_dim_mm = 0.5
            dia_dim_mm = 100
            fab_date = '12-01-24'
        else:
            th_dim_mm = float(input("Unknown sample disk dimensions. What thickness in mm? "))
            dia_dim_mm = float(input("What diameter in mm? "))

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
        # push_points = []
        push_points = [[0,0],[0.3*dia_dim_mm,0],[-0.3*dia_dim_mm,0], # default push points
            [0.15*dia_dim_mm,0.15*dia_dim_mm],[-0.15*dia_dim_mm,0.15*dia_dim_mm],
            [0.15*dia_dim_mm,-0.15*dia_dim_mm],[-0.15*dia_dim_mm,-0.15*dia_dim_mm],
            [0,0.3*dia_dim_mm],[0,-0.3*dia_dim_mm]]
        
        # push_points = [[-5.392221801367152, -0.6172784632712794], [-20.306653778948643, -21.8735172342719], # TODO: REMOVE hardcoded push_points
        #                 [-4.141772916377541, -0.576192808475054], [22.419426953903425, 25.622294022510296], 
        #                 [14.326545277267826, -32.812433833659064], [-4.163882510295342, -8.60347212910758], 
        #                 [-1.568040444519299, 2.283433546094633], [7.448211278155931, 8.306531257567638], 
        #                 [-4.127919477779628, 24.218559297630353], [1.743523889492196, 16.497891749167188]]
        
        z_mesh_locs = push_points 

        # set experiment push parameters
        if not t_hold_s:
            t_hold_s = float(input("Input sample push and hold time [s]:"))
        if not strain:
            strain = float(input("Input sample strain [%]:"))/100
        # elif strain < 0: # uncomment for randomly generated strains and push points
            # print("!!!Negative input strain detected... Inititating RaNd0M 5traiN S3QuENce (range=5-30%)!!!")
            # strain = []
            # for i in range(len(push_points)):
                # strain.append((np.random.random()*25+5)/100)
                # push_points[i][0] = np.random.random()*dia_dim_mm*0.26
                # push_points[i][1] = np.random.random()*dia_dim_mm*0.26
        if type(strain) != list: # create a list of strains if only one strain specified
            strain = np.ones(len(push_points))*strain
        elif len(strain) != len(push_points):
            print("Error strain array does not match push_points array length. Only first strain will be tested for all push points.")
            strain = np.ones(len(push_points))*strain[0]
        # strain = np.array([17.94306607, 23.95295652, 16.99213858, 10.88333719,  8.1215735, 13.80547079,  # TODO: REMOVE hardcoded strains
        #           7.00766015,  9.95756214, 28.47087393, 23.59077957])/100
        
        # if strain.any() < strain_limit:
        #     print(f"Strains requested: {strain*100}")
        #     raise Exception(f"Strain limit of {strain_limit*100} % exceeded. Please revise your settings.")
        
        date_time_start = str(datetime.now())
        print(fab_date,t_hold_s,strain)

        main()

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
        xpos_buf_intp_mm = np.interp(tv_buf_s, tpos_buf_s, pos_buf_mm[0]) - (home_offset_mm[0]+ref_loc_mm[0])
        ypos_buf_intp_mm = np.interp(tv_buf_s, tpos_buf_s, pos_buf_mm[1]) - (home_offset_mm[1]+ref_loc_mm[1])
        zpos_buf_intp_mm = np.interp(tv_buf_s, tpos_buf_s, pos_buf_mm[2]) - hover_z_mm
            
        print("CSV file saving...")
        with open(input_filename+'.csv', 'a', newline='') as csvfile:
            csv_data = csv.writer(csvfile, delimiter=',')
            csv_data.writerow(["UTC:",date_time_start])
            
            # cut off unsync'd data
            # max_len = 0
            # if len(v_buf_bit) > len(tv_buf_s):
            #     max_len = len(tv_buf_s)
            # else:
            #     max_len = len(v_buf_bit)
            max_len = len(v_buf_bit)
            max_len -= max_len % num_elecs**2

            # write data to .csv file
            csv_data.writerow(["time_pc [s]", "voltage [bit]", "i_src [A]","f [N]","x [mm]","y [mm]","z [mm]"])
            csv_data.writerows(np.transpose([tv_buf_s[0:max_len],v_buf_bit[0:max_len],i_buf_A[0:max_len],f_buf_intp_N[0:max_len],
                                             xpos_buf_intp_mm[0:max_len],ypos_buf_intp_mm[0:max_len],zpos_buf_intp_mm[0:max_len]])) 
        print(f"csv file saved as: {input_filename}.csv")
        
        r_adj_mean = None # remove after fixed above functions
        
        # save all params to .pkl file of same name as .csv and .gcode
        eit_sample = PiezoResSample(sample_name, th_dim_mm, dia_dim_mm, fab_date)
        eit_test = EITFDataFrame(input_filename+'.csv', date_time_start, v_buf_bit, i_buf_A, tv_buf_s, i_src_A, v_meas_max_V, 
                                 nplc, eit_cycles, r_adj_mean, eit_sample, strain=strain, t_hold_s=t_hold_s, v_z_push=v_z_push,
                                 f_data_N=f_buf_intp_N, x_data_mm=xpos_buf_intp_mm, y_data_mm=ypos_buf_intp_mm, 
                                 z_data_mm=zpos_buf_intp_mm, z_mesh=z_mesh, z_mesh_locs=z_mesh_locs, 
                                 z_mesh_datetime=z_mesh_datetime)
        with open(input_filename+".pkl","wb") as fp:
            pkl.dump(eit_test,fp)
        print(f"pkl file saved as: {input_filename}.pkl")

        # print out results report
        # r_flag, vmax_flag = eit_reader_checker.report(input_filename+'.csv',i_src_A) 
        # _, r_adj_mean, _ = eit_reader_checker.get_inter_elec_res(v_buf_bit, i_src_A)
        
        sys.exit()
        



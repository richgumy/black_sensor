"""
FILE: eit_reader_checker.py
AUTHOR: R Ellingham
DATE CREATED: July 2023
DATE MODIFIED: July 2023
PROGRAM DESC: Checks for higher than expected current and voltage values and gives the adjacent electrode resistance values. 
Uses raw data .csv files generated from eit_reader.py
Generates a report giving:
- number of cycles (i.e. recon frames)
- total time
- mean/median min and max sample time
- min and max voltage
- min and max current
- all adjacent electrode resistance mean values

> python eit_reader_checker.py <file_dir> <i_src_A>

"""
import pandas as pd
import numpy as np      

def get_Radj_data(ert_v_data_V, i_src_A, num_elecs=16):
    # Iterates through voltage data and determines all adjacent electrode resistances
    num_cycles = len(ert_v_data_V)//(num_elecs**2)
    R_elec_arr = np.zeros(((num_elecs*num_cycles),1))
    cycle = 0
    for i in range(len(ert_v_data_V)):
        if not (i % (num_elecs+1)):
            cycle = i // num_elecs**2
            R_elec_arr[i//(num_elecs)] = abs(ert_v_data_V[i+cycle])/i_src_A
    R_elec_arr_ = np.reshape(R_elec_arr,(num_cycles,num_elecs))
    R_elec_arr_[1][0] = R_elec_arr_[0][0]
    for cycle in range(2,num_cycles):
        R_elec_arr_[cycle][1:cycle] = R_elec_arr_[cycle][0:cycle-1]
        R_elec_arr_[cycle][0] = R_elec_arr_[0][0]
    R_elec_arr_[-1][-1] = R_elec_arr_[-2][-1]
    return R_elec_arr_

def main(file_dir, i_src_A, v_max_V=20, num_elecs=16):
    elec_data_raw = pd.read_csv(file_dir,skiprows=1)
    elec_data_raw['voltage [V]'][1:-1] = elec_data_raw['voltage [V]'][0:-2] # shift all readings by one
    elec_data_raw['voltage [V]'][0] = elec_data_raw['voltage [V]'][num_elecs**2]
    # split data
    r_adj_ohm = get_Radj_data(elec_data_raw['voltage [V]'], i_src_A)
    t_arr_s = elec_data_raw['time_pc [s]']
    td_arr_s = (np.array(t_arr_s[1:])-np.array(t_arr_s[0:-1]))
    v_arr_V = elec_data_raw['voltage [V]']
    i_arr_A = elec_data_raw['i_src [A]']
    i_arr_A = np.array(i_arr_A)
    title = f"*** EIT reader report for : {file_dir} ***"
    # print report
    print(f"{(len(title))*'*'}")
    print(f"{title}")
    print(f"{(len(title))*'*'}")
    print(f"num cycles ={len(v_arr_V)//256}")
    print(f"total time [s] ={t_arr_s.max():.8g}s")
    print(f"sample time [s] - mean={np.mean(td_arr_s):.2e}s, median={np.median(td_arr_s):.2e}s, min={np.min(td_arr_s):.2e}s, max={np.max(td_arr_s):.2e}s")
    print(f"sample freq. [Hz] mean={1/((num_elecs**2)*np.mean(td_arr_s)):.2e}Hz")
    print(f"V max={v_max_V:.2e}V")
    print(f"voltage [V] - min={np.min(v_arr_V):.2e}V, max={np.max(v_arr_V):.2e}V")
    print(f"i_src set [A] ={i_src_A:.2e}A")
    print(f"i_src actual [A] - min={np.min(i_arr_A[np.nonzero(i_arr_A)]):.6e}A, max={np.max(i_arr_A):.6e}A")
    print(f"r adjacent [Ohm] - min={np.min(r_adj_ohm):.2e} Ohm, max={np.max(r_adj_ohm):.2e} Ohm, mean=")
    for i in range(num_elecs):
        r_adj_mean = np.mean(r_adj_ohm,0)[i]
        r_adj_range = (np.max(r_adj_ohm,0)-np.min(r_adj_ohm,0))[i]/2
        if r_adj_range/r_adj_mean > 0.01:
            print(f"\tE{i}={r_adj_mean:.2e} \t +/- {r_adj_range:.2e} Ohm ! Warning unstable electrode !")
        else:
            print(f"\tE{i}={r_adj_mean:.2e} \t +/- {r_adj_range:.2e} Ohm")
    print(f"{(len(title))*'*'}")

if __name__ == "__main__":
    import sys
    if len(sys.argv)>1: 
        file_dir = sys.argv[1]
    if len(sys.argv)>2: 
        i_src_A = float(sys.argv[2])

    main(file_dir,i_src_A)

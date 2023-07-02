## Test script to run through a range of different I_src and NPLC values for the SMU
# python eit_reader.py <filename> <r=raw_data or leave blank for 16x16 format> <cycles> <i_src> <nplc>

import os
import numpy as np

test_name = "CBSR_dut_test_sweep_dualPS"

frmt = 'r'

cycles = 10

i_src_arr = [100e-6, 500e-6, 1e-3, 1.25e-3, 1.5e-3, 2e-3, 5e-3]
i_src_str_arr = ['100uA', '500uA', '1mA', '1.25mA', '1.5mA','2mA']

nplc_arr = np.array([0.01])
nplc_str_arr = nplc_arr.astype(str)

for i in range(len(i_src_arr)):
    for n in range(len(nplc_arr)):
        filename = f"{test_name}_{i_src_str_arr[i]}_nplc{nplc_str_arr[n]}" 
        cmd = f"python eit_reader.py {filename} {frmt} {cycles} {i_src_arr[i]} {nplc_str_arr[n]}"
        print(f"running >> {cmd}")
        os.system(cmd)
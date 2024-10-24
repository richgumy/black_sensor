"""
FILE: ambient_noise_analyser.py
AUTHOR: R Ellingham
PROGRAM DESC: From ERT data gathered complete an analysis of the noise seen from a calibration or ERT data
Analysis includes:
- Histogram for each ADC measurement
- Stats about ADC measurement std dev
USAGE: python make_ert_data_csv.py csv_filename comport
E.G. "python make_ert_data_csv.py data_capture COM6"
"""

import csv
import numpy as np
import matplotlib.pyplot as plt

# def main(input_filename):
#     Isrc_uA =  0
#     ert_scans = []
#     ert_table = np.zeros((16,16))
#     t_arr = np.zeros((16,))
#     with open(input_filename, 'r', newline='') as csvfile:
#         print("woo opend doc: " + input_filename)
#         data = csv.reader(csvfile, delimiter=',')
#         data = list(data)
#         i = 0
#         intrascan_indx = 0
#         interscan_indx = 0
#         for row in data:
#             # print(row)
#             if i < 3:
#                 if i == 1:
#                     Isrc_uA = row[1]
#                     print(Isrc_uA)
#             else:
#                 if row[1] == 'A':
#                     intrascan_indx = 0
#                     interscan_indx = interscan_indx + 1
#                     ert_scans.append(ert_table)
#                 if row[1] != 'A' and row[1] != '':
#                     # print(row[0])
#                     # print(row[1:17])
#                     t_arr[intrascan_indx] = float(row[0])
#                     ert_table[intrascan_indx][0:16] = np.array(row[1:17],dtype=float)
#                     intrascan_indx = intrascan_indx + 1
#             i = i + 1
#     print(ert_table)
#     ert_scans = np.array(ert_scans)
#     print(ert_scans)
#     scanz = ert_scans
        


# if __name__ == "__main__":
#     import sys
#     try:
#         # Input parameters
#         if len(sys.argv)>1: 
#             input_filename = (sys.argv[1])
#         else: 
#             input_filename = "1k3res_cal2.csv"
#         main(input_filename)
#     except Exception:
#         print(traceback.format_exc())
#     finally:
#         sys.exit()
        
        
## SPYDER VERSION BELOW ##
input_filename = "no_batt.csv"
Isrc_uA =  0
ert_scans = []
t_scans = []
ert_scan = np.zeros((16,16))
t_scan = np.zeros(16)
with open(input_filename, 'r', newline='') as csvfile:
    print("woo opend doc: " + input_filename)
    data = csv.reader(csvfile, delimiter=',')
    data = list(data)
    i = 0
    intrascan_indx = 0
    interscan_indx = 0
    for i, row in enumerate(data):
        # print(row)
        if i < 4:
            if i == 1:
                Isrc_uA = row[1]
                print(Isrc_uA+" uA Isrc")
        else:
            if row[1] == 'A':
                ert_scans.append(ert_scan.copy())
                t_scans.append(t_scan)
                intrascan_indx = 0
                interscan_indx = interscan_indx + 1
            if row[1] != 'A' and row[1] != '':
                t_scan[intrascan_indx] = float(row[0])
                ert_scan[intrascan_indx][0:16] = row[1:17]
                intrascan_indx = intrascan_indx + 1
                

ert_scans = np.array(ert_scans)
for j in range(0,16):
    histo = []
    for i in range(len(ert_scans)):
        histo.append(ert_scans[i][0][j])
    # print("Electrode %d" % j)
    # print("stdev:",round(np.std(histo)*1000)," uV")
    # print("median:",np.median(histo), " mV")
    CV = (float(np.std(histo))/float(np.median(histo)))
    # print("CV: %f" % CV)
    print(j,',',np.std(histo),',',np.median(histo),',',CV)
    plt.hist(histo,range=(np.median(histo)-round(np.std(histo)/8),np.median(histo)+round(np.std(histo)/8)))
    plt.figure()
plt.show()

       
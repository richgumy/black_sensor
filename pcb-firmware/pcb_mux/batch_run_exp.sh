#!/bin/sh

# Batch run experiments
filepath="..\..\..\PCB_MUX\data\with_force_data\CBSR_8p"
filename="\CBSR_9p_2_9push_strain_60s_1mA"
p=16
for i in 5 10 15 20 25 30
do
    for v in 1 2 3
    do
        echo filename="${filename:0:p}$i${filename:p}_$v"
        echo strain="$i"
        python eit_cfa_reader.py "${filename:0:p}$i${filename:p}_$v" "0.001" "22" "CBSR_9p_2" "06-08-23" "60" $i
    done
done
read

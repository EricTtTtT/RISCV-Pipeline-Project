#!/bin/bash

rm output.txt
for nb in 3 12 32
do
    for incre in 2 8 14
    do
        python3 L2Cache_generate.py -nb $nb -incre $incre
        ncverilog Final_tb_L2.v CHIP_L2_report.v slow_memory.v +define+L2Cache_report
    done
done
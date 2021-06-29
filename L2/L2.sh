#!/bin/bash

rm output.txt
for nb in 3 6 9 12 15 18 21 24 27 30
do
    for incre in 2 4 6 8 10 12 14
    do
        python3 L2Cache_generate.py -nb $nb -incre $incre
        ncverilog Final_tb_L2.v CHIP_L2_report.v slow_memory.v +define+L2Cache_report
    done
done
# size
# cp output.txt "output_I2w_D2w_L22w_64.txt"
# cp output.txt "output_I2w_D2w_L22w_128.txt"
# cp output.txt "output_I2w_D2w_L22w_256.txt"
# type
# cp output.txt "output_Idm_Ddm_L2dm_128.txt"
# cp output.txt "output_I2w_D2w_L2dm_128.txt"
# cp output.txt "output_Idm_Ddm_L22w_128.txt"
cp output.txt "output_I2w_Ddm_L22w_128.txt"
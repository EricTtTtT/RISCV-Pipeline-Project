#!/bin/bash

rm output.txt
for notBr in 10 20 30 40 50 60 70 80 90 100
do
    for interBr in 10 20 30 40 50 60 70 80 90 100
    do
        for Br in 10 20 30 40 50 60 70 80 90 100
        do
            python3 BrPred_generate.py -notBr $notBr -interBr $interBr -Br $Br
            # ncverilog Final_tb_Br.v CHIP_BrPred_2bit_predictor_report.v slow_memory.v +define+BrPred
            ncverilog Final_tb_Br.v CHIP_BrPred_2bit_counter_report.v slow_memory.v +define+BrPred
            # ncverilog Final_tb_Br.v CHIP_BrPred_bht_2bit_report.v slow_memory.v +define+BrPred
        done
    done
done

# cp output.txt output_predictor_v1.txt
cp output.txt output_predictor_v2.txt
# cp output.txt output_bht_1bit.txt
# cp output.txt output_bht_2bit.txt
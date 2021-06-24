#!/bin/bash

rm output.txt
for notBr in 10 30 50
do
    for interBr in 10 30 50
    do
        for Br in 10 30 50
        do
            python3 BrPred_generate.py -notBr $notBr -interBr $interBr -Br $Br
            ncverilog Final_tb_Br.v CHIP_BrPred_bht_2bit_report.v slow_memory.v +define+BrPred
            # ncverilog Final_tb_Br.v CHIP_BrPred_2bit_predictor_report.v slow_memory.v +define+BrPred
            # ncverilog Final_tb_Br.v CHIP_BrPred_2bit_counter_report.v slow_memory.v +define+BrPred
        done
    done
done

# cp output.txt bht_2bit.txt
# cp output.txt 2bit_predictor.txt
# cp output.txt 2bit_counter.txt
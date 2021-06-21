#!/bin/bash
# Note: nb >= 3 and nb <= 47 (if nb >= 48, there will be overflow)
# Note: nb >= 0 and nb*incre <= 1023 (size of slow memory is 1024 words)

for nb in 5 10 15 20 30 40
do
    for incre in 1 2 3 5 7 10 15 20 25
    do
        python3 L2Cache_generate.py -nb $nb -incre $incre
        ncverilog Final_tb.v CHIP_L2.v slow_memory.v +define+L2Cache
    done
done
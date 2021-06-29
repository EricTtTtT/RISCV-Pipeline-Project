# RISCV-Pipeline-Project

## Usuage 
Copy content in the files below into CHIP.v, and run ...  
# command for baseline
    $ncverilog Final_tb.v CHIP.v slow_memory.v +define+hasHazard +access+r  
# command for branch prediction
    $ncverilog Final_tb.v CHIP.v slow_memory.v +define+BrPred +access+r  
# command for compression
    $ncverilog Final_tb.v CHIP.v slow_memory.v +define+compression +access+r  
# command for L2 cache
    $ncverilog Final_tb.v CHIP.v slow_memory.v +define+L2Cache +access+r  

## BaseLine
  CHIP_hasHazard.v  

## Extension  
  use CHIP_complete.v to extend  

# BrPred  

BrPred:  
  * three types of instructinos in testbench  
    a. all not taken.  
    b. not taken, taken, not taken, taken...  
    c. all taken.  
  * four types of branch predictors
    a. 2-bit predictor v1  
    b. 2-bit predictor v2  
    c. branch hash table 1-bit  
    d. branch hash table 2-bit  
  * use bash script to test 1000 testbenches for each predictor  

# Compress:  
  * CHIP_compress.v  

# L2_cache:  
  * share L2 cache of I cache and D cache  
  * try 2-way and direct-mapped cache  
  * try different size of L2 cache
  * use bash script to test 70 testbenches for each setting

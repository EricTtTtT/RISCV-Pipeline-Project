Baseline:
  CHIP.v  

BrPred:
  5 cases to compare:

  1. CHIP_BrPred_2bit_predictor.v : predictor v1
  2. CHIP_BrPred_2bit_counter.v   : predictor v2
  3. CHIP_BrPred_bht.v.           : Branch prediction table
  4. CHIP_BrPred_bht_2bit.v       : 2-bit branch prediction table
  5. CHIP_complete.v.             : No prediction

Compress:
  CHIP_compress.v

L2_cache: X


Usuage
Copy content in the files above into CHIP.v, and run ...

$ncverilog Final_tb.v CHIP.v slow_memory.v +define+hasHazard +access+r
$ncverilog Final_tb.v CHIP.v slow_memory.v +define+BrPred +access+r
$ncverilog Final_tb.v CHIP.v slow_memory.v +define+compression +access+r


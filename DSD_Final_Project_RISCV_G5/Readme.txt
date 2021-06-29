Baseline:
  CHIP.v  

BrPred:
  !!!! The Best one: BHT_1-bit !!!!
  5 cases to compare:

  1. ./2-bit_predictor_v1/CHIP.v   : predictor v1
  2. ./2-bit_predictor_v2/CHIP.v   : predictor v2
  3. ./BHT_1-bit/CHIP.v            : branch history table
  4. ./BHT_2-bit_v2/CHIP.v         : 2-bit branch prediction table
  5. ./without_prediction/CHIP.v   : without prediction

Compress:
  CHIP.v

L2_cache: X


Usuage
Copy content in the files above into CHIP.v, and run ...

$ncverilog Final_tb.v CHIP.v slow_memory.v +define+hasHazard +access+r
$ncverilog Final_tb.v CHIP.v slow_memory.v +define+BrPred +access+r
$ncverilog Final_tb.v CHIP.v slow_memory.v +define+compression +access+r


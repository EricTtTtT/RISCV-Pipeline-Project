# RISCV-Pipeline-Project

## TODO
1. Critical path
2. 擋input
3. L2 cache
4. BrPred

## TO-ASK
1. Dcache刪除write back部分
2. Dcache一開始把資料先全給0
3. 用不到的指令，e.g. >>>

## Usuage 
Copy content in the files below into CHIP.v, and run ...
1. $ncverilog Final_tb.v CHIP.v slow_memory.v +define+hasHazard +access+r
2. $ncverilog Final_tb.v CHIP.v slow_memory.v +define+BrPred +access+r
3. $ncverilog Final_tb.v CHIP.v slow_memory.v +define+compression +access+r 

## BaseLine
  CHIP_hasHazard.v
  
## Extension:

* BrPred:
  1. 2-bits predictor: CHIP_Bred.v
  2. 2-bits predictor(小改）:  CHIP_Bred_comp2.v 
  3. without prediction: CHIP_Bred_comp1.v (from baseline)

* Compress: 
  CHIP_compress.v

* L2_cache:
  X

## file discription: 😎
* 

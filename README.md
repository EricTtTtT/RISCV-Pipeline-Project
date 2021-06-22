# RISCV-Pipeline-Project

## Extension 用 complete.v來修

## BrPred
共三段branch
a. all not taken
b. not taken, taken, not taken, taken....
c. all taken

1. BrPred.v: 即為complete.v，當作對照組，run time剛好跟predictor一樣。
2. 2bits_predictor.v: 初始直為not taken, 因此a.全對。b.因為在not taken和taken之間跳動，所以全錯。c.前面錯三個，後面全對。
3. 2bits_counter.v: a.全對，b.兩個跳會錯其中一個，c.前面錯三個，後面全對。

## Note
waste 710 cycle now
ctrl_bj_taken 應該要多加!ctrl_lw_stall才對，可以過的話就沒差

## TODO
1. 擋input？ L2有問題

## TO-ASK
1. Dcache刪除write back部分, OK
2. Dcache一開始把資料先全給0, OK

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


## Usuage 
Copy content in the files below into CHIP.v, and run ...
1. $ncverilog Final_tb.v CHIP.v slow_memory.v +define+hasHazard +access+r
2. $ncverilog Final_tb.v CHIP.v slow_memory.v +define+BrPred +access+r
3. $ncverilog Final_tb.v CHIP.v slow_memory.v +define+compression +access+r 

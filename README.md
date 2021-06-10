# RISCV-Pipeline-Project

## TODO
1. mem stage 用太多東西，critical path會變大，移到wb stage？(write reg會變comb)
2. Icache, Dcache分開寫
3. Memory pre-fetch
4. cache write buffer
5. ICACHE 把write部分刪掉
6. K-map 化簡 !test[1] & test[0]
8. cache 有乘法部分，用mux改掉。([proc_addr[1:0]*32+31 -: 32];

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

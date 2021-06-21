/*=========================================================
Description:
    !!! Format modified !!!
Notes:
    TODO:   need to check again
	FIT TB: modify design for fitting testbench
	====:   for larger command
	----:   for smaller command
=========================================================*/
// Top module of your design, you cannot modify this module!!
	module CHIP (	clk,
					rst_n,
		//----------for slow_memD------------
					mem_read_D,
					mem_write_D,
					mem_addr_D,
					mem_wdata_D,
					mem_rdata_D,
					mem_ready_D,
		//----------for slow_memI------------
					mem_read_I,
					mem_write_I,
					mem_addr_I,
					mem_wdata_I,
					mem_rdata_I,
					mem_ready_I,
		//----------for TestBed--------------				
					DCACHE_addr, 
					DCACHE_wdata,
					DCACHE_wen   
				);
	input			clk, rst_n;
	//--------------------------
	output			mem_read_D;
	output			mem_write_D;
	output	[31:4]	mem_addr_D;
	output	[127:0]	mem_wdata_D;
	input	[127:0]	mem_rdata_D;
	input			mem_ready_D;
	//--------------------------
	output			mem_read_I;
	output			mem_write_I;
	output	[31:4]	mem_addr_I;
	output	[127:0]	mem_wdata_I;
	input	[127:0]	mem_rdata_I;
	input			mem_ready_I;
	//----------for TestBed--------------
	output	[29:0]	DCACHE_addr;
	output	[31:0]	DCACHE_wdata;
	output			DCACHE_wen;
	//--------------------------

		// wire declaration  
		// Processor 和 Cache之間
		wire        ICACHE_ren;
		wire        ICACHE_wen;
		wire [29:0] ICACHE_addr;
		wire [31:0] ICACHE_wdata;
		wire        ICACHE_stall;
		wire [31:0] ICACHE_rdata;

		wire        DCACHE_ren;
		wire        DCACHE_wen;
		wire [29:0] DCACHE_addr;
		wire [31:0] DCACHE_wdata;
		wire        DCACHE_stall;
		wire [31:0] DCACHE_rdata;

		//=========================================
			// Note that the overall design of your RISCV includes:
			// 1. pipelined RISCV processor
			// 2. data cache
			// 3. instruction cache


			RISCV_Pipeline i_RISCV(
				// control interface
				.clk            (clk)           , 
				.rst_n          (rst_n)         ,
		//----------I cache interface-------		
				.ICACHE_ren     (ICACHE_ren)    ,
				.ICACHE_wen     (ICACHE_wen)    ,
				.ICACHE_addr    (ICACHE_addr)   ,
				.ICACHE_wdata   (ICACHE_wdata)  ,
				.ICACHE_stall   (ICACHE_stall)  ,
				.ICACHE_rdata   (ICACHE_rdata)  ,
		//----------D cache interface-------
				.DCACHE_ren     (DCACHE_ren)    ,
				.DCACHE_wen     (DCACHE_wen)    ,
				.DCACHE_addr    (DCACHE_addr)   ,
				.DCACHE_wdata   (DCACHE_wdata)  ,
				.DCACHE_stall   (DCACHE_stall)  ,
				.DCACHE_rdata   (DCACHE_rdata)	
			);

			Dcache D_cache(
				.clk        (clk)         ,
				.proc_reset (~rst_n)      ,
				.proc_read  (DCACHE_ren)  ,
				.proc_write (DCACHE_wen)  ,
				.proc_addr  (DCACHE_addr) ,
				.proc_rdata (DCACHE_rdata),
				.proc_wdata (DCACHE_wdata),
				.proc_stall (DCACHE_stall),
				.mem_read   (mem_read_D)  ,
				.mem_write  (mem_write_D) ,
				.mem_addr   (mem_addr_D)  ,
				.mem_wdata  (mem_wdata_D) ,
				.mem_rdata  (mem_rdata_D) ,
				.mem_ready  (mem_ready_D) 
			);

			Icache I_cache(
				.clk        (clk)         ,
				.proc_reset (~rst_n)      ,
				.proc_read  (ICACHE_ren)  ,
				.proc_write (ICACHE_wen)  ,
				.proc_addr  (ICACHE_addr) ,
				.proc_rdata (ICACHE_rdata),
				.proc_wdata (ICACHE_wdata),
				.proc_stall (ICACHE_stall),
				.mem_read   (mem_read_I)  ,
				.mem_write  (mem_write_I) ,
				.mem_addr   (mem_addr_I)  ,
				.mem_wdata  (mem_wdata_I) ,
				.mem_rdata  (mem_rdata_I) ,
				.mem_ready  (mem_ready_I)
			);
	endmodule
//============ Icache =================================
	module Icache(
		clk,
		proc_reset,
		proc_read,
		proc_write,
		proc_addr,
		proc_rdata,
		proc_wdata,
		proc_stall,
		mem_read,
		mem_write,
		mem_addr,
		mem_rdata,
		mem_wdata,
		mem_ready
		);
		
		//==== input/output definition ============================
	input          clk;
	// processor interface
	input          proc_reset;
	input          proc_read, proc_write;
	input   [29:0] proc_addr;
	input   [31:0] proc_wdata;
	output reg         proc_stall;
	output reg  [31:0] proc_rdata;
	// memory interface
	input  [127:0] mem_rdata;
	input          mem_ready;
	output reg         mem_read, mem_write;
	output reg [27:0] mem_addr;
	output reg [127:0] mem_wdata;
		
		//==== wire/reg definition ================================
			
		parameter STATE_compare_tag = 2'b00;
		parameter STATE_allocate = 2'b10;
		parameter STATE_idle = 2'b11;

		integer i;

		reg [155:0] cache0 [0:3]; // "1" bit lru, "1" bit valid, "26" bits tags, "128" bits data. =157 bits
		reg [155:0] cache1 [0:3]; // "1" bit lru, "1" bit valid, "26" bits tags, "128" bits data. =157 bits
		reg [155:0] nxt_cache0 [0:3]; // "1" bit lru, "1" bit valid, "26" bits tags, "128" bits data. =157 bits
		reg [155:0] nxt_cache1 [0:3]; // "1" bit lru, "1" bit valid, "26" bits tags, "128" bits data. =157 bits
		reg[31:0] data0, data1;

		reg [1:0] state;
		reg [1:0] nxt_state;

		wire miss;
		wire miss0;
		wire miss1;

		//assign sets = proc_addr[3:2];
		assign miss0 = !( cache0[proc_addr[3:2]][154] & (cache0[proc_addr[3:2]][153:128] == proc_addr[29:4]));
		assign miss1 = !( cache1[proc_addr[3:2]][154] & (cache1[proc_addr[3:2]][153:128] == proc_addr[29:4]));
		//modify
		assign miss = (proc_read) & miss0 & miss1;  // == hit1 | hit2
		
		//==== combinational circuit ==============================
		reg [27:0] mem_addr_temp_nxt,mem_addr_temp;

		always @(*) begin
			case(state)
				STATE_compare_tag:begin
					if (miss) begin //clean
						nxt_state = STATE_allocate;
					end
					else begin
						nxt_state = STATE_compare_tag;
					end
				end

				STATE_allocate:begin
					if (mem_ready) begin
						nxt_state = STATE_compare_tag;
					end
					else begin
						nxt_state = STATE_allocate;
					end
				end

				STATE_idle:begin
					nxt_state = STATE_compare_tag;
				end
				default:begin
					nxt_state= state;
				end
			endcase
		end

		always @(*) begin
			proc_rdata = 0 ;
			for (i = 0; i<4; i=i+1) begin
				nxt_cache0[i] = cache0[i];
				nxt_cache1[i] = cache1[i];
			end
			mem_wdata = 0;
			mem_addr  = 0;
			mem_read = 0;
			mem_write = 0;
			mem_addr_temp_nxt = 0 ;
			proc_stall = 0;
			
			case(proc_addr[1:0])
				2'b00:begin data0=cache0[proc_addr[3:2]][31:0]; data1=cache1[proc_addr[3:2]][31:0];end
				2'b01:begin data0=cache0[proc_addr[3:2]][63:32]; data1=cache1[proc_addr[3:2]][63:32];end
				2'b10:begin data0=cache0[proc_addr[3:2]][95:64]; data1=cache1[proc_addr[3:2]][95:64];end
				2'b11:begin data0=cache0[proc_addr[3:2]][127:96]; data1=cache1[proc_addr[3:2]][127:96];end
			endcase

			case(state)
				STATE_compare_tag:begin
					proc_stall = miss;
		
					//memory prefetch
					mem_read = !mem_ready;
					mem_addr  =  proc_addr[29:2];
					mem_addr_temp_nxt = (proc_addr[29:2]==mem_addr_temp)? mem_addr_temp : mem_addr;

					if (!miss) begin
						if (!miss0) begin
							proc_rdata = data0; //hit
						end
						else if (!miss1) begin
							proc_rdata = data1; //hit
						end
					end        
				end

				STATE_allocate:begin
					proc_stall = 1;
					mem_addr  = proc_addr[29:2];

					if (mem_ready) begin
						mem_read = 0;
						if (!cache0[proc_addr[3:2]][155]) begin
							nxt_cache0[proc_addr[3:2]][127:0] = mem_rdata; //data
							nxt_cache0[proc_addr[3:2]][153:128] = proc_addr[29:4]; //tags
							nxt_cache0[proc_addr[3:2]][154] = 1; //valid
							nxt_cache0[proc_addr[3:2]][155] = 1;  //lru0;
							nxt_cache1[proc_addr[3:2]][155] = 0;  //lru1;
						end
						else begin
							nxt_cache1[proc_addr[3:2]][127:0] = mem_rdata; //data
							nxt_cache1[proc_addr[3:2]][153:128] = proc_addr[29:4]; //tags
							nxt_cache1[proc_addr[3:2]][154] = 1; //valid
							nxt_cache0[proc_addr[3:2]][155] = 0;  //lru0;
							nxt_cache1[proc_addr[3:2]][155] = 1;  //lru1;
						end
					end
					else begin
						mem_read = 1;
					end
				end

				STATE_idle:begin
					proc_stall = 0;
				end

			endcase
		end

		//==== sequential circuit =================================
		always@( posedge clk ) begin
			if( proc_reset ) begin
				for (i = 0 ; i < 4 ; i=i+1) begin
					cache0[i] <=  0;
					cache1[i] <=  0;
				end
				state <= STATE_idle;
				mem_addr_temp <= 0;
			end
			else begin
				for (i = 0; i<4; i=i+1) begin
					cache0[i] <= nxt_cache0[i];
					cache1[i] <= nxt_cache1[i];
				end
				state <= nxt_state;
				mem_addr_temp <= mem_addr_temp_nxt;
			end
		end

		endmodule

//============ Icache =================================

//============ Dcache =================================
	module Dcache(
		clk, proc_reset,
		proc_read, proc_write, proc_addr, proc_rdata, proc_wdata, proc_stall,
		mem_read, mem_write, mem_addr, mem_rdata, mem_wdata, mem_ready );
		//---- input/output definition ----------------------
	input          clk;
	// processor interface
	input         		proc_reset;
	input         		proc_read, proc_write;
	input	[29:0]		proc_addr;
	input   [31:0]		proc_wdata;
	output reg     		proc_stall;
	output reg [31:0]	proc_rdata;
	// memory interface
	input  [127:0] 		mem_rdata;
	input          		mem_ready;
	output reg        	mem_read, mem_write;
	output reg [27:0]	mem_addr;
	output reg [127:0]	mem_wdata;
		
		parameter NUM_BLOCKS = 4;
		parameter BLOCK_ADDR_SIZE = 2;  // log2 NUM_BLOCKS

		// block = [cache1, cache2]
		// cache = [word0, word1, word2, word3] 
		parameter TAG_SIZE = 28-BLOCK_ADDR_SIZE;  // 30 - 2 - BLOCK_ADDR_SIZE
		parameter BLOCK_hSIZE = 130+TAG_SIZE;  // 1+1+TAG_SIZE+128
		parameter IDLE = 2'd0;
		parameter COMP = 2'd1;
		parameter WRITE = 2'd2;
		parameter ALLOC = 2'd3;
		integer i;

		//---- wire/reg definition ----------------------------
		wire valid1, dirty1, hit1;
		wire valid2, dirty2, hit2;
		wire hit;
		wire [BLOCK_ADDR_SIZE-1:0] block_addr;
		wire [TAG_SIZE-1:0] tag;
		wire [BLOCK_hSIZE-1:0] cache1_select, cache2_select;

		// flip flops
		reg [1:0] state, state_next;
		reg [BLOCK_hSIZE-1:0] cache1 [0:NUM_BLOCKS-1];
		reg [BLOCK_hSIZE-1:0] cache1_next [0:NUM_BLOCKS-1];
		reg [BLOCK_hSIZE-1:0] cache2 [0:NUM_BLOCKS-1];
		reg [BLOCK_hSIZE-1:0] cache2_next [0:NUM_BLOCKS-1];
		reg [NUM_BLOCKS-1:0] lru, lru_next;  // low --> last used is cache 2, use cache1 first
		reg mem_ready_ff;
		reg [127:0] mem_rdata_ff;

		//==== Finite State Machine ===========================
		always @(*) begin
			case(state)
				IDLE: begin
					state_next = COMP;
				end
				COMP: begin
					state_next = (proc_stall==1'b0)? COMP : (dirty1 & dirty2)? WRITE : ALLOC;
				end
				WRITE: begin
					state_next = mem_ready_ff? ALLOC : WRITE;
				end
				ALLOC: begin
					state_next = mem_ready_ff? COMP : ALLOC;                
				end
				default: state_next = state;
			endcase
		end

		//==== Combinational Circuit ==========================
		assign block_addr = proc_addr[1+BLOCK_ADDR_SIZE : 2];
		assign tag = proc_addr[29 : 30-TAG_SIZE];
		assign cache1_select = cache1[block_addr];
		assign cache2_select = cache2[block_addr];
		
		assign valid1 = cache1_select[BLOCK_hSIZE-1];
		assign dirty1 = cache1_select[BLOCK_hSIZE-2];
		assign valid2 = cache2_select[BLOCK_hSIZE-1];
		assign dirty2 = cache2_select[BLOCK_hSIZE-2];
		assign hit1 = valid1 & (cache1_select[127+TAG_SIZE : 128] == tag);
		assign hit2 = valid2 & (cache2_select[127+TAG_SIZE : 128] == tag);
		assign hit = hit1 | hit2;
		
		//---- I/O signals ------------------------------------
		always @(*) begin
			proc_stall = ((state==COMP & hit) | !(proc_read | proc_write)) ? 0 : 1;
			case ({hit1, hit2, proc_addr[1:0]})
				4'b1000: proc_rdata = cache1_select[31:0];
				4'b1001: proc_rdata = cache1_select[63:32];
				4'b1010: proc_rdata = cache1_select[95:64];
				4'b1011: proc_rdata = cache1_select[127:96];
				4'b0100: proc_rdata = cache2_select[31:0];
				4'b0101: proc_rdata = cache2_select[63:32];
				4'b0110: proc_rdata = cache2_select[95:64];
				4'b0111: proc_rdata = cache2_select[127:96];
				default: proc_rdata = 32'd0;
			endcase
			mem_read = ~mem_ready_ff && state==ALLOC;
			mem_write = ~mem_ready_ff && state==WRITE;
			case ({state, lru[block_addr]})  // WRITE is 2'b10
				3'b100: mem_addr = {cache1_select[127+TAG_SIZE : 128], block_addr};
				3'b101: mem_addr = {cache2_select[127+TAG_SIZE : 128], block_addr};
				default: mem_addr = proc_addr[29:2];
			endcase
			mem_wdata = lru[block_addr]? cache2_select[127:0] : cache1_select[127:0];
		end

		//---- handle cache_next and lru bits -----------------
		always @(*) begin
			for (i=0; i<NUM_BLOCKS; i=i+1) begin
				cache1_next[i] = cache1[i];
				cache2_next[i] = cache2[i];
				lru_next[i] = lru[i];
			end
			lru_next[block_addr] = state==COMP?
										hit1? 1 : hit2? 0 : lru[block_addr]
									: lru[block_addr];

			case(state)
				COMP: begin
					if (proc_write) begin
						case ({hit1, hit2, proc_addr[1:0]})
							4'b1000: cache1_next[block_addr][31:0] = proc_wdata;
							4'b1001: cache1_next[block_addr][63:32] = proc_wdata;
							4'b1010: cache1_next[block_addr][95:64] = proc_wdata;
							4'b1011: cache1_next[block_addr][127:96] = proc_wdata;
							4'b0100: cache2_next[block_addr][31:0] = proc_wdata;
							4'b0101: cache2_next[block_addr][63:32] = proc_wdata;
							4'b0110: cache2_next[block_addr][95:64] = proc_wdata;
							4'b0111: cache2_next[block_addr][127:96] = proc_wdata;
						endcase
						if (hit1) begin
							cache1_next[block_addr][127+TAG_SIZE : 128] = tag;
							cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b11;
						end
						if (hit2) begin
							cache2_next[block_addr][127+TAG_SIZE : 128] = tag;
							cache2_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b11;
						end
					end else begin
						cache1_next[block_addr] = cache1_select;
						cache2_next[block_addr] = cache2_select;
					end
				end
				WRITE: begin
					if (mem_ready_ff) begin
						if (lru[block_addr]) begin
							cache2_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
						end else begin
							cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
						end
					end
				end
				ALLOC: begin
					if (!dirty1 & !dirty2) begin
						if (lru[block_addr]) begin
							cache2_next[block_addr][127:0] = mem_rdata_ff;
							cache2_next[block_addr][127+TAG_SIZE : 128] = tag;
							cache2_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
						end else begin
							cache1_next[block_addr][127:0] = mem_rdata_ff;
							cache1_next[block_addr][127+TAG_SIZE : 128] = tag;
							cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
						end
					end else if (!dirty1) begin
						cache1_next[block_addr][127:0] = mem_rdata_ff;
						cache1_next[block_addr][127+TAG_SIZE : 128] = tag;
						cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
					end else begin
						cache2_next[block_addr][127:0] = mem_rdata_ff;
						cache2_next[block_addr][127+TAG_SIZE : 128] = tag;
						cache2_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;   
					end
				end
			endcase
		end

		//==== Sequential Circuit =============================
		always@( posedge clk ) begin
			if( proc_reset ) begin
				for (i=0; i<NUM_BLOCKS; i=i+1) begin
					cache1[i] <= 0;
					cache2[i] <= 0;
					lru[i] <= 0;
				end
				state <= IDLE;
				mem_ready_ff <= 1'b0;
				mem_rdata_ff <= 127'd0;
			end else begin
				for (i=0; i<NUM_BLOCKS; i=i+1) begin
					cache1[i] <= cache1_next[i];
					cache2[i] <= cache2_next[i];
					lru[i] <= lru_next[i];
				end
				state <= state_next;
				mem_ready_ff <= mem_ready;
				mem_rdata_ff <= mem_rdata;
			end
		end

	endmodule
//============ Dcache =================================

//============ CHIP ===================================
module RISCV_Pipeline(
	clk, rst_n, ICACHE_ren, ICACHE_wen, ICACHE_addr, ICACHE_wdata, ICACHE_stall, ICACHE_rdata,
				DCACHE_ren, DCACHE_wen, DCACHE_addr, DCACHE_wdata, DCACHE_stall, DCACHE_rdata
);
input clk, rst_n;
input ICACHE_stall, DCACHE_stall;
input [31:0] ICACHE_rdata, DCACHE_rdata;
output reg ICACHE_ren, ICACHE_wen, DCACHE_ren, DCACHE_wen;
output reg [29:0] ICACHE_addr, DCACHE_addr;
output reg [31:0] ICACHE_wdata, DCACHE_wdata;
	//======== wire & reg =============
		integer i;
		parameter R_type = 3'd0;
		parameter I_type = 3'd2;
		parameter S_type = 3'd1;
		parameter B_type = 3'd3;
		parameter J_type = 3'd6;
		reg	[2:0] type;

	//-------- PC ---------------------
		reg [31:0] PC, PC_ID, PC_EX;	// ff
		reg PC_start;					// ff
		reg [31:0] PC_nxt;
		reg [31:0] PC_B_ID, PC_jalr_ID;
		reg [31:0] PC_FA_j;

	//-------- Registers --------------
		reg [31:0] register [0:31];		// ff
		wire [4:0] rs1_ID, rs2_ID;
		wire [31:0] rs1_data_ID, rs2_data_ID;
		reg [4:0] rs1_EX, rs2_EX;		// ff
		reg [31:0] rs1_data_EX, rs2_data_EX;	// ff

		wire [4:0] rd_ID;
		reg [4:0] rd_EX, rd_MEM, rd_WB;	// ff
		reg signed [31:0] rd_data_EX;
		reg signed [31:0] rd_data_MEM;		// ff
		reg signed [31:0] rd_data_MEM_real;
		reg signed [31:0] rd_data_WB;		// ff
		// reg signed [31:0] rd_data_WB_real;

		reg [31:0] compare_rs1;	// TODO: need this??
		reg [31:0] compare_rs2;

	//-------- Instruction ------------
		wire [31:0] inst_IF;
		reg [31:0] inst_ID;		// ff
		wire [6:0] op;
		wire [2:0] func3;
		wire [6:0] func7;

	//-------- Immediate --------------
		reg [31:0] imme_ID;
		reg [31:0] imme_EX;		// ff

	//-------- ALU --------------------
		reg signed [31:0] alu_in1;
		reg signed [31:0] alu_in2;
		reg signed [31:0] alu_in2_temp;
		reg signed [31:0] alu_out_EX;
		reg signed [31:0] alu_out_MEM;	// ff
		reg signed [31:0] alu_out_WB;	// ff
		reg [3:0] alu_ctrl_ID;
		reg [3:0] alu_ctrl_EX;	// ff

	//-------- Memory -----------------
		reg [31:0] mem_rdata_MEM;	// read from mem
		// reg [31:0] mem_rdata_WB;	// ff
		reg [31:0] mem_wdata_EX;
		reg [31:0] mem_wdata_MEM;	// write to mem, ff

	//-------- Control unit -----------
		// *_EX, *_MEM, *_WB are all ff
		reg ctrl_jalr_ID,		ctrl_jalr_EX;
		reg ctrl_jal_ID,		ctrl_jal_EX;
		reg ctrl_beq_ID;
		reg ctrl_bne_ID;
		reg ctrl_memread_ID,	ctrl_memread_EX, 	ctrl_memread_MEM;
		reg ctrl_memtoreg_ID,	ctrl_memtoreg_EX, 	ctrl_memtoreg_MEM, 	ctrl_memtoreg_WB;
		reg ctrl_memwrite_ID, 	ctrl_memwrite_EX, 	ctrl_memwrite_MEM; 
		reg ctrl_regwrite_ID, 	ctrl_regwrite_EX, 	ctrl_regwrite_MEM, 	ctrl_regwrite_WB;
		reg ctrl_ALUSrc_ID, 	ctrl_ALUSrc_EX;

		reg [1:0] ctrl_FA, ctrl_FB;
		reg [1:0] ctrl_FA_j, ctrl_FB_j;
		reg ctrl_lw_stall;
		reg ctrl_bj_taken;
	//-------- BranchPrediction -------
		wire brancheqaul;
//======== Combinational Circuit ======================

	//======== instruction ============
		assign inst_IF = (ctrl_bj_taken | ctrl_jalr_ID)?
							32'h00000013
						:	{ICACHE_rdata[7:0], ICACHE_rdata[15:8], ICACHE_rdata[23:16], ICACHE_rdata[31:24]};// nop or fetch instruction
		assign op = inst_ID[6:0];
		assign rd_ID = inst_ID[11:7];
		assign rs1_ID = inst_ID[19:15];
		assign rs2_ID = inst_ID[24:20];
		assign func3 = inst_ID[14:12]; 
		assign func7 = inst_ID[31:25];

		//-------- type ---------------
		always @(*) begin	// FIT TB
			type[2] = op[3];
			type[1] = ( (!op[6])&(!op[5])) | (op[6]&op[5]) ;
			type[0] = op[5] & (!op[4] & !op[2] );
		end
		// always @(*) begin
		// 	case (op[6:2])
		// 		5'b01100: type = 0; //R-type
		// 		5'b00100: type = 1; //I-type
		// 		5'b00000: type = 1; //I-type, lw
		// 		5'b11001: type = 1; //I-type, jalr
		// 		5'b01000: type = 2; //S, sw
		// 		5'b11000: type = 3; //B, beq
		// 		5'b11011: type = 4; //jal
		// 		default: type=5;
		// 	endcase
		// end


	//======== control unit ===========
		always @(*) begin
			ctrl_jal_ID = 0;
			ctrl_jalr_ID = 0;
			ctrl_beq_ID = 0;
			ctrl_bne_ID = 0;
			ctrl_memread_ID = 0;
			ctrl_memwrite_ID = 0;
			ctrl_memtoreg_ID = 0;
			ctrl_regwrite_ID = 0;
			ctrl_ALUSrc_ID = 0;
			case (type)
				R_type: begin
					ctrl_regwrite_ID = 1;
					ctrl_ALUSrc_ID = 0;
				end
				I_type: begin
					ctrl_memread_ID = !op[4] & !op[2];	//lw, read mem
					ctrl_memtoreg_ID = !op[4] & !op[2];	//lw, read mem
					ctrl_regwrite_ID = 1;
					ctrl_ALUSrc_ID = 1;	
					ctrl_jalr_ID = op[2]; //jalr
				end
				S_type: begin
					ctrl_memwrite_ID = 1;
					ctrl_ALUSrc_ID = 1;			
				end
				B_type: begin
					ctrl_beq_ID = !func3[0];
					ctrl_bne_ID = func3[0];
					ctrl_ALUSrc_ID = 0;			
				end
				J_type: begin
					ctrl_jal_ID = 1;
					ctrl_regwrite_ID = 1;
					ctrl_ALUSrc_ID = 1;
				end
				default: begin
					ctrl_jal_ID = 0;
					ctrl_jalr_ID = 0;
					ctrl_beq_ID = 0;
					ctrl_bne_ID = 0;
					ctrl_memread_ID = 0;
					ctrl_memwrite_ID = 0;
					ctrl_memtoreg_ID = 0;
					ctrl_regwrite_ID = 0;
					ctrl_ALUSrc_ID = 0; 
				end
			endcase
		end

	//======== register files =========
		assign rs1_data_ID = register[rs1_ID];
		assign rs2_data_ID = register[rs2_ID];
        always @(*) begin
            // rd_data written
		    rd_data_EX = (ctrl_jal_EX | ctrl_jalr_EX)? PC_EX+4 : alu_out_EX;
        end

	//======== imm generator ==========
		always @(*) begin
			case (type)
				I_type: begin
					imme_ID = { {21{inst_ID[31]}}, inst_ID[30:20]};	// FIT TB
					//imme_ID = (func3==3'b001 | func3==3'b101)? 
					//			{ {27{1'b0}} , inst_ID[24:20]}: //shamt, slli, srai, srli
					//			{ {21{inst_ID[31]}}, inst_ID[30:20]}; //addi, andi, ori, xori, slli, srai, srli, slti, lw
				end
				S_type: begin		
					imme_ID = { {21{inst_ID[31]}}, inst_ID[30:25], inst_ID[11:7]}; //sw
				end
				B_type: begin		
					imme_ID = { {20{inst_ID[31]}}, inst_ID[7], inst_ID[30:25], inst_ID[11:8], 1'b0 }; //beq, bne
				end
				J_type: begin
					imme_ID = { {12{inst_ID[31]}}, inst_ID[19:12], inst_ID[20], inst_ID[30:25], inst_ID[24:21], 1'b0 }; //jal
				end
				default: begin
					imme_ID = 0;
				end
			endcase
		end

	//======== ALU ctrl unit ==========
		always @(*) begin	// FIT TB
			alu_ctrl_ID[3] = !func3[1] & func3[0];
			alu_ctrl_ID[2] = (func3[2] & func3[0]) | (!func3[2] & func3[1] & op[4]);
			alu_ctrl_ID[1] = func3[2];
			alu_ctrl_ID[0] = (func7[5] & func3[2] & !func3[1] & func3[0]) | (func3[2]&func3[1]) | (func7[5] & !func3[2]  & op[5] );
			alu_ctrl_ID[0] = ( (func7[5] & !func3[1]) & ((func3[2]&func3[0]) | op[5]) )  | (func3[2]&func3[1]);
		end
		// always @(*) begin
		// 	if (op[6:2]==5'b00100) begin//I-type
		// 		if (func3==3'b101) alu_ctrl_ID = func7[5]? 4'd8:4'd7; //srai, srli
		// 		else if (func3==3'b001) alu_ctrl_ID = 4'd6; //slli
		// 		else if (func3==3'b010) alu_ctrl_ID = 4'd5; //slti
		// 		else if (func3==3'b100) alu_ctrl_ID = 4'd4; ///xori
		// 		else if (func3==3'b110) alu_ctrl_ID = 4'd3; //ori
		// 		else if (func3==3'b111) alu_ctrl_ID = 4'd2; //andi
		// 		else alu_ctrl_ID = 4'd0; //addi
		// 	end
		// 	else if (op[6:2]==5'b01100) begin //R-type
		// 		if (func3==3'b000) alu_ctrl_ID = func7[5]? 4'd1:4'd0; //sub, add
		// 		else if (func3==3'b111) alu_ctrl_ID = 4'd2; //and
		// 		else if (func3==3'b110) alu_ctrl_ID = 4'd3; //or
		// 		else if (func3==3'b100) alu_ctrl_ID = 4'd4; //xor
		// 		else alu_ctrl_ID = 4'd5; //slt
		// 	end
		// 	else if (op==7'b1100011) begin //beq, bne
		// 		alu_ctrl_ID = 4'd1;
		// 	end
		// 	else begin //jal, sw
		// 		alu_ctrl_ID = 4'd0;
		// 	end
		// end

	//======== CACHE ==================
        always @(*) begin
            //-------- D cache ------------
                DCACHE_ren = ctrl_memread_MEM;
                DCACHE_addr = alu_out_MEM[31:2]; 
                mem_rdata_MEM = {DCACHE_rdata[7:0], DCACHE_rdata[15:8], DCACHE_rdata[23:16], DCACHE_rdata[31:24]};
                // TODO: rd may be overwrite by lw
                //       rd at mem stage may be modified(lw)
                rd_data_MEM_real = (ctrl_memread_MEM)? mem_rdata_MEM : rd_data_MEM;
                // Dmem_write	(wen, addr, data)
                DCACHE_wen = ctrl_memwrite_MEM;
                mem_wdata_EX = rs2_data_EX;     // FIT TB
                // case(ctrl_FB) //genaral case，sw may be forwarded
                // 	2'b00: mem_wdata_EX = rs2_data_EX; 
                // 	2'b01: mem_wdata_EX = rd_data_MEM_real; 
                // 	2'b10: mem_wdata_EX = rd_data_WB; 
                // 	default: mem_wdata_EX = rs2_data_EX; 
                // endcase
                DCACHE_wdata = {mem_wdata_MEM[7:0], mem_wdata_MEM[15:8], mem_wdata_MEM[23:16], mem_wdata_MEM[31:24]};

            //-------- I cache ------------
                ICACHE_ren = 1'b1;  // FIT TB
                //ICACHE_ren = PC_start? 1'b1:1'b0;
                ICACHE_wen = 1'b0;
                ICACHE_wdata = 0;
                ICACHE_addr = PC[31:2]; 
        end

    //======== ALU ====================
        always @(*) begin
            //alu
            case(ctrl_FA)
                2'b00: alu_in1 = rs1_data_EX;
                2'b01: alu_in1 = rd_data_MEM;
                2'b10: alu_in1 = rd_data_WB;
                default: alu_in1 = rs1_data_EX;
            endcase
            case(ctrl_FB)
                2'b00: alu_in2_temp = rs2_data_EX;
                2'b01: alu_in2_temp = rd_data_MEM_real;
                2'b10: alu_in2_temp = rd_data_WB;
                default:alu_in2_temp = rs2_data_EX;
            endcase
            alu_in2 = ctrl_ALUSrc_EX? imme_EX : alu_in2_temp;

            case (alu_ctrl_EX)
                4'd0: alu_out_EX = alu_in1 + alu_in2;
                4'd1: alu_out_EX = alu_in1 - alu_in2;
                4'd7: alu_out_EX = alu_in1 & alu_in2;
                4'd3: alu_out_EX = alu_in1 | alu_in2;
                4'd2: alu_out_EX = alu_in1 ^ alu_in2;
                4'd4: alu_out_EX = alu_in1 < alu_in2;
                4'd8: alu_out_EX = alu_in1 << alu_in2;
                4'd14: alu_out_EX = alu_in1 >> alu_in2;
                4'd15: alu_out_EX = alu_in1 >>> alu_in2;
                default: alu_out_EX = 0; 
            endcase
        end

    //======== PC =====================
        always @(*) begin
            //jalr = rs1 + imme (rs1 forwarded)
            case(ctrl_FA_j) //in fact, only 00,10
                2'b00: PC_FA_j = rs1_data_ID;
                //2'b01: PC_jalr_ID = rd_data_EX;
                2'b10: PC_FA_j = rd_data_MEM;
                default: PC_FA_j = rs1_data_ID;
            endcase
            PC_jalr_ID = imme_ID + PC_FA_j;

            // case(ctrl_FA_j) //in fact, only 01
            // 	//2'b00: compare_rs1 = rs1_data_ID;
            // 	2'b01: compare_rs1 = rd_data_EX;
            // 	//2'b10: compare_rs1 = rd_data_MEM;
            // 	default: compare_rs1 = rs1_data_ID;
            // endcase
            compare_rs1 = rd_data_EX;
            // case(ctrl_FB_j) //in fact, usdless
            // 	2'b00: compare_rs2 = rs2_data_ID;
            // 	2'b01: compare_rs2 = rd_data_EX;
            // 	2'b10: compare_rs2 = rd_data_MEM;
            // 	default: compare_rs2 = rs2_data_ID;
            // endcase
            compare_rs2 = rs2_data_ID;

            //PC_nxt, branch,jal or jalr or pc+4
            PC_B_ID = PC_ID + imme_ID;

            ctrl_bj_taken = ( ((ctrl_beq_ID & compare_rs1==compare_rs2) | ctrl_bne_ID & (compare_rs1!=compare_rs2)) | ctrl_jal_ID);

            //PC_nxt = ctrl_jalr_ID? PC_jalr_ID : ctrl_bj_taken? PC_B_ID : PC+4; //rs1+imme or pc+imme or pc+4;
            PC_nxt = ctrl_bj_taken? PC_B_ID : ctrl_jalr_ID? PC_jalr_ID : PC+4;
        end


	//======== Hazard =================
        always @(*) begin
            // Forwding
            // rs1 at ID, for jalr
            if ( (ctrl_jalr_ID|ctrl_beq_ID|ctrl_bne_ID) & ctrl_regwrite_EX & rd_EX!=0 & rd_EX==rs1_ID) begin //EX
                ctrl_FA_j = 2'b01; 
            end
            else if ( (ctrl_jalr_ID|ctrl_beq_ID|ctrl_bne_ID) & ctrl_regwrite_MEM & rd_MEM!=0 & rd_MEM==rs1_ID) begin //MEM
                ctrl_FA_j = 2'b10; 
            end
            else begin
                ctrl_FA_j = 2'b00;
            end

            // rs2 at ID, for jalr
            // if ( (ctrl_beq_ID|ctrl_bne_ID) & ctrl_regwrite_EX & rd_EX!=0 & rd_EX==rs2_ID) begin //EX
            // 	ctrl_FB_j = 2'b01;  
            // end
            // else if ( (ctrl_beq_ID|ctrl_bne_ID) & ctrl_regwrite_MEM & rd_MEM!=0 & rd_MEM==rs2_ID) begin //MEM
            // 	ctrl_FB_j = 2'b10;  
            // end
            // else begin
            // 	ctrl_FB_j = 2'b00;
            // end

            //(rs1 at EX)
            if (ctrl_regwrite_MEM & rd_MEM!=0 & rd_MEM==rs1_EX) begin
                ctrl_FA = 2'b01; 
            end
            else if (ctrl_regwrite_WB & rd_WB!=0 & rd_WB==rs1_EX) begin
                ctrl_FA = 2'b10; 
            end
            else begin
                ctrl_FA = 2'b00;
            end

            //(rs2 at EX)
            if (ctrl_regwrite_MEM & rd_MEM!=0 & rd_MEM==rs2_EX) begin //& type_EX!=3'd1 if I-type，no need of rs2
                ctrl_FB = 2'b01;
            end
            else if (ctrl_regwrite_WB & rd_WB!=0 & rd_WB==rs2_EX) begin
                ctrl_FB = 2'b10;
            end
            else begin
                ctrl_FB = 2'b00;
            end


            //load use hazard
            ctrl_lw_stall = (ctrl_memread_EX & (rd_EX==rs1_ID | rd_EX==rs2_ID));
        end
	//======== BranchPrediction =======
		assign brancheqaul = (rs1_data_ID == rs2_data_ID)
//======== Sequential Circuit =========================
	always @(posedge clk ) begin
		if (!rst_n) begin
			for (i = 0 ; i<32; i=i+1) begin
				register[i] <= 0 ;
			end
			imme_EX <= 0 ;

			//ctrl
			ctrl_jalr_EX <= 0;
			ctrl_jal_EX  <= 0;

			ctrl_memread_EX <= 0;
			ctrl_memread_MEM <= 0;

			ctrl_memtoreg_EX <= 0;
			ctrl_memtoreg_MEM <= 0;
			ctrl_memtoreg_WB <= 0;

			ctrl_memwrite_EX <= 0;
			ctrl_memwrite_MEM <= 0;

			ctrl_regwrite_EX <= 0;
			ctrl_regwrite_MEM <= 0;
			ctrl_regwrite_WB <= 0;

			ctrl_ALUSrc_EX <= 0;

			//inst
			inst_ID <= inst_IF;

			//alu
			alu_ctrl_EX <= 0;
			alu_out_MEM <= 0;
			alu_out_WB <= 0;

			//register
			rs1_EX <= 0;			
			rs2_EX <= 0;			
			rd_EX <= 0;
			rd_MEM <= 0;
			rd_WB <= 0;
		
			rs1_data_EX <= 0; 
			rs2_data_EX <= 0; 

			rd_data_MEM <= 0;
			rd_data_WB <= 0;

			//PC
			PC_ID <= 0;
			PC_EX <= 0;
			PC <= 0;
			PC_start <= 0;

		end
        //======== usual case =========
		else if (!ICACHE_stall & !DCACHE_stall ) begin
			if (ctrl_regwrite_MEM & rd_MEM!=0) begin
				register[rd_MEM] <= rd_data_MEM_real; //comb?
			end
			else begin
				register[rd_MEM] <= register[rd_MEM];
			end

			imme_EX <= (!ctrl_lw_stall)? imme_ID : 0;
			
			//ctrl
			ctrl_jalr_EX <= (!ctrl_lw_stall)? ctrl_jalr_ID : 0; 
			ctrl_jal_EX  <= (!ctrl_lw_stall)? ctrl_jal_ID : 0; 

			ctrl_memread_EX <= (!ctrl_lw_stall)? ctrl_memread_ID : 0;
			ctrl_memread_MEM <= ctrl_memread_EX;

			ctrl_memtoreg_EX <= (!ctrl_lw_stall)? ctrl_memtoreg_ID : 0;
			ctrl_memtoreg_MEM <= ctrl_memtoreg_EX;
			ctrl_memtoreg_WB <= ctrl_memtoreg_MEM;

			ctrl_memwrite_EX <= (!ctrl_lw_stall)? ctrl_memwrite_ID : 0;
			ctrl_memwrite_MEM <= ctrl_memwrite_EX;

			ctrl_regwrite_EX <= (!ctrl_lw_stall)? ctrl_regwrite_ID : 0;
			ctrl_regwrite_MEM <= ctrl_regwrite_EX;
			ctrl_regwrite_WB <= ctrl_regwrite_MEM;

			ctrl_ALUSrc_EX <= (!ctrl_lw_stall)? ctrl_ALUSrc_ID : 0;

			//memory
			mem_wdata_MEM <= mem_wdata_EX;

			//inst
			inst_ID <= (!ctrl_lw_stall)? inst_IF : inst_ID;

			//alu
			alu_out_MEM <= alu_out_EX;
			alu_out_WB <= alu_out_MEM;

			alu_ctrl_EX <= (!ctrl_lw_stall)? alu_ctrl_ID : 0;

			//register
			rs1_EX <= (!ctrl_lw_stall)? rs1_ID : 0;			
			rs2_EX <= (!ctrl_lw_stall)? rs2_ID : 0;			
			rd_EX  <= (!ctrl_lw_stall)? rd_ID : 0;
			rs1_data_EX <= (!ctrl_lw_stall)? rs1_data_ID : 0; 
			rs2_data_EX <= (!ctrl_lw_stall)? rs2_data_ID : 0; 

			rd_MEM <= rd_EX;
			rd_WB <= rd_MEM;
			rd_data_MEM <= rd_data_EX;
			rd_data_WB <= rd_data_MEM_real; 

			//PC
			PC_ID <= (!ctrl_lw_stall)? PC : PC_ID;
			PC_EX <= (!ctrl_lw_stall)? PC_ID : 0;
			PC_start <= 1;
			PC <= (!ctrl_lw_stall)? PC_nxt : PC;

		end
		//======== stall ==============
		else begin 
			register[rd_MEM] <= register[rd_MEM];

			//ctrl
			ctrl_jalr_EX <= ctrl_jalr_EX;
			ctrl_jal_EX  <= ctrl_jal_EX; 

			ctrl_memread_EX <= ctrl_memread_EX;
			ctrl_memread_MEM <= ctrl_memread_MEM;

			ctrl_memtoreg_EX <= ctrl_memtoreg_EX;
			ctrl_memtoreg_MEM <= ctrl_memtoreg_MEM;
			ctrl_memtoreg_WB <= ctrl_memtoreg_WB;

			ctrl_memwrite_EX <= ctrl_memwrite_EX;
			ctrl_memwrite_MEM <= ctrl_memwrite_MEM;

			ctrl_regwrite_EX <= ctrl_regwrite_EX;
			ctrl_regwrite_MEM <= ctrl_regwrite_MEM;
			ctrl_regwrite_WB <= ctrl_regwrite_WB;

			ctrl_ALUSrc_EX <= ctrl_ALUSrc_EX;

			//inst
			inst_ID <= inst_ID;

			//imme
			imme_EX <= imme_EX;

			//alu
			alu_ctrl_EX <= alu_ctrl_EX;
			alu_out_MEM <= alu_out_MEM;
			alu_out_WB <= alu_out_WB;

			//register
			rs1_EX <= rs1_EX;			
			rs2_EX <= rs2_EX;			
			rd_EX <= rd_EX;
			rd_MEM <= rd_MEM;
			rd_WB <= rd_WB;
		
			rs1_data_EX <= rs1_data_EX; 
			rs2_data_EX <= rs2_data_EX; 

			rd_data_MEM <= rd_data_MEM;
			rd_data_WB <= rd_data_WB;

			//PC
			PC_ID <= PC_ID;
			PC_EX <= PC_EX;
			PC <= PC;
			PC_start <= PC_start;
			
		end
	end
endmodule
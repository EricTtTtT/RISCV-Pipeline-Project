/*=========================================================
Description:
	Add L2 cache for each L1 I_cache & D_cache

TODO:
	1. L2 set associated effect (2-way v.s. dm)
	2. L2 size effect

Notes:
	TODO:   need to check again
	FIT TB: modify design for fitting testbench
	====:   for larger commanc
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
					DCACHE_wen,
					DCACHE_ren,
					DCACHE_stall,   
					ICACHE_wen,
					ICACHE_ren,
					ICACHE_stall   
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
	output			DCACHE_ren;
	output			DCACHE_stall;
	output			ICACHE_wen;
	output			ICACHE_ren;
	output			ICACHE_stall;
	//--------------------------

		// wire declaration  
		// Processor 和 Cache之間
		// wire        ICACHE_ren;
		// wire        ICACHE_wen;
		wire [29:0] ICACHE_addr;
		wire [31:0] ICACHE_wdata;
		// wire        ICACHE_stall;
		wire [31:0] ICACHE_rdata;

		// wire        DCACHE_ren;
		// wire        DCACHE_wen;
		wire [29:0] DCACHE_addr;
		wire [31:0] DCACHE_wdata;
		// wire        DCACHE_stall;
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

//============ I cache (read only) ====================
	//======== dm =====================
		// module Icache(
		// 	clk, proc_reset,
		// 	proc_read, proc_write, proc_addr, proc_rdata, proc_wdata, proc_stall,
		// 	mem_read, mem_write, mem_addr, mem_rdata, mem_wdata, mem_ready );
		// 	//---- input/output definition ----------------------
		// input          clk;
		// // processor interface
		// input         		proc_reset;
		// input         		proc_read, proc_write;
		// input [29:0]		proc_addr;
		// input [31:0]		proc_wdata;
		// output reg     		proc_stall;
		// output reg [31:0]	proc_rdata;
		// // memory interface
		// input  [127:0] mem_rdata;
		// input          mem_ready;
		// output         mem_read, mem_write;
		// output [27:0] mem_addr;
		// output [127:0] mem_wdata;

		// //---- L2 I/O ---------------------
		// wire [127:0]    L2_rdata;
		// wire            L2_stall;
		// reg             L2_read, L2_write;
		// reg [27:0]      L2_addr;
		// reg [127:0]     L2_wdata;
		// parameter NUM_BLOCKS = 8;
		// parameter BLOCK_ADDR_SIZE = 3;  // log2 NUM_BLOCKS

		// 	L2cache inst_L2_cache(
		// 		.clk(clk), .proc_reset(proc_reset),
		// 		.proc_read(L2_read), .proc_write(L2_write), .proc_addr({L2_addr, 2'b00}), .proc_wdata(L2_wdata),
		// 		.proc_rdata(L2_rdata),  .proc_stall(L2_stall),
		// 		.mem_read(mem_read), .mem_write(mem_write), .mem_addr(mem_addr), .mem_rdata(mem_rdata), .mem_wdata(mem_wdata), .mem_ready(mem_ready)
		// 	);
			

		// 	// block = [cache1]
		// 	// cache = [word0, word1, word2, word3] 
		// 	parameter TAG_SIZE = 28-BLOCK_ADDR_SIZE;  // 30 - 2 - BLOCK_ADDR_SIZE
		// 	parameter BLOCK_hSIZE = 130+TAG_SIZE;  // 1+1+TAG_SIZE+128
		// 	parameter IDLE = 2'd0;
		// 	parameter COMP = 2'd1;
		// 	parameter ALLOC = 2'd3;
		// 	integer i;

		// 	//---- wire/reg definition ----------------------------
		// 	wire valid1, dirty1, hit1;
		// 	wire [BLOCK_ADDR_SIZE-1:0] block_addr;
		// 	wire [TAG_SIZE-1:0] tag;
		// 	wire [BLOCK_hSIZE-1:0] cache1_select;

		// 	// flip flops
		// 	reg [1:0] state, state_next;
		// 	reg [BLOCK_hSIZE-1:0] cache1 [0:NUM_BLOCKS-1];
		// 	reg [BLOCK_hSIZE-1:0] cache1_next [0:NUM_BLOCKS-1];

		// 	//==== Finite State Machine ===========================
		// 	always @(*) begin
		// 		case(state)
		// 			IDLE: begin
		// 				state_next = COMP;
		// 			end
		// 			COMP: begin
		// 				state_next = (proc_stall==1'b0)? COMP : ALLOC;
		// 			end
		// 			ALLOC: begin
		// 				state_next = !L2_stall? COMP : ALLOC;                
		// 			end
		// 			default: state_next = state;
		// 		endcase
		// 	end

		// 	//==== Combinational Circuit ==========================
		// 	assign block_addr = proc_addr[1+BLOCK_ADDR_SIZE : 2];
		// 	assign tag = proc_addr[29 : 30-TAG_SIZE];
		// 	assign cache1_select = cache1[block_addr];
			
		// 	assign valid1 = cache1_select[BLOCK_hSIZE-1];
		// 	assign dirty1 = cache1_select[BLOCK_hSIZE-2];
		// 	assign hit1 = valid1 & (cache1_select[127+TAG_SIZE : 128] == tag);
			
		// 	//---- I/O signals ------------------------------------
		// 	always @(*) begin
		// 		proc_stall = ((state==COMP & hit1) | !(proc_read)) ? 0 : 1;
		// 		case ({hit1, proc_addr[1:0]})
		// 			3'b100: proc_rdata = cache1_select[31:0];
		// 			3'b101: proc_rdata = cache1_select[63:32];
		// 			3'b110: proc_rdata = cache1_select[95:64];
		// 			3'b111: proc_rdata = cache1_select[127:96];
		// 			default: proc_rdata = 32'd0;
		// 		endcase
		// 		L2_read = state==ALLOC;
		// 		L2_write = 0;
		// 		L2_addr = proc_addr[29:2];
		// 		L2_wdata = 0;
		// 	end

		// 	//---- handle cache_next and lru bits -----------------
		// 	always @(*) begin
		// 		for (i=0; i<NUM_BLOCKS; i=i+1) begin
		// 			cache1_next[i] = cache1[i];
		// 		end

		// 		case(state)
		// 			ALLOC: begin
		// 				cache1_next[block_addr][127:0] = L2_rdata;
		// 				cache1_next[block_addr][127+TAG_SIZE : 128] = tag;
		// 				cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
		// 			end
		// 		endcase
		// 	end

		// 	//==== Sequential Circuit =============================
		// 	always@( posedge clk ) begin
		// 		if( proc_reset ) begin
		// 			for (i=0; i<NUM_BLOCKS; i=i+1) begin
		// 				cache1[i] <= 0;
		// 			end
		// 			state <= IDLE;
		// 		end else begin
		// 			for (i=0; i<NUM_BLOCKS; i=i+1) begin
		// 				cache1[i] <= cache1_next[i];
		// 			end
		// 			state <= state_next;
		// 		end
		// 	end

		// endmodule

	//============ 2-way ==============
		module Icache(
			clk, proc_reset,
			proc_read, proc_write, proc_addr, proc_rdata, proc_wdata, proc_stall,
			mem_read, mem_write, mem_addr, mem_rdata, mem_wdata, mem_ready );
			//---- input/output definition ----------------------
		input          clk;
		// processor interface
		input         		proc_reset;
		input         		proc_read, proc_write;
		input [29:0]		proc_addr;
		input [31:0]		proc_wdata;
		output reg     		proc_stall;
		output reg [31:0]	proc_rdata;
		// memory interface
		input  [127:0] mem_rdata;
		input          mem_ready;
		output         mem_read, mem_write;
		output [27:0] mem_addr;
		output [127:0] mem_wdata;

		//---- L2 I/O ---------------------
		wire [127:0]    L2_rdata;
		wire            L2_stall;
		reg             L2_read, L2_write;
		reg [27:0]      L2_addr;
		reg [127:0]     L2_wdata;

			L2cache inst_L2_cache(
				.clk(clk), .proc_reset(proc_reset),
				.proc_read(L2_read), .proc_write(L2_write), .proc_addr({L2_addr, 2'b00}), .proc_wdata(L2_wdata),
				.proc_rdata(L2_rdata),  .proc_stall(L2_stall),
				.mem_read(mem_read), .mem_write(mem_write), .mem_addr(mem_addr), .mem_rdata(mem_rdata), .mem_wdata(mem_wdata), .mem_ready(mem_ready)
			);
			
			parameter NUM_BLOCKS = 4;
			parameter BLOCK_ADDR_SIZE = 2;  // log2 NUM_BLOCKS
			parameter BLOCK_SIZE = 128;


			// block = [cache1, cache2]
			// cache = [word0, word1, word2, word3] 
			parameter TAG_SIZE = 28-BLOCK_ADDR_SIZE;  // 30 - 2 - BLOCK_ADDR_SIZE
			parameter BLOCK_TOTAL = 1+TAG_SIZE+BLOCK_SIZE;  // 1+TAG_SIZE+128, remove dirty bit
			parameter IDLE = 2'd0;
			parameter COMP = 2'd1;
			parameter ALLOC = 2'd3;
			integer i;

			//---- wire/reg definition ----------------------------
			wire valid1, hit1;
			wire valid2, hit2;
			wire hit;
			wire [BLOCK_ADDR_SIZE-1:0] block_addr;
			wire [TAG_SIZE-1:0] tag;
			wire [BLOCK_TOTAL-1:0] cache1_select, cache2_select;

			// flip flops
			reg [1:0] state, state_next;
			reg [BLOCK_TOTAL-1:0] cache1 [0:NUM_BLOCKS-1];
			reg [BLOCK_TOTAL-1:0] cache1_next [0:NUM_BLOCKS-1];
			reg [BLOCK_TOTAL-1:0] cache2 [0:NUM_BLOCKS-1];
			reg [BLOCK_TOTAL-1:0] cache2_next [0:NUM_BLOCKS-1];
			reg [NUM_BLOCKS-1:0] lru, lru_next;  // low --> last used is cache 2, use cache1 first
			// reg mem_ready_ff;
			// reg [127:0] mem_rdata_ff;

			//==== Finite State Machine ===========================
			always @(*) begin
				case(state)
					IDLE: begin
						state_next = COMP;
					end
					COMP: begin
						state_next = (proc_stall==1'b0)? COMP : ALLOC;
					end
					ALLOC: begin
						state_next = !L2_stall? COMP : ALLOC;                
					end
					default: state_next = state;
				endcase
			end

			//==== Combinational Circuit ==========================
			assign block_addr = proc_addr[1+BLOCK_ADDR_SIZE : 2];
			assign tag = proc_addr[29 : 30-TAG_SIZE];
			assign cache1_select = cache1[block_addr];
			assign cache2_select = cache2[block_addr];
			
			assign valid1 = cache1_select[BLOCK_TOTAL-1];
			assign valid2 = cache2_select[BLOCK_TOTAL-1];
			assign hit1 = valid1 & (cache1_select[BLOCK_SIZE+TAG_SIZE-1 : BLOCK_SIZE] == tag);
			assign hit2 = valid2 & (cache2_select[BLOCK_SIZE+TAG_SIZE-1 : BLOCK_SIZE] == tag);
			assign hit = hit1 | hit2;
			
			//---- I/O signals ------------------------------------
			always @(*) begin
				proc_stall = ((state==COMP & hit) | !proc_read) ? 0 : 1;
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
				L2_read = state==ALLOC;
				L2_write = 1'b0;
				L2_addr = proc_addr[29:2];
				L2_wdata = 128'd0;
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
				case({state, lru[block_addr]})  // ALLOC: 2'b11
					3'b110: begin
						cache1_next[block_addr][BLOCK_SIZE-1 : 0] = L2_rdata;
						cache1_next[block_addr][BLOCK_SIZE+TAG_SIZE-1 : BLOCK_SIZE] = tag;
						cache1_next[block_addr][BLOCK_TOTAL-1] = 1'b1;
					end
					3'b111: begin
						cache2_next[block_addr][BLOCK_SIZE-1 : 0] = L2_rdata;
						cache2_next[block_addr][BLOCK_SIZE+TAG_SIZE-1 : BLOCK_SIZE] = tag;
						cache2_next[block_addr][BLOCK_TOTAL-1] = 1'b1;
					end
					default: begin
						cache1_next[block_addr] = cache1_select;
						cache2_next[block_addr] = cache2_select;
					end
				endcase
			end

			//==== Sequential Circuit =============================
			always@( posedge clk ) begin
				if( proc_reset ) begin
					for (i=0; i<NUM_BLOCKS; i=i+1) begin
						cache1[i] <= 128'd0;
						cache2[i] <= 128'd0;
						lru[i] <= 0;
					end
					state <= IDLE;
				end else begin
					for (i=0; i<NUM_BLOCKS; i=i+1) begin
						cache1[i] <= cache1_next[i];
						cache2[i] <= cache2_next[i];
						lru[i] <= lru_next[i];
					end
					state <= state_next;
				end
			end

		endmodule


//============ D cache ================================
	//========= fully =================
		// module Dcache(
		// 	clk, proc_reset,
		// 	proc_read, proc_write, proc_addr, proc_rdata, proc_wdata, proc_stall,
		// 	mem_read, mem_write, mem_addr, mem_rdata, mem_wdata, mem_ready );
		// 	//---- input/output definition ----------------------
		// input          clk;
		// // processor interface
		// input         		proc_reset;
		// input         		proc_read, proc_write;
		// input [29:0]		proc_addr;
		// input [31:0]		proc_wdata;
		// output reg     		proc_stall;
		// output reg [31:0]	proc_rdata;
		// // memory interface
		// input  [127:0] mem_rdata;
		// input          mem_ready;
		// output         mem_read, mem_write;
		// output [27:0] mem_addr;
		// output [127:0] mem_wdata;

		// //---- L2 I/O ---------------------
		// wire [127:0]    L2_rdata;
		// wire            L2_stall;
		// reg             L2_read, L2_write;
		// reg [27:0]      L2_addr;
		// reg [127:0]     L2_wdata;

		// 	L2cache inst_L2_cache(
		// 		.clk(clk), .proc_reset(proc_reset),
		// 		.proc_read(L2_read), .proc_write(L2_write), .proc_addr({L2_addr, 2'b00}), .proc_wdata(L2_wdata),
		// 		.proc_rdata(L2_rdata),  .proc_stall(L2_stall),
		// 		.mem_read(mem_read), .mem_write(mem_write), .mem_addr(mem_addr), .mem_rdata(mem_rdata), .mem_wdata(mem_wdata), .mem_ready(mem_ready)
		// 	);
			
		// 	parameter NUM_BLOCKS = 8;
		// 	parameter BLOCK_ADDR_SIZE = 3;  // log2 NUM_BLOCKS

		// 	// block = [cache1]
		// 	// cache = [word0, word1, word2, word3] 
		// 	parameter TAG_SIZE = 28-BLOCK_ADDR_SIZE;  // 30 - 2 - BLOCK_ADDR_SIZE
		// 	parameter BLOCK_hSIZE = 130+TAG_SIZE;  // 1+1+TAG_SIZE+128
		// 	parameter IDLE = 2'd0;
		// 	parameter COMP = 2'd1;
		// 	parameter WRITE = 2'd2;
		// 	parameter ALLOC = 2'd3;
		// 	integer i;

		// 	//---- wire/reg definition ----------------------------
		// 	wire miss;
		// 	wire miss_individual [0:NUM_BLOCKS-1];
		// 	reg [BLOCK_ADDR_SIZE-1:0] block_addr, block_addr_next;

		// 	wire [TAG_SIZE-1:0] tag;
		// 	wire [BLOCK_hSIZE-1:0] cache_select;

		// 	// flip flops
		// 	reg [1:0] state, state_next;
		// 	reg [BLOCK_hSIZE-1:0] cache [0:NUM_BLOCKS-1];
		// 	reg [BLOCK_hSIZE-1:0] cache_next [0:NUM_BLOCKS-1];

		// 	//==== Finite State Machine & NL =======================
		// 	always @(*) begin
		// 		case(state)
		// 			IDLE: begin
		// 				state_next = COMP;
		// 				block_addr = 0;
		// 				block_addr_next = 0;
		// 			end
		// 			COMP: begin
		// 				state_next = (proc_stall==1'b0)? COMP : (dirty)? WRITE : ALLOC;
		// 				block_addr_next = block_addr;
		// 			end
		// 			WRITE: begin
		// 				state_next = !L2_stall? ALLOC : WRITE;
		// 				block_addr_next = block_addr;
		// 			end
		// 			ALLOC: begin
		// 				state_next = !L2_stall? COMP : ALLOC;
		// 				block_addr_next = block_addr;            
		// 			end
		// 			default: begin
		// 				state_next = state;
		// 				block_addr_next = block_addr;
		// 			end
		// 		endcase
		// 	end

		// 	//==== Combinational Circuit ==========================
		// 	assign block_addr = proc_addr[1+BLOCK_ADDR_SIZE : 2];
		// 	assign tag = proc_addr[29 : 30-TAG_SIZE];
		// 	assign cache_select = cache[block_addr];
		// 	assign valid = cache_select[BLOCK_hSIZE-1];
		// 	assign dirty = cache_select[BLOCK_hSIZE-2];
		// 	assign hit = valid & (cache_select[127+TAG_SIZE : 128] == tag);

		// 	assign miss_individual[0] = !(cache[0][157] && (proc_addr[29:2] == cache[0][155:128]));
		// 	assign miss_individual[1] = !(cache[1][157] && (proc_addr[29:2] == cache[1][155:128]));
		// 	assign miss_individual[2] = !(cache[2][157] && (proc_addr[29:2] == cache[2][155:128])); 
		// 	assign miss_individual[3] = !(cache[3][157] && (proc_addr[29:2] == cache[3][155:128])); 
		// 	assign miss_individual[4] = !(cache[4][157] && (proc_addr[29:2] == cache[4][155:128]));
		// 	assign miss_individual[5] = !(cache[5][157] && (proc_addr[29:2] == cache[5][155:128]));
		// 	assign miss_individual[6] = !(cache[6][157] && (proc_addr[29:2] == cache[6][155:128]));
		// 	assign miss_individual[7] = !(cache[7][157] && (proc_addr[29:2] == cache[7][155:128]));  
		// 	assign miss = (proc_read|proc_write) & (miss_individual[0] && miss_individual[1] && 
		// 					miss_individual[2] && miss_individual[3] && 
		// 					miss_individual[4] && miss_individual[5] && 
		// 					miss_individual[6] && miss_individual[7]);
			
		// 	//---- I/O signals ------------------------------------
		// 	always @(*) begin
		// 		proc_stall = ((state==COMP & hit) | !(proc_read | proc_write)) ? 0 : 1;
		// 		case ({hit, proc_addr[1:0]})
		// 			3'b100: proc_rdata = cache_select[31:0];
		// 			3'b101: proc_rdata = cache_select[63:32];
		// 			3'b110: proc_rdata = cache_select[95:64];
		// 			3'b111: proc_rdata = cache_select[127:96];
		// 			default: proc_rdata = 32'd0;
		// 		endcase
		// 		L2_read = state==ALLOC;
		// 		L2_write = state==WRITE;
		// 		case ({state})  // WRITE is 2'b10
		// 			2'b10: L2_addr = {cache_select[127+TAG_SIZE : 128], block_addr};
		// 			default: L2_addr = proc_addr[29:2];
		// 		endcase
		// 		L2_wdata = cache_select[127:0];
		// 	end

		// 	//---- handle cache_next and lru bits -----------------
		// 	always @(*) begin
		// 		for (i=0; i<NUM_BLOCKS; i=i+1) begin
		// 			cache_next[i] = cache[i];
		// 		end

		// 		case(state)
		// 			COMP: begin
		// 				if (proc_write) begin
		// 					case ({hit1, proc_addr[1:0]})
		// 						3'b100: cache_next[block_addr][31:0] = proc_wdata;
		// 						3'b101: cache_next[block_addr][63:32] = proc_wdata;
		// 						3'b110: cache_next[block_addr][95:64] = proc_wdata;
		// 						3'b111: cache_next[block_addr][127:96] = proc_wdata;
		// 					endcase
		// 					if (hit1) begin     // TODO: how to write in if else?
		// 						cache_next[block_addr][127+TAG_SIZE : 128] = tag;
		// 						cache_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b11;
		// 					end
		// 				end else begin
		// 					cache_next[block_addr] = cache_select;
		// 				end
		// 			end
		// 			WRITE: begin
		// 				if (!L2_stall) begin
		// 					cache_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
		// 				end
		// 			end
		// 			ALLOC: begin
		// 				if (!dirty1) begin
		// 					cache_next[block_addr][127:0] = L2_rdata;
		// 					cache_next[block_addr][127+TAG_SIZE : 128] = tag;
		// 					cache_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
		// 				end
		// 			end
		// 		endcase
		// 	end

		// 	//==== Sequential Circuit =============================
		// 	always@( posedge clk ) begin
		// 		if( proc_reset ) begin
		// 			for (i=0; i<NUM_BLOCKS; i=i+1) begin
		// 				cache[i] <= 0;
		// 			end
		// 			state <= IDLE;
		// 			block_addr <= 0;
		// 		end else begin
		// 			for (i=0; i<NUM_BLOCKS; i=i+1) begin
		// 				cache[i] <= cache_next[i];
		// 			end
		// 			state <= state_next;
		// 			block_addr <= block_addr_next;
		// 		end
		// 	end

		// endmodule

	//============ dm =================
		// module Dcache(
		// 	clk, proc_reset,
		// 	proc_read, proc_write, proc_addr, proc_rdata, proc_wdata, proc_stall,
		// 	mem_read, mem_write, mem_addr, mem_rdata, mem_wdata, mem_ready );
		// 	//---- input/output definition ----------------------
		// input          clk;
		// // processor interface
		// input         		proc_reset;
		// input         		proc_read, proc_write;
		// input [29:0]		proc_addr;
		// input [31:0]		proc_wdata;
		// output reg     		proc_stall;
		// output reg [31:0]	proc_rdata;
		// // memory interface
		// input  [127:0] mem_rdata;
		// input          mem_ready;
		// output         mem_read, mem_write;
		// output [27:0] mem_addr;
		// output [127:0] mem_wdata;
		// parameter NUM_BLOCKS = 8;
		// parameter BLOCK_ADDR_SIZE = 3;  // log2 NUM_BLOCKS

		// 	//---- L2 I/O ---------------------
		// 	wire [127:0]    L2_rdata;
		// 	wire            L2_stall;
		// 	reg             L2_read, L2_write;
		// 	reg [27:0]      L2_addr;
		// 	reg [127:0]     L2_wdata;

		// 	L2cache inst_L2_cache(
		// 		.clk(clk), .proc_reset(proc_reset),
		// 		.proc_read(L2_read), .proc_write(L2_write), .proc_addr({L2_addr, 2'b00}), .proc_wdata(L2_wdata),
		// 		.proc_rdata(L2_rdata),  .proc_stall(L2_stall),
		// 		.mem_read(mem_read), .mem_write(mem_write), .mem_addr(mem_addr), .mem_rdata(mem_rdata), .mem_wdata(mem_wdata), .mem_ready(mem_ready)
		// 	);
			

		// 	// block = [cache1]
		// 	// cache = [word0, word1, word2, word3] 
		// 	parameter TAG_SIZE = 28-BLOCK_ADDR_SIZE;  // 30 - 2 - BLOCK_ADDR_SIZE
		// 	parameter BLOCK_hSIZE = 130+TAG_SIZE;  // 1+1+TAG_SIZE+128
		// 	parameter IDLE = 2'd0;
		// 	parameter COMP = 2'd1;
		// 	parameter WRITE = 2'd2;
		// 	parameter ALLOC = 2'd3;
		// 	integer i;

		// 	//---- wire/reg definition ----------------------------
		// 	wire valid1, dirty1, hit1;
		// 	wire [BLOCK_ADDR_SIZE-1:0] block_addr;
		// 	wire [TAG_SIZE-1:0] tag;
		// 	wire [BLOCK_hSIZE-1:0] cache1_select;

		// 	// flip flops
		// 	reg [1:0] state, state_next;
		// 	reg [BLOCK_hSIZE-1:0] cache1 [0:NUM_BLOCKS-1];
		// 	reg [BLOCK_hSIZE-1:0] cache1_next [0:NUM_BLOCKS-1];

		// 	//==== Finite State Machine ===========================
		// 	always @(*) begin
		// 		case(state)
		// 			IDLE: begin
		// 				state_next = COMP;
		// 			end
		// 			COMP: begin
		// 				state_next = (proc_stall==1'b0)? COMP : (dirty1)? WRITE : ALLOC;
		// 			end
		// 			WRITE: begin
		// 				state_next = !L2_stall? ALLOC : WRITE;
		// 			end
		// 			ALLOC: begin
		// 				state_next = !L2_stall? COMP : ALLOC;                
		// 			end
		// 			default: state_next = state;
		// 		endcase
		// 	end

		// 	//==== Combinational Circuit ==========================
		// 	assign block_addr = proc_addr[1+BLOCK_ADDR_SIZE : 2];
		// 	assign tag = proc_addr[29 : 30-TAG_SIZE];
		// 	assign cache1_select = cache1[block_addr];
			
		// 	assign valid1 = cache1_select[BLOCK_hSIZE-1];
		// 	assign dirty1 = cache1_select[BLOCK_hSIZE-2];
		// 	assign hit1 = valid1 & (cache1_select[127+TAG_SIZE : 128] == tag);
			
		// 	//---- I/O signals ------------------------------------
		// 	always @(*) begin
		// 		proc_stall = ((state==COMP & hit1) | !(proc_read | proc_write)) ? 0 : 1;
		// 		case ({hit1, proc_addr[1:0]})
		// 			3'b100: proc_rdata = cache1_select[31:0];
		// 			3'b101: proc_rdata = cache1_select[63:32];
		// 			3'b110: proc_rdata = cache1_select[95:64];
		// 			3'b111: proc_rdata = cache1_select[127:96];
		// 			default: proc_rdata = 32'd0;
		// 		endcase
		// 		L2_read = state==ALLOC;
		// 		L2_write = state==WRITE;
		// 		case ({state})  // WRITE is 2'b10
		// 			2'b10: L2_addr = {cache1_select[127+TAG_SIZE : 128], block_addr};
		// 			default: L2_addr = proc_addr[29:2];
		// 		endcase
		// 		L2_wdata = cache1_select[127:0];
		// 	end

		// 	//---- handle cache_next and lru bits -----------------
		// 	always @(*) begin
		// 		for (i=0; i<NUM_BLOCKS; i=i+1) begin
		// 			cache1_next[i] = cache1[i];
		// 		end

		// 		case(state)
		// 			COMP: begin
		// 				if (proc_write) begin
		// 					case ({hit1, proc_addr[1:0]})
		// 						3'b100: cache1_next[block_addr][31:0] = proc_wdata;
		// 						3'b101: cache1_next[block_addr][63:32] = proc_wdata;
		// 						3'b110: cache1_next[block_addr][95:64] = proc_wdata;
		// 						3'b111: cache1_next[block_addr][127:96] = proc_wdata;
		// 					endcase
		// 					if (hit1) begin     // TODO: how to write in if else?
		// 						cache1_next[block_addr][127+TAG_SIZE : 128] = tag;
		// 						cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b11;
		// 					end
		// 				end else begin
		// 					cache1_next[block_addr] = cache1_select;
		// 				end
		// 			end
		// 			WRITE: begin
		// 				if (!L2_stall) begin
		// 					cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
		// 				end
		// 			end
		// 			ALLOC: begin
		// 				if (!dirty1) begin
		// 					cache1_next[block_addr][127:0] = L2_rdata;
		// 					cache1_next[block_addr][127+TAG_SIZE : 128] = tag;
		// 					cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
		// 				end
		// 			end
		// 		endcase
		// 	end

		// 	//==== Sequential Circuit =============================
		// 	always@( posedge clk ) begin
		// 		if( proc_reset ) begin
		// 			for (i=0; i<NUM_BLOCKS; i=i+1) begin
		// 				cache1[i] <= 0;
		// 			end
		// 			state <= IDLE;
		// 		end else begin
		// 			for (i=0; i<NUM_BLOCKS; i=i+1) begin
		// 				cache1[i] <= cache1_next[i];
		// 			end
		// 			state <= state_next;
		// 		end
		// 	end

		// endmodule

	//============ 2-way ==============
		module Dcache(
		    clk, proc_reset,
		    proc_read, proc_write, proc_addr, proc_rdata, proc_wdata, proc_stall,
		    mem_read, mem_write, mem_addr, mem_rdata, mem_wdata, mem_ready );
		    //---- input/output definition ----------------------
		input          clk;
		// processor interface
		input         		proc_reset;
		input         		proc_read, proc_write;
		input [29:0]		proc_addr;
		input [31:0]		proc_wdata;
		output reg     		proc_stall;
		output reg [31:0]	proc_rdata;
		// memory interface
		input  [127:0] mem_rdata;
		input          mem_ready;
		output         mem_read, mem_write;
		output [27:0] mem_addr;
		output [127:0] mem_wdata;
		parameter NUM_BLOCKS = 4;
		parameter BLOCK_ADDR_SIZE = 2;  // log2 NUM_BLOCKS

			//---- L2 I/O ---------------------
			wire [127:0]    L2_rdata;
			wire            L2_stall;
			reg             L2_read, L2_write;
			reg [27:0]      L2_addr;
			reg [127:0]     L2_wdata;

				L2cache inst_L2_cache(
					.clk(clk), .proc_reset(proc_reset),
					.proc_read(L2_read), .proc_write(L2_write), .proc_addr({L2_addr, 2'b00}), .proc_wdata(L2_wdata),
					.proc_rdata(L2_rdata),  .proc_stall(L2_stall),
					.mem_read(mem_read), .mem_write(mem_write), .mem_addr(mem_addr), .mem_rdata(mem_rdata), .mem_wdata(mem_wdata), .mem_ready(mem_ready)
				);
			

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
		    // reg mem_ready_ff;
		    // reg [127:0] mem_rdata_ff;

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
		                state_next = !L2_stall? ALLOC : WRITE;
		            end
		            ALLOC: begin
		                state_next = !L2_stall? COMP : ALLOC;                
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
		        L2_read = state==ALLOC;
		        L2_write = state==WRITE;
		        case ({state, lru[block_addr]})  // WRITE is 2'b10
		            3'b100: L2_addr = {cache1_select[127+TAG_SIZE : 128], block_addr};
		            3'b101: L2_addr = {cache2_select[127+TAG_SIZE : 128], block_addr};
		            default: L2_addr = proc_addr[29:2];
		        endcase
		        L2_wdata = lru[block_addr]? cache2_select[127:0] : cache1_select[127:0];
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
		                    if (hit1) begin     // TODO: how to write in if else?
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
		                if (!L2_stall) begin
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
		                        cache2_next[block_addr][127:0] = L2_rdata;
		                        cache2_next[block_addr][127+TAG_SIZE : 128] = tag;
		                        cache2_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
		                    end else begin
		                        cache1_next[block_addr][127:0] = L2_rdata;
		                        cache1_next[block_addr][127+TAG_SIZE : 128] = tag;
		                        cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
		                    end
		                end else if (!dirty1) begin
		                    cache1_next[block_addr][127:0] = L2_rdata;
		                    cache1_next[block_addr][127+TAG_SIZE : 128] = tag;
		                    cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
		                end else begin
		                    cache2_next[block_addr][127:0] = L2_rdata;
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
		        end else begin
		            for (i=0; i<NUM_BLOCKS; i=i+1) begin
		                cache1[i] <= cache1_next[i];
		                cache2[i] <= cache2_next[i];
		                lru[i] <= lru_next[i];
		            end
		            state <= state_next;
		        end
		    end

		endmodule


//============ L2 cache ===============================
	//============ 2-way ==============
		module L2cache(
			clk, proc_reset,
			proc_read, proc_write, proc_addr, proc_rdata, proc_wdata, proc_stall,
			mem_read, mem_write, mem_addr, mem_rdata, mem_wdata, mem_ready );
			//---- input/output definition ----------------------
		input          clk;
		// processor interface
		input         		proc_reset;
		input         		proc_read, proc_write;
		input	[29:0]		proc_addr;
		input   [127:0]		proc_wdata;
		output reg     		proc_stall;
		output reg [127:0]	proc_rdata;
		// memory interface
		input  [127:0] 		mem_rdata;
		input          		mem_ready;
		output reg        	mem_read, mem_write;
		output reg [27:0]	mem_addr;
		output reg [127:0]	mem_wdata;
		parameter NUM_BLOCKS = 32;
		parameter BLOCK_ADDR_SIZE = 5;  // log2 NUM_BLOCKS
			

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
				if (hit1) begin
					proc_rdata = cache1_select;
				end else begin
					proc_rdata = cache2_select;
				end
				// case ({hit1, hit2, proc_addr[1:0]})
				//     4'b1000: proc_rdata = cache1_select[31:0];
				//     4'b1001: proc_rdata = cache1_select[63:32];
				//     4'b1010: proc_rdata = cache1_select[95:64];
				//     4'b1011: proc_rdata = cache1_select[127:96];
				//     4'b0100: proc_rdata = cache2_select[31:0];
				//     4'b0101: proc_rdata = cache2_select[63:32];
				//     4'b0110: proc_rdata = cache2_select[95:64];
				//     4'b0111: proc_rdata = cache2_select[127:96];
				//     default: proc_rdata = 32'd0;
				// endcase
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
							// case ({hit1, hit2, proc_addr[1:0]})
							//     4'b1000: cache1_next[block_addr][31:0] = proc_wdata;
							//     4'b1001: cache1_next[block_addr][63:32] = proc_wdata;
							//     4'b1010: cache1_next[block_addr][95:64] = proc_wdata;
							//     4'b1011: cache1_next[block_addr][127:96] = proc_wdata;
							//     4'b0100: cache2_next[block_addr][31:0] = proc_wdata;
							//     4'b0101: cache2_next[block_addr][63:32] = proc_wdata;
							//     4'b0110: cache2_next[block_addr][95:64] = proc_wdata;
							//     4'b0111: cache2_next[block_addr][127:96] = proc_wdata;
							// endcase
							if (hit1) begin     // TODO: how to write in if else?
								cache1_next[block_addr][127:0] = proc_wdata;
								cache1_next[block_addr][127+TAG_SIZE : 128] = tag;
								cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b11;
							end
							if (hit2) begin
								cache2_next[block_addr][127:0] = proc_wdata;

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
	
	//========= dm ====================
		// module L2cache(
		// 	clk, proc_reset,
		// 	proc_read, proc_write, proc_addr, proc_rdata, proc_wdata, proc_stall,
		// 	mem_read, mem_write, mem_addr, mem_rdata, mem_wdata, mem_ready );
		// 	//---- input/output definition ----------------------
		// input          clk;
		// // processor interface
		// input         		proc_reset;
		// input         		proc_read, proc_write;
		// input	[29:0]		proc_addr;
		// input   [127:0]		proc_wdata;
		// output reg     		proc_stall;
		// output reg [127:0]	proc_rdata;
		// // memory interface
		// input  [127:0] 		mem_rdata;
		// input          		mem_ready;
		// output reg        	mem_read, mem_write;
		// output reg [27:0]	mem_addr;
		// output reg [127:0]	mem_wdata;
		// parameter NUM_BLOCKS = 64;
		// parameter BLOCK_ADDR_SIZE = 6;  // log2 NUM_BLOCKS
			

		// 	// block = [cache1]
		// 	// cache = [word0, word1, word2, word3] 
		// 	parameter TAG_SIZE = 28-BLOCK_ADDR_SIZE;  // 30 - 2 - BLOCK_ADDR_SIZE
		// 	parameter BLOCK_hSIZE = 130+TAG_SIZE;  // 1+1+TAG_SIZE+128
		// 	parameter IDLE = 2'd0;
		// 	parameter COMP = 2'd1;
		// 	parameter WRITE = 2'd2;
		// 	parameter ALLOC = 2'd3;
		// 	integer i;

		// 	//---- wire/reg definition ----------------------------
		// 	wire valid1, dirty1, hit1;
		// 	wire hit;
		// 	wire [BLOCK_ADDR_SIZE-1:0] block_addr;
		// 	wire [TAG_SIZE-1:0] tag;
		// 	wire [BLOCK_hSIZE-1:0] cache1_select;

		// 	// flip flops
		// 	reg [1:0] state, state_next;
		// 	reg [BLOCK_hSIZE-1:0] cache1 [0:NUM_BLOCKS-1];
		// 	reg [BLOCK_hSIZE-1:0] cache1_next [0:NUM_BLOCKS-1];
		// 	reg mem_ready_ff;
		// 	reg [127:0] mem_rdata_ff;

		// 	//==== Finite State Machine ===========================
		// 	always @(*) begin
		// 		case(state)
		// 			IDLE: begin
		// 				state_next = COMP;
		// 			end
		// 			COMP: begin
		// 				state_next = (proc_stall==1'b0)? COMP : (dirty1)? WRITE : ALLOC;
		// 			end
		// 			WRITE: begin
		// 				state_next = mem_ready_ff? ALLOC : WRITE;
		// 			end
		// 			ALLOC: begin
		// 				state_next = mem_ready_ff? COMP : ALLOC;                
		// 			end
		// 			default: state_next = state;
		// 		endcase
		// 	end

		// 	//==== Combinational Circuit ==========================
		// 	assign block_addr = proc_addr[1+BLOCK_ADDR_SIZE : 2];
		// 	assign tag = proc_addr[29 : 30-TAG_SIZE];
		// 	assign cache1_select = cache1[block_addr];
			
		// 	assign valid1 = cache1_select[BLOCK_hSIZE-1];
		// 	assign dirty1 = cache1_select[BLOCK_hSIZE-2];
		// 	assign hit1 = valid1 & (cache1_select[127+TAG_SIZE : 128] == tag);
		// 	assign hit = hit1;
			
		// 	//---- I/O signals ------------------------------------
		// 	always @(*) begin
		// 		proc_stall = ((state==COMP & hit) | !(proc_read | proc_write)) ? 0 : 1;
		// 		proc_rdata = cache1_select;
				
		// 		// case ({hit1, hit2, proc_addr[1:0]})
		// 		//     4'b1000: proc_rdata = cache1_select[31:0];
		// 		//     4'b1001: proc_rdata = cache1_select[63:32];
		// 		//     4'b1010: proc_rdata = cache1_select[95:64];
		// 		//     4'b1011: proc_rdata = cache1_select[127:96];
		// 		//     4'b0100: proc_rdata = cache2_select[31:0];
		// 		//     4'b0101: proc_rdata = cache2_select[63:32];
		// 		//     4'b0110: proc_rdata = cache2_select[95:64];
		// 		//     4'b0111: proc_rdata = cache2_select[127:96];
		// 		//     default: proc_rdata = 32'd0;
		// 		// endcase
		// 		mem_read = ~mem_ready_ff && state==ALLOC;
		// 		mem_write = ~mem_ready_ff && state==WRITE;
		// 		case ({state})  // WRITE is 2'b10
		// 			2'b10: mem_addr = {cache1_select[127+TAG_SIZE : 128], block_addr};
		// 			default: mem_addr = proc_addr[29:2];
		// 		endcase
		// 		mem_wdata = cache1_select[127:0];
		// 	end

		// 	//---- handle cache_next and lru bits -----------------
		// 	always @(*) begin
		// 		for (i=0; i<NUM_BLOCKS; i=i+1) begin
		// 			cache1_next[i] = cache1[i];
		// 		end

		// 		case(state)
		// 			COMP: begin
		// 				if (proc_write) begin
		// 					// case ({hit1, hit2, proc_addr[1:0]})
		// 					//     4'b1000: cache1_next[block_addr][31:0] = proc_wdata;
		// 					//     4'b1001: cache1_next[block_addr][63:32] = proc_wdata;
		// 					//     4'b1010: cache1_next[block_addr][95:64] = proc_wdata;
		// 					//     4'b1011: cache1_next[block_addr][127:96] = proc_wdata;
		// 					//     4'b0100: cache2_next[block_addr][31:0] = proc_wdata;
		// 					//     4'b0101: cache2_next[block_addr][63:32] = proc_wdata;
		// 					//     4'b0110: cache2_next[block_addr][95:64] = proc_wdata;
		// 					//     4'b0111: cache2_next[block_addr][127:96] = proc_wdata;
		// 					// endcase
		// 					if (hit1) begin     // TODO: how to write in if else?
		// 						cache1_next[block_addr][127:0] = proc_wdata;
		// 						cache1_next[block_addr][127+TAG_SIZE : 128] = tag;
		// 						cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b11;
		// 					end
		// 				end else begin
		// 					cache1_next[block_addr] = cache1_select;
		// 				end
		// 			end
		// 			WRITE: begin
		// 				if (mem_ready_ff) begin
		// 					cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
		// 				end
		// 			end
		// 			ALLOC: begin
		// 				if (!dirty1) begin
		// 					cache1_next[block_addr][127:0] = mem_rdata_ff;
		// 					cache1_next[block_addr][127+TAG_SIZE : 128] = tag;
		// 					cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
		// 				end
		// 			end
		// 		endcase
		// 	end

		// 	//==== Sequential Circuit =============================
		// 	always@( posedge clk ) begin
		// 		if( proc_reset ) begin
		// 			for (i=0; i<NUM_BLOCKS; i=i+1) begin
		// 				cache1[i] <= 0;
		// 			end
		// 			state <= IDLE;
		// 			mem_ready_ff <= 1'b0;
		// 			mem_rdata_ff <= 127'd0;
		// 		end else begin
		// 			for (i=0; i<NUM_BLOCKS; i=i+1) begin
		// 				cache1[i] <= cache1_next[i];
		// 			end
		// 			state <= state_next;
		// 			mem_ready_ff <= mem_ready;
		// 			mem_rdata_ff <= mem_rdata;
		// 		end
		// 	end

		// endmodule

//============ CHIP ===================================

//=========== CHIP =================
module RISCV_Pipeline(
	clk, rst_n, ICACHE_ren, ICACHE_wen, ICACHE_addr, ICACHE_wdata, ICACHE_stall, ICACHE_rdata,
				DCACHE_ren, DCACHE_wen, DCACHE_addr, DCACHE_wdata, DCACHE_stall, DCACHE_rdata
);
//==== input/output definition ============================
	input clk, rst_n;
	input ICACHE_stall, DCACHE_stall;
	input [31:0] ICACHE_rdata, DCACHE_rdata;
	output reg ICACHE_ren, ICACHE_wen, DCACHE_ren, DCACHE_wen;
	output reg [29:0] ICACHE_addr, DCACHE_addr;
	output reg [31:0] ICACHE_wdata, DCACHE_wdata;

//==== input/output definition ============================

	integer i;
//========= PC =========
	reg [31:0] PC;
	reg [31:0] PC_nxt;
	reg [31:0] PC_ID, PC_EX;
	reg [31:0] PC_B_ID, PC_jalr_ID;
	reg [31:0] PC_FA_j;
	// reg PC_start;
//========= PC =========


//========= Registers ========= 
	reg [31:0] register [0:31];
	reg [31:0] register_save [0:31];
	reg [31:0] register_save_nxt [0:31];
	wire [4:0] rs1_ID;	//rs1
	reg [4:0] rs1_EX;	
	wire [31:0] rs1_data_ID;
	reg [31:0] rs1_data_EX;

	wire [4:0] rs2_ID;	//rs2
	reg [4:0] rs2_EX;	
	wire [31:0] rs2_data_ID;
	reg [31:0] rs2_data_EX;

	wire [4:0] rd_ID;	//rd
	reg [4:0] rd_EX;	
	reg [4:0] rd_MEM;	
	reg [4:0] rd_WB;	

	reg signed [31:0] rd_w_EX;
	reg signed [31:0] rd_w_MEM_real;
	reg signed [31:0] rd_w_MEM;
	reg signed [31:0] rd_w_WB;
	
	reg signed [31:0] write_rd_WB;

	reg [31:0] compare_rs1;
	reg [31:0] compare_rs2;
//========= Registers ========= 

//========= Instruction ========= 
	wire [31:0] inst_IF;
	reg [31:0] inst_ID;
	wire [6:0] op;
	wire [2:0] func3;
	wire [6:0] func7;
//========= Instruction ========= 


//========= Immediate =========
	reg [31:0] imme_ID;
	reg [31:0] imme_EX;
//========= Immediate =========

//========= alu ========= 
	reg signed [31:0] alu_in1;
	reg signed [31:0] alu_in2;
	reg signed [31:0] alu_in2_temp;
	reg signed [31:0] alu_out_EX;
	reg signed [31:0] alu_out_MEM;
	reg signed [31:0] alu_out_WB;
	reg [3:0]	alu_ctrl_ID;
	reg [3:0]	alu_ctrl_EX;
//========= alu ========= 


//========= memory ========= 
	reg [31:0] read_data_MEM; //read from mem
	reg [31:0] read_data_WB; 

	reg [31:0] wdata_EX; //write mem
	reg [31:0] wdata_MEM;
//========= memory ========= 

//========= Control unit =========
	reg ctrl_jalr_ID,		ctrl_jalr_EX; 	//PC+4 at EX
	reg ctrl_jal_ID,		ctrl_jal_EX; 	//
	reg ctrl_beq_ID;	//beq 
	reg ctrl_bne_ID;	//beq 
	reg ctrl_memread_ID,	ctrl_memread_EX, 	ctrl_memread_MEM;
	reg ctrl_memtoreg_ID,	ctrl_memtoreg_EX, 	ctrl_memtoreg_MEM, 	ctrl_memtoreg_WB;
	reg ctrl_memwrite_ID, 	ctrl_memwrite_EX, 	ctrl_memwrite_MEM; 
	reg ctrl_regwrite_ID, 	ctrl_regwrite_EX, 	ctrl_regwrite_MEM, 	ctrl_regwrite_WB;
	reg ctrl_ALUSrc_ID, 	ctrl_ALUSrc_EX;

	reg [1:0] ctrl_FA, ctrl_FB;
	reg [1:0] ctrl_FA_j, ctrl_FB_j;

	reg ctrl_lw_stall;

	reg ctrl_bj_taken;
//========= Control unit =========


//========= wire & reg ==============
	reg	[2:0]	type;
	parameter R_type = 3'd0;
	parameter I_type = 3'd2;
	parameter S_type = 3'd1;
	parameter B_type = 3'd3;
	parameter J_type = 3'd6;
//========= wire & reg ==============

//========= Wire assignment ==========
	assign inst_IF = (ctrl_bj_taken|ctrl_jalr_ID)? 32'h00000013:{ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]};
	// assign inst_IF = !PC_start? 32'h00000013 : ctrl_bj_taken? 32'h00000013:ctrl_jalr_ID?32'h00000013:{ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]};

	assign op = inst_ID[6:0];
	assign rd_ID = inst_ID[11:7];  //rd;
	assign rs1_ID = inst_ID[19:15]; //rs1;
	assign rs2_ID = inst_ID[24:20]; //rs2;
	assign func3 = inst_ID[14:12]; 
	assign func7 = inst_ID[31:25];

	assign rs1_data_ID = register[rs1_ID];
	assign rs2_data_ID = register[rs2_ID];
//========= Wire assignment ==========


//========== type =============
	always @(*)begin
		type[2] = op[3];
		type[1] = ( (!op[6])&(!op[5])) | (op[6]&op[5]) ;
		type[0] = op[5] & (!op[4] & !op[2] );
	end
//========== type =============

//========== decode =============
	always @(*)begin
		//default
		imme_ID = 0;
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
		R_type: begin //R-type
			ctrl_regwrite_ID = 1;
			ctrl_ALUSrc_ID = 0;
		end

		I_type: begin //I-type
			ctrl_memread_ID = !op[4] & !op[2];	//lw, read mem
			ctrl_memtoreg_ID = !op[4] & !op[2];	//lw, read mem
			ctrl_regwrite_ID = 1;
			ctrl_ALUSrc_ID = 1;	
			ctrl_jalr_ID = op[2]; //jalr
			//imme_ID = (func3==3'b001 | func3==3'b101)? 
			//			{ {27{1'b0}} , inst_ID[24:20]}: //shamt, slli, srai, srli
			//			{ {21{inst_ID[31]}}, inst_ID[30:20]}; //addi, andi, ori, xori, slli, srai, srli, slti, lw
			imme_ID = { {21{inst_ID[31]}}, inst_ID[30:20]};
		end

		S_type: begin
			ctrl_memwrite_ID = 1;
			ctrl_ALUSrc_ID = 1;			
			imme_ID = { {21{inst_ID[31]}}, inst_ID[30:25], inst_ID[11:7]}; //sw
		end

		B_type: begin
			ctrl_beq_ID = !func3[0];
			ctrl_bne_ID = func3[0];
			ctrl_ALUSrc_ID = 1;			
			imme_ID = { {20{inst_ID[31]}}, inst_ID[7], inst_ID[30:25], inst_ID[11:8], 1'b0 }; //beq, bne
		end

		J_type:begin
			ctrl_jal_ID = 1;
			ctrl_regwrite_ID = 1;
			ctrl_ALUSrc_ID = 1;
			imme_ID = { {12{inst_ID[31]}}, inst_ID[19:12], inst_ID[20], inst_ID[30:25], inst_ID[24:21], 1'b0 }; //jal
		end
		default:begin
			imme_ID = 0;
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
//========== decode =============


//======== alu_ctrl ========
	always @(*)begin
		alu_ctrl_ID[3] = !func3[1] & func3[0];
		alu_ctrl_ID[2] = (func3[2] & func3[0]) | (!func3[2] & func3[1] & op[4]);
		alu_ctrl_ID[1] = func3[2];
		alu_ctrl_ID[0] = (func7[5] & func3[2] & !func3[1] & func3[0]) | (func3[2]&func3[1]) | (func7[5] & !func3[2]  & op[5] );
		alu_ctrl_ID[0] = ( (func7[5] & !func3[1]) & ((func3[2]&func3[0]) | op[5]) )  | (func3[2]&func3[1]);
	end
//======== alu_ctrl ========


//======== mux & comb ckt ========
	always @(*)begin

		//Dmem_read		(wen, addr, data)
		DCACHE_ren = ctrl_memread_MEM;
		DCACHE_addr = alu_out_MEM[31:2]; 
		read_data_MEM = {DCACHE_rdata[7:0],DCACHE_rdata[15:8],DCACHE_rdata[23:16],DCACHE_rdata[31:24]};
		//rd may be overwrite by lw
		rd_w_MEM_real = (ctrl_memread_MEM)? read_data_MEM : rd_w_MEM; //rd at mem stage may be modified(lw)

		//Dmem_write	(wen, addr, data)
		DCACHE_wen = ctrl_memwrite_MEM;
		case(ctrl_FB) //genaral case，sw may be forwarded
			2'b00: wdata_EX = rs2_data_EX; 
			//2'b01: wdata_EX = rd_w_MEM_real; 
			2'b01: wdata_EX = rd_w_MEM; 
			2'b10: wdata_EX = rd_w_WB; 
			default:wdata_EX = rs2_data_EX; 
		endcase
		//wdata_EX = rs2_data_EX; //This is ok
		DCACHE_wdata = {wdata_MEM[7:0],wdata_MEM[15:8],wdata_MEM[23:16],wdata_MEM[31:24]};

		//Icache
		// ICACHE_ren = PC_start? 1'b1:1'b0;
		ICACHE_ren = 1'b1;
		ICACHE_wen = 1'b0;
		ICACHE_wdata = 0;
		ICACHE_addr = PC[31:2]; 

		//alu
		case(ctrl_FA)
			2'b00: alu_in1 = rs1_data_EX;
			2'b01: alu_in1 = rd_w_MEM;
			2'b10: alu_in1 = rd_w_WB;
			default: alu_in1 = rs1_data_EX;
		endcase
		case(ctrl_FB)
			2'b00: alu_in2_temp = rs2_data_EX;
			//2'b01: alu_in2_temp = rd_w_MEM_real;
			2'b01: alu_in2_temp = rd_w_MEM;
			2'b10: alu_in2_temp = rd_w_WB;
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

		//rd_w, rd_data written
		rd_w_EX = (ctrl_jal_EX | ctrl_jalr_EX)? PC_EX+4 : alu_out_EX;
		
		//jalr = rs1 + imme (rs1 forwarded)
		case(ctrl_FA_j) //in fact, only 00,10
			2'b00: PC_FA_j = rs1_data_ID;
			//2'b01: PC_jalr_ID = rd_w_EX;
			2'b10: PC_FA_j = rd_w_MEM;
			default: PC_FA_j = rs1_data_ID;
		endcase
		PC_jalr_ID = imme_ID + PC_FA_j;

		case(ctrl_FA_j) //in fact, only 01
			2'b00: compare_rs1 = rs1_data_ID;
			//2'b01: compare_rs1 = rd_w_EX;
			2'b10: compare_rs1 = rd_w_MEM;
			default: compare_rs1 = rs1_data_ID;
		endcase
		//compare_rs1 = rd_w_EX;
		
		case(ctrl_FB_j) //in fact, usdless
			2'b00: compare_rs2 = rs2_data_ID;
			//2'b01: compare_rs2 = rd_w_EX;
			2'b10: compare_rs2 = rd_w_MEM;
			default: compare_rs2 = rs2_data_ID;
		endcase
		//compare_rs2 = rs2_data_ID;

		//PC_nxt, branch,jal or jalr or pc+4
		PC_B_ID = PC_ID + imme_ID;

		ctrl_bj_taken = ( ((ctrl_beq_ID & compare_rs1==compare_rs2) | ctrl_bne_ID & (compare_rs1!=compare_rs2)) | ctrl_jal_ID);

		//PC_nxt = ctrl_jalr_ID? PC_jalr_ID : ctrl_bj_taken? PC_B_ID : PC+4; //rs1+imme or pc+imme or pc+4;
		PC_nxt = ctrl_bj_taken? PC_B_ID : ctrl_jalr_ID? PC_jalr_ID : PC+4;
	end
//======== mux & comb ckt ========

//comb write register
	always @(*)begin
		write_rd_WB =  ctrl_memtoreg_WB? read_data_WB : rd_w_WB;
		register[0] = 0;

		for (i = 1; i<32; i=i+1)begin
			register[i] = (ctrl_regwrite_WB & (rd_WB==i))? write_rd_WB:register_save[i];
		end
		register_save_nxt[0] = 0;
		for (i = 1; i<32; i=i+1)begin
			register_save_nxt[i] = register[i];
		end
	end
//comb write register

//========= hazard ===========
	always @(*)begin
		//Forwding
		//rs1 at ID, for jalr
		if ( (ctrl_jalr_ID|ctrl_beq_ID|ctrl_bne_ID) & ctrl_regwrite_EX & rd_EX!=0 & rd_EX==rs1_ID)begin //EX
			ctrl_FA_j = 2'b01; 
		end
		else if ( (ctrl_jalr_ID|ctrl_beq_ID|ctrl_bne_ID) & ctrl_regwrite_MEM & rd_MEM!=0 & rd_MEM==rs1_ID)begin //MEM
			ctrl_FA_j = 2'b10; 
		end
		else begin
			ctrl_FA_j = 2'b00;
		end

		//rs2 at ID, for jalr
		if ( (ctrl_beq_ID|ctrl_bne_ID) & ctrl_regwrite_EX & rd_EX!=0 & rd_EX==rs2_ID)begin //EX
			ctrl_FB_j = 2'b01;  
		end
		else if ( (ctrl_beq_ID|ctrl_bne_ID) & ctrl_regwrite_MEM & rd_MEM!=0 & rd_MEM==rs2_ID)begin //MEM
			ctrl_FB_j = 2'b10;  
		end
		else begin
			ctrl_FB_j = 2'b00;
		end

		//(rs1 at EX)
		if (ctrl_regwrite_MEM & rd_MEM!=0 & rd_MEM==rs1_EX)begin
			ctrl_FA = 2'b01; 
		end
		else if (ctrl_regwrite_WB & rd_WB!=0 & rd_WB==rs1_EX)begin
			ctrl_FA = 2'b10; 
		end
		else begin
			ctrl_FA = 2'b00;
		end

		//(rs2 at EX)
		if (ctrl_regwrite_MEM & rd_MEM!=0 & rd_MEM==rs2_EX)begin //& type_EX!=3'd1 if I-type，no need of rs2
			ctrl_FB = 2'b01;
		end
		else if (ctrl_regwrite_WB & rd_WB!=0 & rd_WB==rs2_EX)begin
			ctrl_FB = 2'b10;
		end
		else begin
			ctrl_FB = 2'b00;
		end

		//load use hazard
		//ctrl_lw_stall = (ctrl_memread_EX & (rd_EX==rs1_ID | rd_EX==rs2_ID));
		ctrl_lw_stall = (ctrl_memread_EX & (rd_EX==rs1_ID | rd_EX==rs2_ID)) | ctrl_FA_j==2'b01 | ctrl_FB_j==2'b01;	
	end
//========= hazard ===========

	reg test,waste;
	reg [10:0]count, count_nxt;
	always @(*)begin
		test = (ctrl_FA_j==2'b01) | (ctrl_FB_j==2'b01);
		waste = (inst_IF == 32'h00000013) | ctrl_lw_stall | ICACHE_stall | DCACHE_stall;
		count_nxt = waste? count+1:count;
	end
	always @(posedge clk)begin
		if (!rst_n) count<= 0;
		else count<=count_nxt;
	end

//========= seq ===========
	always @(posedge clk )begin
		if (!rst_n)begin
			// for (i = 0 ; i<32; i=i+1)begin
			// 	register[i] <= 0 ;
			// end
			for (i = 0; i<32; i=i+1)begin
				register_save[i] <= 0;
			end
			read_data_WB <= 0;			
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
			inst_ID <= 32'h00000013;

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

			rd_w_MEM <= 0;
			rd_w_WB <= 0;

			//PC
			PC_ID <= 0;
			PC_EX <= 0;
			PC <= 0;
			// PC_start <= 0;

		end
		else if (!ICACHE_stall & !DCACHE_stall ) begin
			// if (ctrl_regwrite_MEM & rd_MEM!=0)begin
			// 	register[rd_MEM] <= rd_w_MEM_real; //可用comb?
			// end
			// else begin
			// 	register[rd_MEM] <= register[rd_MEM];
			// end
			for (i = 0; i<32; i=i+1)begin
				register_save[i] <= register_save_nxt[i];
			end
			read_data_WB <= read_data_MEM;

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
			wdata_MEM <= wdata_EX;

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
			rd_w_MEM <= rd_w_EX;
			rd_w_WB <= rd_w_MEM_real; 

			//PC
			PC_ID <= (!ctrl_lw_stall)? PC : PC_ID;
			PC_EX <= (!ctrl_lw_stall)? PC_ID : 0;
			// PC_start <= 1;
			PC <= (!ctrl_lw_stall)? PC_nxt : PC;

		end
		//============ stall ================
		else begin 
			//register[rd_MEM] <= register[rd_MEM];
			for (i = 0; i<32; i=i+1)begin
				register_save[i] <= register_save[i];
			end
			read_data_WB <= read_data_WB;

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

			rd_w_MEM <= rd_w_MEM;
			rd_w_WB <= rd_w_WB;

			//PC
			PC_ID <= PC_ID;
			PC_EX <= PC_EX;
			PC <= PC;
			// PC_start <= PC_start;
		end
	end

endmodule
//========= seq ===========


//=========== CHIP =================
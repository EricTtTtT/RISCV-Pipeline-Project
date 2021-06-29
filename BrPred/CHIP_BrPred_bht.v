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
//================= Icache ============================
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

	always @(*)begin
		case(state)
			STATE_compare_tag:begin
				if (miss)begin //clean
					nxt_state = STATE_allocate;
				end
				else begin
					nxt_state = STATE_compare_tag;
				end
			end

			STATE_allocate:begin
				if (mem_ready)begin
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

	always @(*)begin
		proc_rdata = 0 ;
		for (i = 0; i<4; i=i+1)begin
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

				if (!miss)begin
					if (!miss0)begin
						proc_rdata = data0; //hit
					end
					else if (!miss1)begin
						proc_rdata = data1; //hit
					end
				end        
			end

			STATE_allocate:begin
				proc_stall = 1;
				mem_addr  = proc_addr[29:2];

				if (mem_ready)begin
					mem_read = 0;
					if (!cache0[proc_addr[3:2]][155])begin
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
			for (i = 0 ; i < 4 ; i=i+1)begin
				cache0[i] <=  0;
				cache1[i] <=  0;
			end
			state <= STATE_idle;
			mem_addr_temp <= 0;
		end
		else begin
			for (i = 0; i<4; i=i+1)begin
				cache0[i] <= nxt_cache0[i];
				cache1[i] <= nxt_cache1[i];
			end
			state <= nxt_state;
			mem_addr_temp <= mem_addr_temp_nxt;
		end
	end

	endmodule

//================= Icache ============================

//================= Dcache ============================
	module Dcache(
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
	parameter STATE_write_back = 2'b01;
	parameter STATE_allocate = 2'b10;
	parameter STATE_idle = 2'b11;

	integer i;

	reg [156:0] cache0 [0:3]; // "1" bit lru, "1" bit valid, "1"bit dirty, "26" bits tags, "128" bits data. =157 bits
	reg [156:0] cache1 [0:3]; // "1" bit lru, "1" bit valid, "1"bit dirty, "26" bits tags, "128" bits data. =157 bits
	reg [156:0] nxt_cache0 [0:3]; // "1" bit lru, "1" bit valid, "1"bit dirty, "26" bits tags, "128" bits data. =157 bits
	reg [156:0] nxt_cache1 [0:3]; // "1" bit lru, "1" bit valid, "1"bit dirty, "26" bits tags, "128" bits data. =157 bits
	reg [31:0] data0,data1;

	reg [1:0] state;
	reg [1:0] nxt_state;

	reg[6:0] offset;

	wire lru0; //0 first priority
	wire lru1;
	wire dirty;
	wire dirty0;
	wire dirty1;
	wire miss;
	wire miss0;
	wire miss1;

	//assign sets = proc_addr[3:2];
	assign lru0 = cache0[proc_addr[3:2]][156];
	assign lru1 = cache1[proc_addr[3:2]][156];
	assign dirty0 = cache0[proc_addr[3:2]][154];
	assign dirty1 = cache1[proc_addr[3:2]][154];
	assign miss0 = !( cache0[proc_addr[3:2]][155] & (cache0[proc_addr[3:2]][153:128] == proc_addr[29:4]));
	assign miss1 = !( cache1[proc_addr[3:2]][155] & (cache1[proc_addr[3:2]][153:128] == proc_addr[29:4]));
	assign dirty = cache0[proc_addr[3:2]][154] | cache1[proc_addr[3:2]][154];  // dirty0 | dirty1
	assign miss = (proc_read|proc_write) & miss0 & miss1;  // == hit1 | hit2

	//==== combinational circuit ==============================

	always @(*)begin
		case(state)
			STATE_compare_tag:begin
				if (miss & !dirty)begin //clean
					nxt_state = STATE_allocate;
				end

				else if (miss & !dirty0 ) begin //is miss and first row!dirty
					if (!lru0)begin             
						nxt_state = STATE_allocate;
					end
					else begin                  
						nxt_state = STATE_write_back;
					end
				end

				else if (miss & !dirty1 ) begin //is miss and dirty
					if (!lru1)begin
						nxt_state = STATE_allocate;
					end
					else begin
						nxt_state = STATE_write_back;
					end
				end
				else if (miss & dirty)begin
					nxt_state = STATE_write_back;
				end
				else begin
					nxt_state = STATE_compare_tag;
				end
			end

			STATE_allocate:begin
				if (mem_ready)begin
					nxt_state = STATE_compare_tag;
				end
				else begin
					nxt_state = STATE_allocate;
				end
			end

			STATE_write_back:begin
				if (mem_ready)begin
					nxt_state = STATE_allocate;
				end
				else begin
					nxt_state = STATE_write_back;
				end
			end

			STATE_idle:begin
				nxt_state = STATE_compare_tag;
			end
		endcase
	end

	always @(*)begin
		proc_rdata = 0 ;
		for (i = 0; i<4; i=i+1)begin
			nxt_cache0[i] = cache0[i];
			nxt_cache1[i] = cache1[i];
		end
		mem_wdata = 0;
		mem_addr  = 0;
		mem_read = 0;
		mem_write = 0;
		case(proc_addr[1:0])
			2'b00:begin offset=7'd31; data0=cache0[proc_addr[3:2]][31:0]; data1=cache1[proc_addr[3:2]][31:0];end
			2'b01:begin offset=7'd63; data0=cache0[proc_addr[3:2]][63:32]; data1=cache1[proc_addr[3:2]][63:32];end
			2'b10:begin offset=7'd95; data0=cache0[proc_addr[3:2]][95:64]; data1=cache1[proc_addr[3:2]][95:64];end
			2'b11:begin offset=7'd127; data0=cache0[proc_addr[3:2]][127:96]; data1=cache1[proc_addr[3:2]][127:96];end
		endcase

		case(state)
			STATE_compare_tag:begin
				proc_stall = miss;

				if (!miss & proc_read)begin
					if (!miss0)begin
						proc_rdata = data0; //hit
					end
					else begin
						proc_rdata = data1; //hit
					end    
				end
				else if (!miss & proc_write) begin
					if (!miss0)begin
						nxt_cache0[proc_addr[3:2]][ offset -: 32] = proc_wdata;
						nxt_cache0[proc_addr[3:2]][153:128] = proc_addr[29:4]; //tags
						nxt_cache0[proc_addr[3:2]][154] = 1; //dirty
						nxt_cache0[proc_addr[3:2]][155] = 1; //valid
					end
					else begin
						nxt_cache1[proc_addr[3:2]][ offset -: 32] = proc_wdata;
						nxt_cache1[proc_addr[3:2]][153:128] = proc_addr[29:4]; //tags
						nxt_cache1[proc_addr[3:2]][154] = 1; //dirty
						nxt_cache1[proc_addr[3:2]][155] = 1; //valid
					end
				end         
			end

			STATE_allocate:begin
				proc_stall = 1;
				mem_addr  = proc_addr[29:2];

				if (mem_ready)begin
					mem_read = 0;
					if (!cache0[proc_addr[3:2]][156])begin
						nxt_cache0[proc_addr[3:2]][127:0] = mem_rdata; //data
						nxt_cache0[proc_addr[3:2]][153:128] = proc_addr[29:4]; //tags
						nxt_cache0[proc_addr[3:2]][154] = 0; //dirty
						nxt_cache0[proc_addr[3:2]][155] = 1; //valid
						nxt_cache0[proc_addr[3:2]][156] = 1;  //lru0;
						nxt_cache1[proc_addr[3:2]][156] = 0;  //lru1;
					end
					else begin
						nxt_cache1[proc_addr[3:2]][127:0] = mem_rdata; //data
						nxt_cache1[proc_addr[3:2]][153:128] = proc_addr[29:4]; //tags
						nxt_cache1[proc_addr[3:2]][154] = 0; //dirty
						nxt_cache1[proc_addr[3:2]][155] = 1; //valid
						nxt_cache0[proc_addr[3:2]][156] = 0;  //lru0;
						nxt_cache1[proc_addr[3:2]][156] = 1;  //lru1;
					end
				end
				else begin
					mem_read = 1;
				end
			end

			STATE_write_back:begin
				proc_stall = 1;

				if (!cache0[proc_addr[3:2]][156])begin
					mem_wdata = cache0[proc_addr[3:2]][127:0];
					mem_addr  = { cache0[proc_addr[3:2]][153:128] , proc_addr[3:2] }; 
					if (mem_ready)begin
						nxt_cache0[proc_addr[3:2]][155] = 1; //valid
						nxt_cache0[proc_addr[3:2]][154] = 0; //dirty
						nxt_cache0[proc_addr[3:2]][156] = 0;  //lru0; don't change
						nxt_cache1[proc_addr[3:2]][156] = 1;  //lru1;
						mem_read = 0;
						mem_write = 0;
					end
					else begin
						nxt_cache0[proc_addr[3:2]][155] = 1; //valid
						nxt_cache0[proc_addr[3:2]][154] = 1; //dirty
						mem_read = 0;
						mem_write = 1 ;
					end
				end
				else begin
					mem_wdata = cache1[proc_addr[3:2]][127:0];
					mem_addr  = { cache1[proc_addr[3:2]][153:128] , proc_addr[3:2] }; 
					if (mem_ready)begin
						nxt_cache1[proc_addr[3:2]][155] = 1; //valid
						nxt_cache1[proc_addr[3:2]][154] = 0; //dirty
						nxt_cache0[proc_addr[3:2]][156] = 1;  //lru0;
						nxt_cache1[proc_addr[3:2]][156] = 0;  //lru1;
						mem_read = 0;
						mem_write = 0;
					end
					else begin
						nxt_cache1[proc_addr[3:2]][155] = 1; //valid
						nxt_cache1[proc_addr[3:2]][154] = 1; //dirty
						mem_read = 0;
						mem_write = 1 ;
					end
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
			for (i = 0 ; i < 4 ; i=i+1)begin
				cache0[i] <=  { 1'b0 , 1'b1, {155{1'b0}} };
				cache1[i] <=  { 1'b0 , 1'b1, {155{1'b0}} };
			end
			state <= STATE_idle;
		end
		else begin
			for (i = 0; i < 4; i=i+1)begin
				cache0[i] <= nxt_cache0[i];
				cache1[i] <= nxt_cache1[i];
			end
			state <= nxt_state;
		end
	end

	endmodule

//================= Dcache ============================

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
	reg [31:0] PC_B, PC_B_nxt, PC_B_IF;
	reg [31:0] PC_FA_j;
	reg PC_start;
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
	reg [31:0] inst_IF;
	reg [31:0] inst_ID;
	wire [6:0] op;
	wire [2:0] func3;
	wire [6:0] func7;
//========= Instruction ========= 


//========= Immediate =========
    reg [31:0] imme_IF, imme_ID, imme_EX;

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
    reg ctrl_beq_IF, ctrl_beq_ID;	
    reg ctrl_bne_IF, ctrl_bne_ID;	
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
	//assign inst_IF = (ctrl_bj_taken|ctrl_jalr_ID)? 32'h00000013:{ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]};
	//assign inst_IF = !PC_start? 32'h00000013 : ctrl_bj_taken? 32'h00000013:ctrl_jalr_ID?32'h00000013:{ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]};

	assign op = inst_ID[6:0];
	assign rd_ID = inst_ID[11:7];  //rd;
	assign rs1_ID = inst_ID[19:15]; //rs1;
	assign rs2_ID = inst_ID[24:20]; //rs2;
	assign func3 = inst_ID[14:12]; 
	assign func7 = inst_ID[31:25];

	assign rs1_data_ID = register[rs1_ID];
	assign rs2_data_ID = register[rs2_ID];
//========= Wire assignment ==========


//========== state ===========
	reg [1:0] state, nxt_state;
	reg taken;
	parameter state_t1 = 2'b00; //1=strong, 2=weak
	parameter state_t2 = 2'b01;
	parameter state_nt2 = 2'b10;
	parameter state_nt1 = 2'b11;
//========== state ===========


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
		//ctrl_beq_ID = 0;
		//ctrl_bne_ID = 0;
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
			//ctrl_beq_ID = !func3[0];
			//ctrl_bne_ID = func3[0];
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
			//ctrl_beq_ID = 0;
			//ctrl_bne_ID = 0;
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
		ICACHE_ren = PC_start? 1'b1:1'b0;
		//ICACHE_ren = 1'b1;
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
	end
//======== mux & comb ckt ========

//======== taken ==============
	always @(*)begin
		//PC_nxt, branch,jal or jalr or pc+4
		PC_B_ID = PC_ID + imme_ID;
		taken = !ctrl_lw_stall & ( (ctrl_beq_ID & (compare_rs1==compare_rs2)) | (ctrl_bne_ID & (compare_rs1!=compare_rs2)) );
		if (inst_IF[6:0] == 7'b1100011)begin //op
			PC_B_nxt = PC+4;
            ctrl_beq_IF = !inst_IF[12]; //func3[0]
            ctrl_bne_IF = inst_IF[12];
            imme_IF = { {20{inst_IF[31]}}, inst_IF[7], inst_IF[30:25], inst_IF[11:8], 1'b0 }; //beq, bne
		end
		else begin
			PC_B_nxt = PC_B;
			ctrl_beq_IF = 0;
			ctrl_bne_IF = 0;
			imme_IF = 0;
		end
		PC_B_IF = PC + imme_IF;
	end
//======== taken ==============

//============= FSM ======================
	always @(*)begin
		case (state)
			state_t1:begin
				if (!ctrl_lw_stall & (ctrl_beq_ID|ctrl_bne_ID))begin
					if (taken)nxt_state = state_t1;
					else nxt_state = state_t2;
				end
				else begin
					nxt_state = state;
				end
			end
			state_t2:begin
				if (!ctrl_lw_stall & (ctrl_beq_ID|ctrl_bne_ID))begin
					if (taken)nxt_state = state_t1;
					else nxt_state = state_nt1;
				end
				else begin
					nxt_state = state;
				end
			end
			state_nt2:begin
				if (!ctrl_lw_stall & (ctrl_beq_ID|ctrl_bne_ID))begin
					if (!taken)nxt_state = state_nt1;
					else nxt_state = state_t1;
				end
				else begin
					nxt_state = state;
				end
			end
			state_nt1:begin
				if (!ctrl_lw_stall & (ctrl_beq_ID|ctrl_bne_ID))begin
					if (!taken)nxt_state = state_nt1;
					else nxt_state = state_nt2;
				end
				else begin
					nxt_state = state;
				end
			end
		endcase
	end
//============= FSM ======================


reg bht[0:3], bht_nxt[0:3]; //4 block, taken = 1, not taken = 0
reg [1:0] back, back_nxt;
reg last_pred, last_pred_nxt;
reg look ;
always @(*)begin
	for (i=0;i<4;i=i+1)bht_nxt[i] = bht[i];

	back_nxt = !ctrl_lw_stall & (ctrl_beq_ID|ctrl_bne_ID)? {back[0], taken} : back;
	bht_nxt[back] = !ctrl_lw_stall & (ctrl_beq_ID|ctrl_bne_ID)? taken : bht[back];

end

wire wrong;
assign wrong = !ctrl_lw_stall & (ctrl_beq_ID|ctrl_bne_ID) & (bht[back]!=taken);

always @(*)begin
	PC_nxt = ctrl_jalr_ID? PC_jalr_ID : ctrl_jal_ID? PC_B_ID : 
			(ctrl_beq_IF|ctrl_bne_IF)? bht[back_nxt]? PC_B_IF: PC+4 :  (ctrl_beq_ID|ctrl_bne_ID)? (bht[back]!=taken)? taken? PC_B_ID : PC_B : PC+4 : PC+4;
	
	inst_IF = !PC_start? 32'h00000013 : (ctrl_jal_ID|ctrl_jalr_ID)? 32'h00000013 :
			(ctrl_beq_ID|ctrl_bne_ID)? (bht[back]!=taken)? taken? 32'h00000013:32'h00000013:{ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]} : {ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]};
end




//============= PC_nxt, inst_IF ======================
	// always @(*) begin
	// 	if ((state==state_t1)|(state==state_t2))begin
	// 		PC_nxt = ctrl_jalr_ID? PC_jalr_ID : ctrl_jal_ID? PC_B_ID : (ctrl_beq_IF|ctrl_bne_IF)? PC_B_IF : ((ctrl_beq_ID|ctrl_bne_ID)&!taken)? PC_B : PC+4;
	// 		inst_IF = !PC_start? 32'h00000013 : ( ((ctrl_beq_ID|ctrl_bne_ID)&!taken) |ctrl_jal_ID|ctrl_jalr_ID)? 32'h00000013 : {ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]};
	// 	end
	// 	else begin
	// 		PC_nxt = ctrl_jalr_ID? PC_jalr_ID : ctrl_jal_ID? PC_B_ID : ((ctrl_beq_ID|ctrl_bne_ID)&taken)? PC_B_ID : PC+4;
	// 		inst_IF = !PC_start? 32'h00000013 : ( ((ctrl_beq_ID|ctrl_bne_ID)&taken) |ctrl_jal_ID|ctrl_jalr_ID)? 32'h00000013 : {ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]};
	// 	end		
	// end
//============= PC_nxt, inst_IF ======================

//============= comb write register ==============
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
//============= comb write register ==============

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
			ctrl_beq_ID <= 0;
			ctrl_bne_ID <= 0;

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
			PC_B <= 0;
			PC <= 0;
			PC_start <= 0;

			state <= state_nt1;

			for (i=0;i<4;i=i+1) bht[i] <= 0;
			back <= 0;
			last_pred <= 0;

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
			ctrl_beq_ID <= (!ctrl_lw_stall)? ctrl_beq_IF : ctrl_beq_ID;
			ctrl_bne_ID <= (!ctrl_lw_stall)? ctrl_bne_IF : ctrl_bne_ID;

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
			PC_B <= (!ctrl_lw_stall)? PC_B_nxt : PC_B;
			PC <= (!ctrl_lw_stall)? PC_nxt : PC;
			PC_start <= 1;

			state <= nxt_state;

			for (i=0;i<4;i=i+1) bht[i] <= bht_nxt[i];
			back <= back_nxt;
			last_pred <= last_pred_nxt;
		end
		//============ stall ================
		else begin 
			//register[rd_MEM] <= register[rd_MEM];
			for (i = 0; i<32; i=i+1)begin
				register_save[i] <= register_save[i];
			end
			read_data_WB <= read_data_WB;

			//ctrl
			ctrl_beq_ID <=  ctrl_beq_ID;
			ctrl_bne_ID <=  ctrl_bne_ID;

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
			PC_B <= PC_B;
			PC_start <= PC_start;

			state <= state;

			for (i=0;i<4;i=i+1) bht[i] <= bht[i];
			back <= back;
			last_pred <= last_pred;
		end
	end

endmodule
//========= seq ===========


//=========== CHIP =================
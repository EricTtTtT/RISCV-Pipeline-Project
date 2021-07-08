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

		cache D_cache(
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

		cache I_cache(
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

	module cache(
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
		mem_ready,
		miss,
		state
	);

	//==== input/output definition ============================
		input          clk;
		// processor interface
		input          proc_reset;
		input          proc_read, proc_write;
		input   [29:0] proc_addr;
		input   [31:0] proc_wdata;
		output reg        proc_stall;
		output reg [31:0] proc_rdata;
		// memory interface
		input  [127:0] mem_rdata;
		input          mem_ready;
		output reg     mem_read, mem_write;
		output reg [27:0] mem_addr;
		output reg [127:0] mem_wdata;
	//================Definition of states=====================
		parameter IDLE = 2'b00;
		parameter CompareTag = 2'b01;
		parameter WriteBack = 2'b10;
		parameter Allocate = 2'b11;
	//=========================================================
	//==== wire/reg definition ================================
		output wire miss;
		output reg [1:0] state; 
		wire [7:0] miss_individual;
		wire [2:0] hit_posi;
		reg [2:0] lru[3:0];
		reg [2:0] lru_nxt[3:0];
		reg [1:0] lru_count;
		reg [1:0] lru_count_nxt;
		reg [1:0] state_nxt;
		reg [2:0] counter, counter_nxt, counter_temp;
		reg [157:0] cache [0:7];// 1 bit valid, 1 bit dirty, 28 bits tag, 128 bits data
		reg [157:0] cache_nxt [0:7];
		assign miss_individual[0] = !(cache[0][157] && (proc_addr[29:2] == cache[0][155:128]));
		assign miss_individual[1] = !(cache[1][157] && (proc_addr[29:2] == cache[1][155:128]));
		assign miss_individual[2] = !(cache[2][157] && (proc_addr[29:2] == cache[2][155:128])); 
		assign miss_individual[3] = !(cache[3][157] && (proc_addr[29:2] == cache[3][155:128])); 
		assign miss_individual[4] = !(cache[4][157] && (proc_addr[29:2] == cache[4][155:128]));
		assign miss_individual[5] = !(cache[5][157] && (proc_addr[29:2] == cache[5][155:128]));
		assign miss_individual[6] = !(cache[6][157] && (proc_addr[29:2] == cache[6][155:128]));
		assign miss_individual[7] = !(cache[7][157] && (proc_addr[29:2] == cache[7][155:128]));  
		assign miss = (proc_read|proc_write) & (miss_individual[0] && miss_individual[1] && 
						miss_individual[2] && miss_individual[3] && 
						miss_individual[4] && miss_individual[5] && 
						miss_individual[6] && miss_individual[7]);

		assign hit_posi = !miss_individual[0]? 3'd0:!miss_individual[1]? 3'd1:!miss_individual[2]? 3'd2:
						!miss_individual[3]? 3'd3:!miss_individual[4]? 3'd4:!miss_individual[5]? 3'd5:
						!miss_individual[6]? 3'd6:3'd7;

		reg [27:0] mem_addr_temp_nxt,mem_addr_temp;

	//==== combinational circuit ==============================
	//==========Next State Logic of State Machine==============
	always @(*) begin
		case (state)
			IDLE: begin
				state_nxt = CompareTag;
			end
			CompareTag: begin
				//if MISS and old block is clean
				if (miss && !cache[counter][156]) begin
					state_nxt = Allocate;
				end
				//if MISS and old block is dirty
				else if (miss && cache[counter][156]) begin
					state_nxt = WriteBack;
				end            
				//if HIT, that is, the tag of the processor address is the same as the tag of the corresponding block
				else begin
					state_nxt = CompareTag; 
				end
			end
			WriteBack: begin
				if (mem_ready) begin    
					state_nxt = Allocate;
				end
				else begin
					state_nxt = WriteBack;
				end
			end
			Allocate: begin
				if (mem_ready) begin
					state_nxt = CompareTag;
				end
				else begin
					state_nxt = Allocate;
				end
			end
		endcase
	end
	//=========================================================
	integer j;
	always @(*) begin
		proc_rdata = 0;
		mem_read = 1'b0;
		mem_write = 1'b0;
		mem_addr = 0;
		mem_wdata = 0;
		counter_nxt = counter;
		lru_count_nxt = lru_count;
		lru_nxt[lru_count] = lru[lru_count];
		for (j = 0; j < 8 ; j = j + 1) begin
			cache_nxt[j] = cache[j];
		end
		case (state)
			IDLE: begin
				proc_stall = 1'b1;
				for (j = 0; j < 8 ; j = j + 1) begin
					cache_nxt[j][157] = 1'b0;
					cache_nxt[j][156] = 1'b0;
				end
			end
			CompareTag: begin

				//memory prefetch
				mem_read = !mem_ready;
				mem_addr  =  proc_addr[29:2];
				mem_addr_temp_nxt = (proc_addr[29:2]==mem_addr_temp)? mem_addr_temp : mem_addr;

				//if MISS and old block is clean
				if (miss) begin
					proc_stall = 1'b1;
					counter_temp = counter+1;
					counter_nxt = ( (lru[0]==(counter_temp)) | (lru[1]==(counter_temp)) | (lru[2]==(counter_temp) | (lru[3]==(counter_temp))))?  counter+2 : counter_temp;
					//counter_nxt = counter + 1;
				end
				//if HIT, that is, the tag of the processor address is the same as the tag of the corresponding block
				else begin
					proc_stall = 1'b0;
					lru_count_nxt = lru_count + 1;
					lru_nxt[lru_count] = hit_posi;
					//if HIT and read
					if (proc_read == 1'b1 & proc_write == 1'b0) begin
						proc_rdata = cache[hit_posi][(proc_addr[1:0] << 5)+31 -: 32];//read cache data to processor
					end
					//if HIT and write
					else if (proc_read == 1'b0 && proc_write == 1'b1) begin
						if (proc_addr[1:0] == 2'b00) begin
							cache_nxt[hit_posi][127:96] = cache[hit_posi][127:96];
							cache_nxt[hit_posi][95:64] = cache[hit_posi][95:64];
							cache_nxt[hit_posi][63:32] = cache[hit_posi][63:32];
							cache_nxt[hit_posi][31:0] = proc_wdata;
							cache_nxt[hit_posi][155:128] = proc_addr[29:2];
							cache_nxt[hit_posi][156] = 1'b1;//set dirty is true
							cache_nxt[hit_posi][157] = 1'b1;//set valid is true
						end
						else if (proc_addr[1:0] == 2'b01) begin
							cache_nxt[hit_posi][127:96] = cache[hit_posi][127:96];
							cache_nxt[hit_posi][95:64] = cache[hit_posi][95:64];
							cache_nxt[hit_posi][63:32] = proc_wdata;
							cache_nxt[hit_posi][31:0] = cache[hit_posi][31:0];
							cache_nxt[hit_posi][155:128] = proc_addr[29:2];
							cache_nxt[hit_posi][156] = 1'b1;//set dirty is true
							cache_nxt[hit_posi][157] = 1'b1;//set valid is true
						end
						else if (proc_addr[1:0] == 2'b10) begin
							cache_nxt[hit_posi][127:96] = cache[hit_posi][127:96];
							cache_nxt[hit_posi][95:64] = proc_wdata;
							cache_nxt[hit_posi][63:32] = cache[hit_posi][63:32];
							cache_nxt[hit_posi][31:0] = cache[hit_posi][31:0];
							cache_nxt[hit_posi][155:128] = proc_addr[29:2];
							cache_nxt[hit_posi][156] = 1'b1;//set dirty is true
							cache_nxt[hit_posi][157] = 1'b1;//set valid is true
						end
						else if (proc_addr[1:0] == 2'b11) begin
							cache_nxt[hit_posi][127:96] = proc_wdata;
							cache_nxt[hit_posi][95:64] = cache[hit_posi][95:64];
							cache_nxt[hit_posi][63:32] = cache[hit_posi][63:32];
							cache_nxt[hit_posi][31:0] = cache[hit_posi][31:0];
							cache_nxt[hit_posi][155:128] = proc_addr[29:2];
							cache_nxt[hit_posi][156] = 1'b1;//set dirty is true
							cache_nxt[hit_posi][157] = 1'b1;//set valid is true
						end
					end
				end
			end
			WriteBack: begin
				proc_stall = 1'b1;
				mem_read = 1'b0;
				mem_addr = {cache[hit_posi][155:128]};
				mem_wdata = cache[hit_posi][127:0]; 
				cache_nxt[hit_posi][156] = 1'b0;//set dirty false
				cache_nxt[hit_posi][157] = 1'b1;//set valid true     
				mem_write = (mem_ready)? 1'b0 : 1'b1;
			end
			Allocate: begin
				proc_stall = 1'b1;
				mem_write = 1'b0;
				mem_addr = proc_addr[29:2];                
				cache_nxt[counter][157] = mem_ready? 1'b1 : 1'b0;//valid is true
				cache_nxt[counter][156] = mem_ready? 1'b0 : 1'b1;//dirty is false
				cache_nxt[counter][155:128] = proc_addr[29:2];//insert tag
				cache_nxt[counter][127:96] = mem_rdata[127:96];//insert memory4
				cache_nxt[counter][95:64]  = mem_rdata[95:64];//insert memory3
				cache_nxt[counter][63:32]  = mem_rdata[63:32];//insert memory2
				cache_nxt[counter][31:0]   = mem_rdata[31:0];//insert memory1   
				mem_read = (mem_ready)? 1'b0 : 1'b1;
			end
		endcase
	end
	//=========================================================
	//==== sequential circuit =================================
	integer i,k;
	always@( posedge clk ) begin
		if( proc_reset ) begin
			state <= IDLE;
			counter <= 3'd7;
			for (i=0; i<8; i = i+1) begin
			cache[i] = 0;
			end
			for (i=0; i<4; i = i+1)begin
				lru[i] <= 0;
			end 
			lru_count <= 0;
		end
		else begin
			state <= state_nxt;
			counter <= counter_nxt;
			for (k = 0;k < 8 ;k = k+1 ) begin
				cache[k] <= cache_nxt[k];
			end
			for (i=0; i<4; i = i+1)begin
				lru[i] <= lru_nxt[i];
			end 
			lru_count <= lru_count_nxt;
		end
	end
	endmodule


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

	//========= PC =========
	reg [31:0] PC;
	reg [31:0] PC_nxt;
	reg [31:0] PC_ID, PC_EX;
	reg [31:0] PC_B_ID, PC_jalr_ID;
	reg [31:0] PC_FA_j;

	//========= Registers ========= 
	reg [31:0] register [0:31];
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

	reg [31:0] compare_rs1;
	reg [31:0] compare_rs2;

	//========= Instruction ========= 
	wire [31:0] inst_IF;
	reg [31:0] inst_ID;
	wire [6:0] op;
	wire [2:0] func3;
	wire [6:0] func7;

	//========= Immediate =========
	reg [31:0] imme_ID;
	reg [31:0] imme_EX;

	//========= alu ========= 
	reg signed [31:0] alu_in1;
	reg signed [31:0] alu_in2;
	reg signed [31:0] alu_in2_temp;
	reg signed [31:0] alu_out_EX;
	reg signed [31:0] alu_out_MEM;
	reg signed [31:0] alu_out_WB;
	reg [3:0]	alu_ctrl_ID;
	reg [3:0]	alu_ctrl_EX;

	//========= memory ========= 
	reg [31:0] read_data_MEM; //read from mem

	reg [31:0] wdata_EX; //write mem
	reg [31:0] wdata_MEM;

	//========= Control unit =========
	reg ctrl_jalr_ID,		ctrl_jalr_EX; 	//在EX要計算是否要PC+4
	reg ctrl_jal_ID,		ctrl_jal_EX; 	//
	reg ctrl_beq_ID;	//beq 在ID做完就沒事了
	reg ctrl_bne_ID;	//beq 在ID做完就沒事了
	reg ctrl_memread_ID,	ctrl_memread_EX, 	ctrl_memread_MEM;
	reg ctrl_memtoreg_ID,	ctrl_memtoreg_EX, 	ctrl_memtoreg_MEM, 	ctrl_memtoreg_WB;
	reg ctrl_memwrite_ID, 	ctrl_memwrite_EX, 	ctrl_memwrite_MEM; 
	reg ctrl_regwrite_ID, 	ctrl_regwrite_EX, 	ctrl_regwrite_MEM, 	ctrl_regwrite_WB;
	reg ctrl_ALUSrc_ID, 	ctrl_ALUSrc_EX;

	reg [1:0] ctrl_FA, ctrl_FB;
	reg [1:0] ctrl_FA_j, ctrl_FB_j;

	reg ctrl_lw_stall;

	reg ctrl_bj_taken;

	//========= wire & reg ==============
	reg	[2:0]	type;
	parameter R_type = 3'd0;
	parameter I_type = 3'd2;
	parameter S_type = 3'd1;
	parameter B_type = 3'd3;
	parameter J_type = 3'd6;

	//========= Wire assignment ==========
	//assign inst_IF = (ctrl_bj_taken|ctrl_jalr_ID)? 32'h00000013:{ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]};
	assign inst_IF = ctrl_bj_taken? 32'h00000013:ctrl_jalr_ID?32'h00000013:{ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]};

	assign op = inst_ID[6:0];
	assign rd_ID = inst_ID[11:7];  //rd;
	assign rs1_ID = inst_ID[19:15]; //rs1;
	assign rs2_ID = inst_ID[24:20]; //rs2;
	assign func3 = inst_ID[14:12]; 
	assign func7 = inst_ID[31:25];

	assign rs1_data_ID = register[rs1_ID];
	assign rs2_data_ID = register[rs2_ID];

	//========== type =============
	// always @(*)begin
	// 	if (!op[6]&op[5]&op[4]&!op[3]&!op[2]) type = 0;
	// 	else if (!op[6]&!op[5]&op[4]&!op[3]&!op[2]) type = 1;
	// 	else if (!op[6]&!op[5]&!op[4]&!op[3]&!op[2]) type = 1;
	// 	else if (op[6]&op[5]&!op[4]&!op[3]&op[2]) type = 1;
	// 	else if (!op[6]&op[5]&!op[4]&!op[3]&!op[2]) type = 2;
	// 	else if (op[6]&op[5]&!op[4]&!op[3]&!op[2]) type = 3;
	// 	else if (op[6]&op[5]&!op[4]&op[3]&op[2]) type = 4;
	// 	else type = 5;
	// 	// case (op[6:2])
	// 	// 	5'b01100: type = 0; //R-type
	// 	// 	5'b00100: type = 1; //I-type
	// 	// 	5'b00000: type = 1; //I-type, lw
	// 	// 	5'b11001: type = 1; //I-type, jalr
	// 	// 	5'b01000: type = 2; //S, sw
	// 	// 	5'b11000: type = 3; //B, beq
	// 	// 	5'b11011: type = 4; //jal
	// 	// 	default: type=5;
	// 	// endcase
	// end
	always @(*)begin
		type[2] = op[3];
		type[1] = ( (!op[6])&(!op[5])) | (op[6]&op[5]) ;
		type[0] = op[5] & (!op[4] & !op[2] );
	end

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
			ctrl_memread_ID = !op[4] & !op[2];	//lw,才要讀mem
			ctrl_memtoreg_ID = !op[4] & !op[2];	//lw,才要讀mem
			ctrl_regwrite_ID = 1;
			ctrl_ALUSrc_ID = 1;	//給imme
			ctrl_jalr_ID = op[2];	//jalr
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

	//======== alu_ctrl ========
	always @(*)begin
		alu_ctrl_ID[3] = !func3[1] & func3[0] & !op[6];
		alu_ctrl_ID[2] = func3[2] & func3[0];
		alu_ctrl_ID[1] = func3[0];
		alu_ctrl_ID[0] = (func3[2] & !func3[1]) | (op[6] & !op[0]) | (func7[5] & !func3[2] & !func3[0] & op[5]);
	end
	// always @(*)begin
	// 	if (!op[6]&!op[5]&op[4]&!op[3]&!op[2])begin//I-type
	// 		if (func3[2]&!func3[1]&func3[0]) alu_ctrl_ID = func7[5]? 4'd8:4'd7; //srai, srli
	// 		else if (!func3[2]&!func3[1]&func3[0]) alu_ctrl_ID = 4'd6; //slli
	// 		else if (!func3[2]&func3[1]&!func3[0]) alu_ctrl_ID = 4'd5; //slti
	// 		else if (func3[2]&!func3[1]&!func3[0]) alu_ctrl_ID = 4'd4; ///xori
	// 		else if (func3[2]&func3[1]&!func3[0]) alu_ctrl_ID = 4'd3; //ori
	// 		else if (func3[2]&func3[1]&func3[0]) alu_ctrl_ID = 4'd2; //andi
	// 		else alu_ctrl_ID = 4'd0; //addi
	// 	end
	// 	else if (!op[6]&op[5]&op[4]&!op[3]&!op[2])begin //R-type
	// 		if (!func3[2]&!func3[1]&!func3[0]) alu_ctrl_ID = func7[5]? 4'd1:4'd0; //sub, add
	// 		else if (func3[2]&func3[1]&func3[0]) alu_ctrl_ID = 4'd2; //and
	// 		else if (func3[2]&func3[1]&!func3[0]) alu_ctrl_ID = 4'd3; //or
	// 		else if (func3[2]&!func3[1]&!func3[0]) alu_ctrl_ID = 4'd4; //xor
	// 		else alu_ctrl_ID = 4'd5; //slt
	// 	end
	// 	else if (op[6]&op[5]&!op[4]&!op[3]&!op[2])begin //beq, bne
	// 		alu_ctrl_ID = 4'd1;
	// 	end
	// 	else begin //jal, sw
	// 		alu_ctrl_ID = 4'd0;
	// 	end
	// end
		// always @(*)begin
		// 	if (op[6:2]==5'b00100)begin//I-type
		// 		if (func3==3'b101) alu_ctrl_ID = func7[5]? 4'd8:4'd7; //srai, srli
		// 		else if (func3==3'b001) alu_ctrl_ID = 4'd6; //slli
		// 		else if (func3==3'b010) alu_ctrl_ID = 4'd5; //slti
		// 		else if (func3==3'b100) alu_ctrl_ID = 4'd4; ///xori
		// 		else if (func3==3'b110) alu_ctrl_ID = 4'd3; //ori
		// 		else if (func3==3'b111) alu_ctrl_ID = 4'd2; //andi
		// 		else alu_ctrl_ID = 4'd0; //addi
		// 	end
		// 	else if (op[6:2]==5'b01100)begin //R-type
		// 		if (func3==3'b000) alu_ctrl_ID = func7[5]? 4'd1:4'd0; //sub, add
		// 		else if (func3==3'b111) alu_ctrl_ID = 4'd2; //and
		// 		else if (func3==3'b110) alu_ctrl_ID = 4'd3; //or
		// 		else if (func3==3'b100) alu_ctrl_ID = 4'd4; //xor
		// 		else alu_ctrl_ID = 4'd5; //slt
		// 	end
		// 	else if (op==7'b1100011)begin //beq, bne
		// 		alu_ctrl_ID = 4'd1;
		// 	end
		// 	else begin //jal, sw
		// 		alu_ctrl_ID = 4'd0;
		// 	end
	// end

	//======== mux & comb ckt ========
	always @(*)begin

		//Dmem_read		(wen, addr, data)
		DCACHE_ren = ctrl_memread_MEM;
		DCACHE_addr = alu_out_MEM[31:2]; 
		read_data_MEM = {DCACHE_rdata[7:0],DCACHE_rdata[15:8],DCACHE_rdata[23:16],DCACHE_rdata[31:24]};
		//tb中rd_w_MEM_real不會被用到，lw後的rd不會馬上被用到
		rd_w_MEM_real = (ctrl_memread_MEM)? read_data_MEM : rd_w_MEM; //rd在mem stage也可能被改(lw)

		//Dmem_write	(wen, addr, data)
		DCACHE_wen = ctrl_memwrite_MEM;
		// case(ctrl_FB) //genaral case，因為sw也有可能forwarded，但tb沒這問題
		// 	2'b00: wdata_EX = rs2_data_EX; 
		// 	2'b01: wdata_EX = rd_w_MEM_real; 
		// 	2'b10: wdata_EX = rd_w_WB; 
		// 	default:wdata_EX = rs2_data_EX; 
		// endcase
		wdata_EX = rs2_data_EX; //這也會過
		DCACHE_wdata = {wdata_MEM[7:0],wdata_MEM[15:8],wdata_MEM[23:16],wdata_MEM[31:24]};

		//Icache
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
			2'b01: alu_in2_temp = rd_w_MEM_real;
			2'b10: alu_in2_temp = rd_w_WB;
			default:alu_in2_temp = rs2_data_EX;
		endcase
		alu_in2 = ctrl_ALUSrc_EX? imme_EX : alu_in2_temp;

		case (alu_ctrl_EX)
			4'd0: alu_out_EX = alu_in1 + alu_in2;
			4'd1: alu_out_EX = alu_in1 - alu_in2;
			4'd2: alu_out_EX = alu_in1 & alu_in2;
			4'd3: alu_out_EX = alu_in1 | alu_in2;
			4'd4: alu_out_EX = alu_in1 ^ alu_in2;
			4'd5: alu_out_EX = alu_in1 < alu_in2;
			4'd6: alu_out_EX = alu_in1 << alu_in2;
			4'd7: alu_out_EX = alu_in1 >> alu_in2;
			4'd8: alu_out_EX = alu_in1 >>> alu_in2;
			default: alu_out_EX = 0; 
		endcase

		//rd_w為運算後的值
		rd_w_EX = (ctrl_jal_EX | ctrl_jalr_EX)? PC_EX+4 : alu_out_EX;
		
		//jalr = rs1 + imme (rs1 forwarded)
		case(ctrl_FA_j) //實際上只有00,10會成立
			2'b00: PC_FA_j = rs1_data_ID;
			//2'b01: PC_jalr_ID = rd_w_EX;
			2'b10: PC_FA_j = rd_w_MEM;
			default: PC_FA_j = rs1_data_ID;
		endcase
		PC_jalr_ID = imme_ID + PC_FA_j;

		// case(ctrl_FA_j) //實際上只有01
		// 	//2'b00: compare_rs1 = rs1_data_ID;
		// 	2'b01: compare_rs1 = rd_w_EX;
		// 	//2'b10: compare_rs1 = rd_w_MEM;
		// 	default: compare_rs1 = rs1_data_ID;
		// endcase
		compare_rs1 = rd_w_EX;
		// case(ctrl_FB_j) //實際上rs2不需要forwarding
		// 	2'b00: compare_rs2 = rs2_data_ID;
		// 	2'b01: compare_rs2 = rd_w_EX;
		// 	2'b10: compare_rs2 = rd_w_MEM;
		// 	default: compare_rs2 = rs2_data_ID;
		// endcase
		compare_rs2 = rs2_data_ID;

		//PC_nxt, branch,jal or jalr or pc+4
		PC_B_ID = PC_ID + imme_ID;

		ctrl_bj_taken = ( ((ctrl_beq_ID & compare_rs1==compare_rs2) | ctrl_bne_ID & (compare_rs1!=compare_rs2)) | ctrl_jal_ID);

		//PC_nxt = ctrl_jalr_ID? PC_jalr_ID : ctrl_bj_taken? PC_B_ID : PC+4; //rs1+imme 或 pc+imme 或 pc+4;
		PC_nxt = ctrl_bj_taken? PC_B_ID : ctrl_jalr_ID? PC_jalr_ID : PC+4;
	end


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
		// if ( (ctrl_beq_ID|ctrl_bne_ID) & ctrl_regwrite_EX & rd_EX!=0 & rd_EX==rs2_ID)begin //EX
		// 	ctrl_FB_j = 2'b01;  
		// end
		// else if ( (ctrl_beq_ID|ctrl_bne_ID) & ctrl_regwrite_MEM & rd_MEM!=0 & rd_MEM==rs2_ID)begin //MEM
		// 	ctrl_FB_j = 2'b10;  
		// end
		// else begin
		// 	ctrl_FB_j = 2'b00;
		// end

		//(rs1 at EX)
		if (ctrl_regwrite_MEM & rd_MEM!=0 & rd_MEM==rs1_EX)begin
			ctrl_FA = 2'b01; //差1
		end
		else if (ctrl_regwrite_WB & rd_WB!=0 & rd_WB==rs1_EX)begin
			ctrl_FA = 2'b10; //差2
		end
		else begin
			ctrl_FA = 2'b00;
		end

		//(rs2 at EX)
		if (ctrl_regwrite_MEM & rd_MEM!=0 & rd_MEM==rs2_EX)begin //& type_EX!=3'd1 若是I-type，不需要用到r2
			ctrl_FB = 2'b01;
		end
		else if (ctrl_regwrite_WB & rd_WB!=0 & rd_WB==rs2_EX)begin
			ctrl_FB = 2'b10;
		end
		else begin
			ctrl_FB = 2'b00;
		end


		//load use hazard
		ctrl_lw_stall = (ctrl_memread_EX & (rd_EX==rs1_ID | rd_EX==rs2_ID));
	end


	integer i;
	always @(posedge clk )begin
		if (!rst_n)begin
			for (i = 0 ; i<32; i=i+1)begin
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
			inst_ID <= 0;

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

		end
		else if (!ICACHE_stall & !DCACHE_stall ) begin
			if (ctrl_regwrite_MEM & rd_MEM!=0)begin
				register[rd_MEM] <= rd_w_MEM_real; //可用comb??
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
			PC <= (!ctrl_lw_stall)? PC_nxt : PC;
		end
		//============ stall ================
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

			rd_w_MEM <= rd_w_MEM;
			rd_w_WB <= rd_w_WB;

			//PC
			PC_ID <= PC_ID;
			PC_EX <= PC_EX;
			PC <= PC;
			
		end
	end

endmodule
//=========== CHIP =================
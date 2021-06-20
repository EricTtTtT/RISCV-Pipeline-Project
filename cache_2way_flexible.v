/*=============================================== 
Module: cache
Description: 
    Implementation of 2-way cache, with flexible number of blocks.
    Since this is 2-way cache, modify NUM_BLOCKS to half of the total number of blocks.
    Then need to modify BLOCK_ADDR_SIZE, TAG_SIZE as well
TODO:
    Can modify the case of two dirty and not hit
===============================================*/
module cache(
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

module cache_no_write(
    clk, proc_reset,
    proc_read, proc_write, proc_addr, proc_rdata, proc_wdata, proc_stall,
    mem_read, mem_write, mem_addr, mem_rdata, mem_wdata, mem_ready );
    //---- input/output definition ----------------------
input               clk;
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
                state_next = (proc_stall==1'b0)? COMP : ALLOC;
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
    	mem_read = ~mem_ready_ff && state==ALLOC;
    	mem_write = 0;
		mem_addr = proc_addr[29:2];
        mem_wdata = 0;
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

        case({state, dirty1, dirty2})  // ALLOC = 2'd3
            4'b1100: begin
                if (lru[block_addr]) begin
                    cache2_next[block_addr][127:0] = mem_rdata;
                    cache2_next[block_addr][127+TAG_SIZE : 128] = tag;
                    cache2_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
                end else begin
                    cache1_next[block_addr][127:0] = mem_rdata;
                    cache1_next[block_addr][127+TAG_SIZE : 128] = tag;
                    cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
                end
            end
            4'b1101: begin
                cache1_next[block_addr][127:0] = mem_rdata;
                cache1_next[block_addr][127+TAG_SIZE : 128] = tag;
                cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
            end
            4'b1110: begin
                cache2_next[block_addr][127:0] = mem_rdata;
                cache2_next[block_addr][127+TAG_SIZE : 128] = tag;
                cache2_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;   
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
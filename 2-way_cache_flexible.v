/*=============================================== 
Module: cache
Description: 
    Implementation of 2-way cache, with flexible number of blocks.
    Since this is 2-way cache, modify NUM_BLOCKS to half of the total number of blocks.
    Then need to modify BLOCK_ADDR_SIZE, TAG_SIZE as well
===============================================*/
module cache(
    clk, proc_reset,
    proc_read, proc_write, proc_addr, proc_rdata, proc_wdata, proc_stall,
    mem_read, mem_write, mem_addr, mem_rdata, mem_wdata, mem_ready );
    //---- input/output definition ----------------------
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output reg [27:0] mem_addr;
    output reg [127:0] mem_wdata;
    
    //---- wire/reg definition ----------------------------
    parameter NUM_BLOCKS = 64;
	parameter BLOCK_ADDR_SIZE = 6;  // log2 NUM_BLOCKS
	parameter TAG_SIZE = 28-BLOCK_ADDR_SIZE;  // 30 - 2 - BLOCK_ADDR_SIZE
    parameter BLOCK_hSIZE = 130+TAG_SIZE;  // 1+1+TAG_SIZE+128
	// cache = [word0, word1, word2, word3] 
	// block = [cache1, cache2]


    parameter IDLE = 2'd0;
    parameter COMP = 2'd1;
    parameter WRITE = 2'd2;
    parameter ALLOC = 2'd3;
    integer i;

    wire valid1, dirty1, hit1;
    wire valid2, dirty2, hit2;
    wire hit;
    wire [BLOCK_ADDR_SIZE-1:0] block_addr;
    wire [TAG_SIZE-1:0] tag;

    // flip flops
    reg [1:0] state, state_next;
    reg [BLOCK_hSIZE-1:0] cache1 [0:NUM_BLOCKS-1];
    reg [BLOCK_hSIZE-1:0] cache1_next [0:NUM_BLOCKS-1];
    reg [BLOCK_hSIZE-1:0] cache2 [0:NUM_BLOCKS-1];
    reg [BLOCK_hSIZE-1:0] cache2_next [0:NUM_BLOCKS-1];
    reg [NUM_BLOCKS-1:0] lru, lru_next;  // low --> use 1 first

    //==== Combinational Circuit ==========================
    assign block_addr = proc_addr[1+BLOCK_ADDR_SIZE : 2];
    assign tag = proc_addr[29 : 30-TAG_SIZE];
    assign valid1 = cache1[block_addr][BLOCK_hSIZE-1];
    assign dirty1 = cache1[block_addr][BLOCK_hSIZE-2];
    assign valid2 = cache2[block_addr][BLOCK_hSIZE-1];
    assign dirty2 = cache2[block_addr][BLOCK_hSIZE-2];
    assign hit1 = valid1 & (cache1[block_addr][127+TAG_SIZE : 128] == tag);
    assign hit2 = valid2 & (cache2[block_addr][127+TAG_SIZE : 128] == tag);
    assign hit = hit1 | hit2;
    
    assign proc_stall = (state==COMP & (hit | (~proc_read & ~proc_write))) ? 0 : 1;
    assign proc_rdata = (hit1 & proc_read)?
                            proc_addr[1]?
                                proc_addr[0]?
                                    cache1[block_addr][127:96]
                                :   cache1[block_addr][95:64]
                            :   proc_addr[0]?
                                    cache1[block_addr][63:32]
                                :   cache1[block_addr][31:0]
                        : (hit2 & proc_read)?
                            proc_addr[1]?
                                proc_addr[0]?
                                    cache2[block_addr][127:96]
                                :   cache2[block_addr][95:64]
                            :   proc_addr[0]?
                                    cache2[block_addr][63:32]
                                :   cache2[block_addr][31:0]
                        : 32'd0;
    assign mem_read = ~mem_ready && state==ALLOC;
    assign mem_write = ~mem_ready && state==WRITE;

    //---- Finite State Machine ---------------------------
    always @(*) begin
        case(state)
            IDLE: begin
                state_next = COMP;
            end
            COMP: begin
                state_next = (hit | (~proc_read & ~proc_write))? COMP : (dirty1 | dirty2)? WRITE : ALLOC;
            end
            WRITE: begin
                state_next = mem_ready? ALLOC : WRITE;
            end
            ALLOC: begin
                state_next = mem_ready? COMP : ALLOC;                
            end
            default: state_next = state;
        endcase
    end

	//---- Control signals and I/O ------------------------
    always @(*) begin
        //---- I/O signals --------------------------------
        mem_addr = (state==WRITE)?
                        dirty1?
                            {cache1[block_addr][127+TAG_SIZE : 128], block_addr}
                        :   {cache2[block_addr][127+TAG_SIZE : 128], block_addr}
                    : proc_addr[29:2];
        mem_wdata = dirty1? cache1[block_addr][127:0] : cache2[block_addr][127:0];
    
        //---- handle cache_next and lru bits -------------
        for (i=0; i<NUM_BLOCKS; i=i+1) begin
            cache1_next[i] = cache1[i];
            cache2_next[i] = cache2[i];
            lru_next[i] = lru[i];
        end
        lru_next[block_addr] = state==COMP?
                                    hit1? 0 : hit2? 1 : lru[block_addr]
                                : lru[block_addr];

        case(state)
            COMP: begin
                if (hit1 & proc_write) begin
                    cache1_next[block_addr][(proc_addr[1:0])*32+31 -: 32] = proc_wdata;
                    cache1_next[block_addr][127+TAG_SIZE : 128] = tag;
                    cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b11;
                end else if (hit2 & proc_write) begin
                    cache2_next[block_addr][(proc_addr[1:0])*32+31 -: 32] = proc_wdata;
                    cache2_next[block_addr][127+TAG_SIZE : 128] = tag;
                    cache2_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b11;
                end else begin  // TODO: remove else?
                    cache1_next[block_addr] = cache1[block_addr];
                    cache2_next[block_addr] = cache2[block_addr];
                end
            end
            WRITE: begin
                if (mem_ready) begin
                    if (dirty1) begin
                        cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
                    end else begin
                        cache2_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
                    end
                end
            end
            ALLOC: begin
                if (!dirty1 & !dirty2) begin
                    if (lru[block_addr]) begin
                        cache2_next[block_addr][127:0] = mem_rdata;
                        cache2_next[block_addr][127+TAG_SIZE : 128] = tag;
                        cache2_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
                    end else begin
                        cache1_next[block_addr][127:0] = mem_rdata;
                        cache1_next[block_addr][127+TAG_SIZE : 128] = tag;
                        cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
                    end
                end else if (!dirty1) begin
                    cache1_next[block_addr][127:0] = mem_rdata;
                    cache1_next[block_addr][127+TAG_SIZE : 128] = tag;
                    cache1_next[block_addr][BLOCK_hSIZE-1 : BLOCK_hSIZE-2] = 2'b10;
                end else begin
                    cache2_next[block_addr][127:0] = mem_rdata;
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
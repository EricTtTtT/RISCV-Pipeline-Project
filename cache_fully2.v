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

reg [1:0] state;
reg [1:0] nxt_state;
reg finish;
reg nxt_finish;

wire lru0; //0=優先
wire lru1;
//wire [1:0] sets; //4 sets
wire valid0;
wire valid1;
wire dirty;
wire dirty0;
wire dirty1;
wire miss;
wire miss0;
wire miss1;

//assign sets = proc_addr[3:2];
assign lru0 = cache0[proc_addr[3:2]][156];
assign lru1 = cache1[proc_addr[3:2]][156];
assign valid0 = cache0[proc_addr[3:2]][155];
assign valid1 = cache1[proc_addr[3:2]][155];
assign dirty0 = cache0[proc_addr[3:2]][154];
assign dirty1 = cache1[proc_addr[3:2]][154];
assign miss0 = !( cache0[proc_addr[3:2]][155] & (cache0[proc_addr[3:2]][153:128] == proc_addr[29:4]));
assign miss1 = !( cache1[proc_addr[3:2]][155] & (cache1[proc_addr[3:2]][153:128] == proc_addr[29:4]));
assign dirty = cache0[proc_addr[3:2]][154] | cache1[proc_addr[3:2]][154];  // dirty0 | dirty1
assign miss = miss0 & miss1;  // == hit1 | hit2

//==== combinational circuit ==============================

always @(*)begin
    case(state)
        STATE_compare_tag:begin
            if (miss & !dirty)begin //clean
                nxt_state = STATE_allocate;
            end

            else if (miss & !dirty0 ) begin //is miss and 第一排!dirty
                if (!lru0)begin             //第一排的優先的話就allocate
                    nxt_state = STATE_allocate;
                end
                else begin                  //不優先的話就先write_back另一邊，接著再allocate，
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
    nxt_finish = 0 ;
    for (i = 0; i<8; i=i+1)begin
        nxt_cache0[i] = cache0[i];
        nxt_cache1[i] = cache1[i];
    end
    mem_wdata = 0;
    mem_addr  = 0;
    mem_read = 0;
    mem_write = 0;
    
    case(state)
        STATE_compare_tag:begin
            if (miss) proc_stall=1;
            else proc_stall =0;
        
            if (!miss & proc_read)begin
                if (!miss0)begin
                    proc_rdata = cache0[proc_addr[3:2]][ proc_addr[1:0]*32+31 -: 32]; //hit
                end
                else if (!miss1)begin
                    proc_rdata = cache1[proc_addr[3:2]][ proc_addr[1:0]*32+31 -: 32]; //hit
                end
            end
            else if (!miss & proc_write) begin
                if (!miss0)begin
                    nxt_cache0[proc_addr[3:2]][ proc_addr[1:0]*32+31 -: 32] = proc_wdata; //data
                    nxt_cache0[proc_addr[3:2]][153:128] = proc_addr[29:4]; //tags
                    nxt_cache0[proc_addr[3:2]][154] = 1; //dirty
                    nxt_cache0[proc_addr[3:2]][155] = 1; //valid
                end
                else if (!miss1) begin
                    nxt_cache1[proc_addr[3:2]][ proc_addr[1:0]*32+31 -: 32] = proc_wdata; //data
                    nxt_cache1[proc_addr[3:2]][153:128] = proc_addr[29:4]; //tags
                    nxt_cache1[proc_addr[3:2]][154] = 1; //dirty
                    nxt_cache1[proc_addr[3:2]][155] = 1; //valid
                end
            end         
        end

        STATE_allocate:begin
            proc_stall = 1;
            mem_addr  = proc_addr[29:2];
            if (mem_ready & !finish)begin
                if (!cache0[proc_addr[3:2]][156])begin
                    nxt_cache0[proc_addr[3:2]][127:0] = mem_rdata; //data
                    nxt_cache0[proc_addr[3:2]][153:128] = proc_addr[29:4]; //tags
                    nxt_cache0[proc_addr[3:2]][154] = 0; //dirty
                    nxt_cache0[proc_addr[3:2]][155] = 1; //valid
                    nxt_cache0[proc_addr[3:2]][156] = 1;  //lru0;
                    nxt_cache1[proc_addr[3:2]][156] = 0;  //lru1;
                    mem_read = 0;
                    mem_write = 0;
                    nxt_finish = 1;
                end
                else begin
                    nxt_cache1[proc_addr[3:2]][127:0] = mem_rdata; //data
                    nxt_cache1[proc_addr[3:2]][153:128] = proc_addr[29:4]; //tags
                    nxt_cache1[proc_addr[3:2]][154] = 0; //dirty
                    nxt_cache1[proc_addr[3:2]][155] = 1; //valid
                    nxt_cache0[proc_addr[3:2]][156] = 0;  //lru0;
                    nxt_cache1[proc_addr[3:2]][156] = 1;  //lru1;
                    mem_read = 0;
                    mem_write = 0;
                    nxt_finish = 1;
                end
            end
            else begin
                mem_read = 1;
                mem_write = 0;
                nxt_finish = 0;
            end
        end

        STATE_write_back:begin
            proc_stall = 1;

            if (!cache0[proc_addr[3:2]][156])begin
                mem_wdata = cache0[proc_addr[3:2]][127:0];
                mem_addr  = { cache0[proc_addr[3:2]][153:128] , proc_addr[3:2] }; 
                if (mem_ready & !finish)begin
                    nxt_cache0[proc_addr[3:2]][155] = 1; //valid
                    nxt_cache0[proc_addr[3:2]][154] = 0; //dirty
                    nxt_cache0[proc_addr[3:2]][156] = 0;  //lru0; write-back後要read, 順序不變
                    nxt_cache1[proc_addr[3:2]][156] = 1;  //lru1;
                    mem_read = 0;
                    mem_write = 0;
                    nxt_finish = 1;
                end
                else begin
                    nxt_cache0[proc_addr[3:2]][155] = 1; //valid
                    nxt_cache0[proc_addr[3:2]][154] = 1; //dirty
                    mem_read = 0;
                    mem_write = 1 ;
                    nxt_finish = 0;
                end
            end
            else begin
                mem_wdata = cache1[proc_addr[3:2]][127:0];
                mem_addr  = { cache1[proc_addr[3:2]][153:128] , proc_addr[3:2] }; 
                if (mem_ready & !finish)begin
                    nxt_cache1[proc_addr[3:2]][155] = 1; //valid
                    nxt_cache1[proc_addr[3:2]][154] = 0; //dirty
                    nxt_cache0[proc_addr[3:2]][156] = 1;  //lru0;
                    nxt_cache1[proc_addr[3:2]][156] = 0;  //lru1;
                    mem_read = 0;
                    mem_write = 0;
                    nxt_finish = 1;
                end
                else begin
                    nxt_cache1[proc_addr[3:2]][155] = 1; //valid
                    nxt_cache1[proc_addr[3:2]][154] = 1; //dirty
                    mem_read = 0;
                    mem_write = 1 ;
                    nxt_finish = 0;
                end
            end
        end

        STATE_idle:begin
            proc_stall = 1;
        end

    endcase
end

//==== sequential circuit =================================
always@( posedge clk ) begin
    if( proc_reset ) begin
        for (i = 0 ; i < 8 ; i=i+1)begin
            cache0[i] <=  0;
            cache1[i] <=  0;
        end
        state <= STATE_idle;
        finish <= nxt_finish;
    end
    else begin
        for (i = 0; i<8; i=i+1)begin
            cache0[i] <= nxt_cache0[i];
            cache1[i] <= nxt_cache1[i];
        end
        state <= nxt_state;
        finish <= nxt_finish;
    end
end

endmodule
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

reg [154:0] cache [0:7]; // "1" bit valid ,dirty, "25" bits tags, "128" bits data. =152 bits, 8 = 2^3 entries
reg [154:0] nxt_cache [0:7]; 
reg [1:0] state;
reg [1:0] nxt_state;
//wire [2:0] blocks; //8 blocks
wire dirty;
wire valid;
wire miss;

//assign  blocks = proc_addr[4:2];
assign dirty = cache[proc_addr[4:2]][153];
assign valid = cache[proc_addr[4:2]][154];
assign miss = !(cache[proc_addr[4:2]][154] & (cache[proc_addr[4:2]][152:128] == proc_addr[29:5]));

reg finish;
reg nxt_finish;

//==== combinational circuit ==============================

always @(*)begin
    case(state)
        STATE_compare_tag:begin
            if (miss & !dirty)begin //clean
                nxt_state = STATE_allocate;
            end
            else if (miss & dirty ) begin //is miss and dirty
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
    mem_wdata = 0;
    proc_rdata = 0; //hit
    mem_addr  = 0;
    for (i = 0; i<8; i=i+1)begin
        nxt_cache[i] = cache[i];
    end
    nxt_finish = 0;
    mem_read = 0;
    mem_write = 0;

    case(state)
        STATE_compare_tag:begin
            if (miss) proc_stall=1;
            else proc_stall =0;
            
            if (!miss & proc_read)begin // valid & hit(tags)
                proc_rdata = cache[proc_addr[4:2]][ proc_addr[1:0]*32+31 -: 32]; //hit
            end
            else if (!miss & proc_write ) begin
                nxt_cache[proc_addr[4:2]][ proc_addr[1:0]*32+31 -: 32] = proc_wdata; //data
                nxt_cache[proc_addr[4:2]][152:128] = proc_addr[29:5]; //tags
                nxt_cache[proc_addr[4:2]][153] = 1; //dirty
                nxt_cache[proc_addr[4:2]][154] = 1; //valid
            end
        end

        STATE_allocate:begin
            proc_stall = 1;
            mem_addr  = proc_addr[29:2];
            if (mem_ready & !finish)begin
                nxt_cache[proc_addr[4:2]][127:0] = mem_rdata; //data
                nxt_cache[proc_addr[4:2]][152:128] = proc_addr[29:5]; //tags
                nxt_cache[proc_addr[4:2]][153] = 0; //dirty
                nxt_cache[proc_addr[4:2]][154] = 1; //valid
                mem_read = 0;
                mem_write = 0;
                nxt_finish = 1;
            end
            else begin
                mem_read = 1;
                mem_write = 0;
                nxt_finish = 0;
            end              
        end

        STATE_write_back:begin
            proc_stall = 1;
            mem_wdata = cache[proc_addr[4:2]][127:0];
            mem_addr  = { cache[proc_addr[4:2]][152:128] , proc_addr[4:2] };
            if (mem_ready & !finish)begin
                nxt_cache[proc_addr[4:2]][154] = 1; //valid
                nxt_cache[proc_addr[4:2]][153] = 0; //dirty
                mem_read = 0;
                mem_write = 0;
                nxt_finish = 1;
            end
            else  begin
                mem_read = 0;
                mem_write = 1;
                nxt_finish = 0;
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
            cache[i] <=  0;
        end
        state <= STATE_idle;
        finish <= nxt_finish;
    end
    else begin
        for (i = 0; i<8; i=i+1)begin
            cache[i] <= nxt_cache[i];
        end
        state <= nxt_state;
        finish <= nxt_finish;
    end
end

endmodule

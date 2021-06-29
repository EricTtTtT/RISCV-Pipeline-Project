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
    wire miss_individual [0:7];
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
//==== combinational circuit ==============================
//==========Next State Logic of State Machine==============
always @(*) begin
    case (state)
        IDLE: begin
            state_nxt = CompareTag;
            counter = 0;
            counter_nxt = 0;
        end
        CompareTag: begin
            //if MISS and old block is clean
            if (miss && !cache[counter][156]) begin
                state_nxt = Allocate;
                counter_nxt = counter;
            end
            //if MISS and old block is dirty
            else if (miss && cache[counter][156]) begin
                state_nxt = WriteBack;
                counter_nxt = counter;
            end            
            //if HIT, that is, the tag of the processor address is the same as the tag of the corresponding block
            else begin
                state_nxt = CompareTag; 
            end
        end
        WriteBack: begin
            if (mem_ready) begin    
                state_nxt = Allocate;
                counter_nxt = counter;      
            end
            else begin
                state_nxt = WriteBack;
                counter_nxt = counter;
            end
        end
        Allocate: begin
            if (mem_ready) begin
                state_nxt = CompareTag;
                counter_nxt = counter;
            end
            else begin
                state_nxt = Allocate;
                counter_nxt = counter;
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
    for (j = 0; j < 8 ; j = j + 1) begin
        cache_nxt[j] = cache[j];
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
            //if MISS and old block is clean
            if (miss && !cache[counter][156]) begin
                proc_stall = 1'b1;
				counter_nxt = counter + 1;

            end
            //if MISS and old block is dirty
            else if (miss && cache[counter][156]) begin
                proc_stall = 1'b1;
				counter_nxt = counter + 1;
            end
            //if HIT, that is, the tag of the processor address is the same as the tag of the corresponding block
            else begin
                proc_stall = 1'b0;
                //if HIT and read
                if (proc_read == 1'b1 && proc_write == 1'b0) begin
                    proc_rdata = cache[counter][(proc_addr[1:0] << 5)+31 -: 32];//read cache data to processor
                    // if (proc_addr[1:0] == 3) begin
                    //     if (counter == 7) begin
                    //         counter_nxt = 0;
                    //     end
                    //     else begin
                    //         counter_nxt = counter + 1;
                    //     end
                    // end
                    // else begin
                    //     counter_nxt = counter;
                    // end
                end
                //if HIT and write
                else if (proc_read == 1'b0 && proc_write == 1'b1) begin
                    //counter_nxt = counter + 1;
                    // if (proc_addr[1:0] == 3) begin
                    //     if (counter == 7) begin
                    //         counter_nxt = 0;
                    //     end
                    //     else begin
                    //         counter_nxt = counter + 1;
                    //     end
                    // end
                    // else begin
                    //     counter_nxt = counter;
                    // end
                    if (proc_addr[1:0] == 2'b00) begin
                        cache_nxt[counter][127:96] = cache[counter][127:96];
                        cache_nxt[counter][95:64] = cache[counter][95:64];
                        cache_nxt[counter][63:32] = cache[counter][63:32];
                        cache_nxt[counter][31:0] = proc_wdata;
                        cache_nxt[counter][155:128] = proc_addr[29:2];
                        cache_nxt[counter][156] = 1'b1;//set dirty is true
                        cache_nxt[counter][157] = 1'b1;//set valid is true
                    end
                    else if (proc_addr[1:0] == 2'b01) begin
                        cache_nxt[counter][127:96] = cache[counter][127:96];
                        cache_nxt[counter][95:64] = cache[counter][95:64];
                        cache_nxt[counter][63:32] = proc_wdata;
                        cache_nxt[counter][31:0] = cache[counter][31:0];
                        cache_nxt[counter][155:128] = proc_addr[29:2];
                        cache_nxt[counter][156] = 1'b1;//set dirty is true
                        cache_nxt[counter][157] = 1'b1;//set valid is true
                    end
                    else if (proc_addr[1:0] == 2'b10) begin
                        cache_nxt[counter][127:96] = cache[counter][127:96];
                        cache_nxt[counter][95:64] = proc_wdata;
                        cache_nxt[counter][63:32] = cache[counter][63:32];
                        cache_nxt[counter][31:0] = cache[counter][31:0];
                        cache_nxt[counter][155:128] = proc_addr[29:2];
                        cache_nxt[counter][156] = 1'b1;//set dirty is true
                        cache_nxt[counter][157] = 1'b1;//set valid is true
                    end
                    else if (proc_addr[1:0] == 2'b11) begin
                        cache_nxt[counter][127:96] = proc_wdata;
                        cache_nxt[counter][95:64] = cache[counter][95:64];
                        cache_nxt[counter][63:32] = cache[counter][63:32];
                        cache_nxt[counter][31:0] = cache[counter][31:0];
                        cache_nxt[counter][155:128] = proc_addr[29:2];
                        cache_nxt[counter][156] = 1'b1;//set dirty is true
                        cache_nxt[counter][157] = 1'b1;//set valid is true
                    end
                end
            end
        end
        WriteBack: begin
            proc_stall = 1'b1;
            mem_read = 1'b0;
            mem_addr = {cache[counter][155:128]};
            mem_wdata = cache[counter][127:0]; 
            cache_nxt[counter][156] = 1'b0;//set dirty false
            cache_nxt[counter][157] = 1'b1;//set valid true     
            mem_write = (mem_ready)? 1'b0 : 1'b1;
        end
        Allocate: begin
            proc_stall = 1'b1;
            mem_write = 1'b0;
            mem_addr = proc_addr[29:2];                
            cache_nxt[counter][157] = 1'b1;//valid is true
            cache_nxt[counter][156] = 1'b0;//dirty is false
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
        counter <= 0;
        for (i=0; i<8; i = i+1) begin
           cache[i] = 0;
        end
    end
    else begin
        state <= state_nxt;
        counter <= counter_nxt;
        for (k = 0;k < 8 ;k = k+1 ) begin
            cache[k] <= cache_nxt[k];
        end
    end
end
endmodule
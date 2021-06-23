`timescale 1 ns/10 ps
`define TestPort    30'h3FF      // 1023
`define BeginSymbol 32'h00000168
`define EndSymbol   32'hFFFFFD5D
`define CheckNum    10'd7

module	TestBed(
	clk,
	rst,
	addr,
	data,
	wen,
	error_num,
	duration,
	finish,
	DCACHE_ren, DCACHE_wen, DCACHE_stall,
	ICACHE_ren, ICACHE_wen, ICACHE_stall
);
	input			clk, rst;
	input	[29:0]	addr;
	input	[31:0]	data;
	input			wen;
	input DCACHE_ren, DCACHE_wen, DCACHE_stall;
	input ICACHE_ren, ICACHE_wen, ICACHE_stall;

	output	[7:0]	error_num;
	output	[31:0]	duration;
	output			finish;
	reg		[7:0]	error_num;
	reg		[31:0]	duration;
	reg				finish;
	
	reg		[31:0]	answer;

	reg		[1:0]	curstate;
	reg		[1:0]	nxtstate;
	reg		[9:0]	curaddr;
	reg		[9:0]	nxtaddr;
	reg		[7:0]	nxt_error_num;
	reg		[31:0]	nxtduration;
	
	reg				state,state_next;

	wire    [31:0]  data_modify;
		
	parameter	state_idle 	= 2'b00;
	parameter	state_check = 2'b01;
	parameter	state_report= 2'b10;	

	integer f;
	integer total_cycle;
	integer DCACHE_ren_times, DCACHE_wen_times, DCACHE_stall_cycles;
	integer ICACHE_ren_times, ICACHE_wen_times, ICACHE_stall_cycles;
	integer ing_D, ing_I;

	assign data_modify = {data[7:0],data[15:8],data[23:16],data[31:24]}; // convert little-endian format to readable format
		
	always@( posedge clk or negedge rst )						// State-DFF
	begin
		if( ~rst )
		begin
			curstate <= state_idle;
			curaddr  <= 0;
			duration <= 0;
			error_num <= 8'd255;
			state <= 0;
		end
		else
		begin
			curstate <= nxtstate;
			curaddr  <= nxtaddr;
			duration <= nxtduration;
			error_num <= nxt_error_num;
			
			state <= state_next;
		end
	end
			
	always@(*)	// FSM for test
	begin
		finish = 1'b0;
		case( curstate )
		state_idle: 	begin
							nxtaddr = 0;
							nxtduration = 0;
							nxt_error_num = 255;	
							if( addr==`TestPort && data_modify==`BeginSymbol && wen )
							begin
								nxt_error_num = 0;
								nxtstate = state_check;
							end	 	
							else nxtstate = state_idle;
						end
		state_check:	begin
							nxtduration = duration + 1;
							nxtaddr = curaddr;						
							nxt_error_num = error_num;	
							if( addr==`TestPort && wen && state==0 )
							begin
								nxtaddr = curaddr + 1;
								if( data_modify != answer )
									nxt_error_num = error_num + 8'd1;
							end
							nxtstate = curstate;
							if( curaddr==`CheckNum )	
								nxtstate = state_report;
						end
		state_report:	begin
							finish = 1'b1;
							nxtaddr = curaddr;
							nxtstate = curstate;		
							nxtduration = duration;
							nxt_error_num = error_num;	
						end				
		endcase	
	end

	initial begin
		f = $fopen("output.txt","a");
		@(posedge rst);
		@(posedge clk);
		if (f) $display("File open suceessfully: %0d", f);
		else $display("File open error: %0d", f);
		@(curstate == state_report) begin
			$display("write result to output.txt");
			$fwrite(f,"duration: %d\n", duration);
			$fwrite(f,"DCACHE_ren_times: %d\n", DCACHE_ren_times);
			$fwrite(f,"DCACHE_wen_times: %d\n", DCACHE_wen_times);
			$fwrite(f,"DCACHE_stall_cycles: %d\n", DCACHE_stall_cycles);
			$fwrite(f,"ICACHE_ren_times: %d\n", ICACHE_ren_times);
			$fwrite(f,"ICACHE_wen_times: %d\n", ICACHE_wen_times);
			$fwrite(f,"ICACHE_stall_cycles: %d\n", ICACHE_stall_cycles);
		end
		$fclose(f);
	end

	always @(posedge clk) begin
		if (~rst) begin
			DCACHE_ren_times = 0;
			DCACHE_wen_times = 0;
			DCACHE_stall_cycles = 0;
			ICACHE_ren_times = 0;
			ICACHE_wen_times = 0;
			ICACHE_stall_cycles = 0;
			ing_D = 0;
			ing_I = 0;
		end else begin
			total_cycle = total_cycle + 1;
			if (ing_D) begin
				if (!DCACHE_stall) begin
					ing_D = 0;
				end
			end else if (DCACHE_ren && DCACHE_stall) begin
				ing_D = 1;
				DCACHE_ren_times = DCACHE_ren_times + 1;
			end else if (DCACHE_ren && !DCACHE_stall) begin
				ing_D = 0;
				DCACHE_ren_times = DCACHE_ren_times + 1;
			end else if (DCACHE_wen && DCACHE_stall) begin
				ing_D = 1;
				DCACHE_wen_times = DCACHE_wen_times + 1;
			end else if (DCACHE_wen && !DCACHE_stall) begin
				ing_D = 0;
				DCACHE_wen_times = DCACHE_wen_times + 1;
			end
			
			if (ing_I) begin
				if (!ICACHE_stall) begin
					ing_I = 0;
				end
			end else if (ICACHE_ren && ICACHE_stall) begin
				ing_I = 1;
				ICACHE_ren_times = ICACHE_ren_times + 1;
			end else if (ICACHE_ren && !ICACHE_stall) begin
				ing_I = 0;
				ICACHE_ren_times = ICACHE_ren_times + 1;
			end else if (ICACHE_wen && ICACHE_stall) begin
				ing_I = 1;
				ICACHE_wen_times = ICACHE_wen_times + 1;
			end else if (ICACHE_wen && !ICACHE_stall) begin
				ing_I = 0;
				ICACHE_wen_times = ICACHE_wen_times + 1;
			end 

			if (DCACHE_stall)
				DCACHE_stall_cycles = DCACHE_stall_cycles + 1;
			if (ICACHE_stall)
				ICACHE_stall_cycles = ICACHE_stall_cycles + 1;
		end
	end

	always@( negedge clk )						
	begin
		if(curstate == state_report) begin
			$display("--------------------------- Simulation FINISH !!---------------------------");
			if (error_num) begin 
				$display("============================================================================");
				$display("\n (T_T) FAIL!! The simulation result is FAIL!!! there were %d errors at all.\n", error_num);
				$display("============================================================================");
			end
			 else begin 
				$display("============================================================================");
				$display("\n \\(^o^)/ CONGRATULATIONS!!  The simulation result is PASS!!!\n");
				$display("============================================================================");
			end
		end
	end
	
	always@(*)begin//sub-FSM (avoid the Dcache stall condition)
		case(state)
			1'b0:begin
				if(wen)
					state_next=1;
				else
					state_next=state;				
			end
			1'b1:begin
				if(!wen)
					state_next=0;
				else
					state_next=state;	
			end
		endcase
	end
	
	
	always@(*)	// ROM for correct result
	begin
		answer = 0;
		case( curaddr )
		10'd0  : answer = 32'd0;
		10'd1  : answer = 32'd1;
		10'd2  : answer = 32'd1;
		10'd3  : answer = 32'd1;
		10'd4  : answer = 32'd1;
		10'd5  : answer = 32'd0;
		10'd6  : answer = `EndSymbol;
		endcase
	end		
endmodule
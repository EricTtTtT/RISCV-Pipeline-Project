`timescale 1 ns/10 ps

module	TestBed(
	clk,
	rst,
	addr,
	data,
	wen,
	error_num,
	duration,
	finish,
	br,
	wrong
);
	input			clk, rst;
	input	[29:0]	addr;
	input	[31:0]	data;
	input			wen;
	output	[7:0]	error_num;
	output	[15:0]	duration;
	output			finish;
	reg				finish;
	reg		[1:0]	curstate;
	reg		[1:0]	nxtstate;
	reg		[7:0]	error_num,nxt_error_num;
	reg				state,state_next;

	input br, wrong;
	integer f, br_times, wrong_times, dur;

		
	parameter	state_A   = 2'b00;
	parameter	state_B   = 2'b01;
	parameter	state_C   = 2'b10;
	parameter	state_end = 2'b11;
	
	assign duration = 0;


	initial begin
		f = $fopen("output.txt","a");
		@(posedge rst);
		@(posedge clk);
		if (f) $display("File open suceessfully: %0d", f);
		else $display("File open error: %0d", f);
		@(curstate == state_end) begin
			$display("write result to output.txt");
			$fwrite(f,"duration: %d\n", dur);
			$fwrite(f,"br_times: %d\n", br_times);
			$fwrite(f,"wrong_times: %d\n", wrong_times);
		end
		$fclose(f);
	end

	always @(posedge clk) begin
		if (~rst) begin
			br_times = 0;
			wrong_times = 0;
			dur = 0;
		end else begin
			dur = dur + 1;
			if (curstate==state_A | curstate==state_B | curstate==state_C) begin
				if (br) begin
					br_times = br_times + 1;
					if (wrong) begin
						wrong_times = wrong_times + 1;
					end
				end
			end
		end
	end

	always@( posedge clk or negedge rst )						// State-DFF
	begin
		if( ~rst )
		begin
			curstate <= state_A;
			error_num <= 8'd0;

			state <= 0;
		end
		else
		begin
			curstate <= nxtstate;
			error_num <= nxt_error_num;
			
			state <= state_next;
		end
	end
			
	always@(*)	// FSM for test
	begin
		finish = 1'b0;
		case( curstate )
		state_A   : begin
						nxt_error_num = error_num;
						nxtstate = curstate;
						if( addr==0 && wen && !state ) begin
							if (data == 0) begin
								$display("\nBranch Part A is complete.");
								nxtstate = state_B;
							end
							else begin
								nxt_error_num = error_num + 1;
							end	
						end
					end
		state_B   : begin
						nxt_error_num = error_num;
						nxtstate = curstate;
						if( addr==0 && wen && !state ) begin
							if (data == 0) begin
								$display("\nBranch Part B is complete.");
								nxtstate = state_C;
							end
							else begin
								nxt_error_num = error_num + 1;
							end	
						end
					end
		state_C   : begin
						nxt_error_num = error_num;
						nxtstate = curstate;
						if( addr==0 && wen && !state ) begin
							if (data == 0) begin
								$display("\nBranch Part C is complete.\n");
								nxtstate = state_end;
							end
							else begin
								nxt_error_num = error_num + 1;
							end	
						end
					end
		state_end :	begin
						finish = 1'b1;
						nxtstate = curstate;
						nxt_error_num = error_num;	
					end						
		endcase
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

	always@( negedge clk )						
	begin
		if(curstate == state_end) begin
			$display("--------------------------- Simulation FINISH !!---------------------------");
			if (error_num) begin 
				$display("============================================================================");
				$display("\n (T_T) FAIL!! The simulation result is FAIL!!!\n");
				$display("============================================================================");
			end
			 else begin 
				$display("============================================================================");
				$display("\n \\(^o^)/ CONGRATULATIONS!!  The simulation result is PASS!!!\n");
				$display("============================================================================");
			end
		end
	end
endmodule
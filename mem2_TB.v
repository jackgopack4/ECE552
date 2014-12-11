module mem2_TB();
	reg clk, rst_n, re, we;
	reg [15:0] i_addr, d_addr, wrt_data;

	wire i_rdy, d_rdy;
  	wire [15:0] instr, rd_data;

	mem_hierarchy iMH(	clk, 
						rst_n, 
						instr, 
						i_rdy, 
						d_rdy, 
						rd_data, 
						i_addr, 
						d_addr, 
						re, 
						we, 
						wrt_data);

	initial begin
		clk = 0;
		rst_n = 0;
		re = 0;
		we = 0;
		i_addr = 16'b0;
		d_addr = 16'b0;
		wrt_data = 16'b0;
		#5;
		rst_n = 1;
		#400;
		wrt_data = 16'hffff;
		we = 1'b1;
		#20;
		we = 1'b0;
		#10;
		re = 1'b1;
		#10;
		re = 1'b0;
		#20;
		$stop;
	end

	always begin
		#5;
		clk = ~clk;
	end

	always @(posedge clk) begin
		if(d_rdy) begin
			$display("rd_data=%h\n",rd_data);
		end else if(i_rdy) begin
			$display("instr=%h\n",instr);
			i_addr = i_addr + 1;
		end else begin
			//$display("iCache invalid, reading from mem\n");
		end

	end

endmodule

module mem_hierarchy(instr, i_rdy, d_rdy, rd_data, i_addr, d_addr, re, we, wrt_data, clk, rst_n);

  input clk, rst_n, re, we;
  input [15:0] i_addr, d_addr, wrt_data;

  output i_rdy, d_rdy;
  output [15:0] instr, rd_data;

  wire i_hit, d_hit, d_dirt_in, i_we, i_re, m_we, m_re, m_rdy_n, clean, dirty;
  wire [7:0] i_tag, d_tag;
  wire [13:0] m_addr;
  wire [63:0] i_line, d_line, m_line, i_data, d_data, m_data;

  cache iCache( .clk(clk),
  			   	.rst_n(rst_n)
  			   	.addr(i_addr[15:2]),
  			   	.wr_data(i_data),
  			   	.we(i_we),
  			   	.re(i_re),
  			   	.wdirty(clean),
  			   	.hit(i_hit),
  			   	.dirty(clean),
  			   	.rd_data(i_line),
  			   	.tag_out(i_tag));

  cache dCache( .clk(clk),
  				.rst_n(rst_n),
  				.addr(d_addr[15:2]),
  				.wr_data(d_data),
  				.wdirty(d_dirt_in),
  				.we(we),
  				.re(d_acc),
  				.hit(d_hit),
  				.dirty(dirty),
  				.rd_data(d_line),
  				.tag_out(d_tag));

  unifed_mem mem(	.clk(clk),
  					.rst_n(rst_n),
  					.re(m_re),
  					.we(m_we),
  					.addr(m_addr),
  					.wdata(m_data),
  					.rd_data(m_line),
  					.rdy(m_rdy_n));

  cache_controller controller(	.clk(clk),
  								.rst_n(rst_n),
  								/* INSTANTIATE REST HERE */);


  assign i_we = 1'b0;
  assign i_re = 1'b1;
  assign clean = 1'b0;
  assign d_acc = re | we; 

  assign instr = !rst_n ? 16'hB0FF : i_addr[1] ? (i_addr[0] ? i_line[63:48] : i_line[47:32]) : (i_addr[0] ? i_line[31:16] : i_line[15:0]);

  assign data = !rst_n ? 16'h0000 : d_addr[1] ? (d_addr[0] ? d_line[63:48] : d_line[47:32]) : (d_addr[0] ? d_line[31:16] : d_line[15:0]);
  
endmodule
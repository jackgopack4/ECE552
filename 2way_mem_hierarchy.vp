module mem_hierarchy(clk, rst_n, instr, i_rdy, d_rdy, rd_data, i_addr, d_addr, re, we, wrt_data);

  input clk, rst_n, re, we;
  input [15:0] i_addr, d_addr, wrt_data;

  output i_rdy, d_rdy;
  output [15:0] instr, rd_data;

  reg [15:0] instr;

  wire [63:0] i_rd_data, i_wr_data;
  wire [8:0] i_tag;
  wire i_hit, i_dirty, i_we,i_toggle;
  wire [1:0] i_sel;

  wire [13:0] m_addr;
  wire m_re, m_we, m_rdy;
  wire [63:0] m_wr_data, m_rd_data;
  
  cache iCache( clk,
                rst_n,
                i_toggle,
                i_addr[15:2],
                i_wr_data,
                1'b0,
                i_we,
                1'b1,
                i_rd_data,
                i_tag,
                i_hit,
                i_dirty );

  unified_mem memory( clk,
                      rst_n,
                      m_addr,
                      m_re,
                      m_we,
                      m_wr_data,
                      m_rd_data,
                      m_rdy);

  cache_controller controller(.clk(clk),
                              .rst_n(rst_n),
                              .i_rdy(i_rdy),
                              .i_sel(i_sel),
                              .i_wr_data(i_wr_data),
                              .i_we(i_we),
                              .m_addr(m_addr),
                              .m_re(m_re),
                              .m_we(m_we),
                              .m_wr_data(m_wr_data),
                              .i_addr(i_addr),
                              .i_hit(i_hit),
                              .i_tag(i_tag),
                              .m_rd_data(m_rd_data),
                              .m_rdy(m_rdy),
                              .re(re),
                              .we(we),
                              .d_addr(d_addr),
                              .wrt_data(wrt_data));
  
  // mux for instruction output
  always@(*) begin
    if     (i_sel == 2'b00) instr = i_rd_data[15:0];
    else if(i_sel == 2'b01) instr = i_rd_data[31:16];
    else if(i_sel == 2'b10) instr = i_rd_data[47:32];
    else                    instr = i_rd_data[63:48];
  end

  assign i_toggle = 1'b1;
  
endmodule


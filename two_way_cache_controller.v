module two_way_cache_controller(clk, rst_n, i_rdy, i_sel, i_wr_data, i_we, m_addr, m_re, m_we, m_wr_data, i_addr, i_hit, i_tag, m_rd_data, m_rdy, re, we, d_addr, wrt_data, d_wr_data, d_dirty_write, d_we, d_re, d_tag, d_hit, d_dirty_read, d_sel, d_rd_data, d_rdy,allow_hlt, d_toggle);
// cache controller module for 2-way set associative cache
// Remains very similar to direct mapped cache controller but with additional toggle signals
// Manages cache writes and reads in addition to controlling ready signals for rest of processor
// Authors: David Hartman and John Peterson
// Date modified: 12 Dec 2014
  
  input clk, rst_n;
  input [15:0] i_addr; // full imem address
  input i_hit, d_hit, d_dirty_read;
  input [8:0] i_tag, d_tag; // 9-bit tag lines from caches
  input [63:0] m_rd_data, d_rd_data; // read data from memory and dcache
  input m_rdy;
  input re, we;
  input [15:0] d_addr;
  input [15:0] wrt_data;
  
  output reg i_rdy, i_we, m_we, m_re, d_dirty_write, d_we, d_re, d_rdy;
  output reg [63:0] i_wr_data, m_wr_data, d_wr_data;
  output reg [13:0] m_addr;
  output reg [1:0] i_sel, d_sel;
  output reg allow_hlt;
  output reg d_toggle;
  
  reg [2:0] state, nextState; // allows 8 states
  reg [63:0] save_d_rd_data;

  // States //
  localparam IDLE 	       = 3'b000; // wait for next cache/mem handle
  localparam WRITE_DCACHE  = 3'b001; // used for program data write into dcache
  localparam DCACHE_TO_MEM = 3'b010; // used to write out dcache to memory on evict
  localparam MEM_TO_DCACHE = 3'b011; // used to install new mem into dcache
  localparam READ_DCACHE   = 3'b100; // used on dcache hit to read value
  localparam READ_ICACHE   = 3'b101; // used on icache hit to read instruction
  localparam MEM_TO_ICACHE = 3'b110; // used on instr miss to load new imem location into icache

  
  always @(posedge clk, negedge rst_n) begin
	if(!rst_n) begin
      state <= IDLE;
    end else begin
      state <= nextState;
    end
  end
  
  //always@(posedge clk) //$display("state=%b, nextState=%b",state,nextState);
  /* FSM */
  always @ (*) begin
        ////$display("STATE=%b", state);
    // default values
    i_rdy = 1'b0;
    i_sel = i_addr[1:0];
    i_we  = 1'b0;
    i_wr_data = m_rd_data;
    m_we  = 1'b0;
    m_re  = 1'b0;
    m_addr = i_addr[15:2];
    m_wr_data = d_rd_data;
    d_dirty_write = 1'b0;
    d_we = 1'b0;
    d_re = 1'b0; // data read from mem is auto put into instr to write to cache
    d_wr_data = d_rd_data;
    d_sel = d_addr[1:0];
    d_rdy = 1'b0;
	allow_hlt = 1'b0;
    d_toggle = 1'b0;
    
    nextState = IDLE;
    
    case (state)
      IDLE: begin
        //$display("State = IDLE");
        d_re = 1'b1;
        if(we) begin
          //$display("we signal raised");
          nextState=(d_hit)? WRITE_DCACHE : ((d_dirty_read) ? DCACHE_TO_MEM : MEM_TO_DCACHE);
        end else if(re) begin
          //$display("re signal raised, no we");
          nextState=(d_hit)? READ_DCACHE : ((d_dirty_read) ? DCACHE_TO_MEM : MEM_TO_DCACHE);
        end else begin
          //$display("neither we nor re");
          nextState = (i_hit) ? READ_ICACHE : MEM_TO_ICACHE;
        end
      end
      WRITE_DCACHE: begin
        //$display("State = WRITE_DCACHE");
        d_we = 1'b1;
        d_dirty_write = 1'b1;
        d_toggle = 1'b1;
        case (d_addr[1:0])
          2'b00: d_wr_data[15:0] = wrt_data;
          2'b01: d_wr_data[31:16] = wrt_data;
          2'b10: d_wr_data[47:32] = wrt_data;
          2'b11: d_wr_data[63:48] = wrt_data;
          default: begin
            //$display("nothing to see here\n");
          end
        endcase
        d_rdy = 1'b1;
        nextState=(i_hit)? READ_ICACHE: MEM_TO_ICACHE;
      end
      DCACHE_TO_MEM: begin
        //$display("State = DCACHE_TO_MEM");
        m_we = 1'b1;
        d_re = 1'b1;
        m_addr={d_tag,d_addr[7:2]};
        if(!m_rdy) nextState=DCACHE_TO_MEM;
        else begin
          nextState=MEM_TO_DCACHE;
        end
      end
      MEM_TO_DCACHE: begin
        //$display("State = MEM_TO_DCACHE");
        m_re = 1'b1;
        m_addr = d_addr[15:2];
        d_wr_data = m_rd_data;
        if(!m_rdy) nextState=MEM_TO_DCACHE;
        else begin
          d_we = 1'b1;
          nextState=(we)? WRITE_DCACHE: READ_DCACHE;
        end
      end
      READ_DCACHE: begin
        //$display("State = READ_DCACHE");
        d_re = 1'b1;
        d_rdy = 1'b1;
        nextState=(i_hit)? READ_ICACHE: MEM_TO_ICACHE;
      end
      READ_ICACHE: begin
        //$display("State = READ_ICACHE");
        i_rdy = 1'b1;
        nextState = IDLE;
	allow_hlt = 1'b1;
      end
      MEM_TO_ICACHE: begin
        //$display("State = MEM_TO_ICACHE");
        m_re = 1'b1;
        if(!m_rdy) nextState=MEM_TO_ICACHE;
        else begin
          i_we=1'b1;
          nextState=READ_ICACHE;
        end
      end
      default: begin
      end
    endcase
   
  end //end FSM logic 
endmodule

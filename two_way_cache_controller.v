module two_way_cache_controller(clk, rst_n, i_rdy, i_sel, i_wr_data, i_we, m_addr, m_re, m_we, m_wr_data, i_addr, i_hit, i_tag, m_rd_data, m_rdy, re, we, d_addr, wrt_data, d_wr_data, d_dirty_write, d_we, d_re, d_tag, d_hit, d_dirty_read, d_sel, d_rd_data, d_rdy,allow_hlt, d_toggle);
// cache controller module for 2-way set associative cache
// Remains very similar to direct mapped cache controller but with additional toggle signals
// Manages cache writes and reads in addition to controlling ready signals for rest of processor
// Authors: David Hartman and John Peterson
// Date modified: 12 Dec 2014
  
  input clk, rst_n;
  input [15:0] i_addr; // full imem address
  input i_hit, d_hit, d_dirty_read;	// high when hit, dirty, or dirty read from memory
  input [8:0] i_tag, d_tag; // 9-bit tag lines from caches
  input [63:0] m_rd_data, d_rd_data; // read data from memory and dcache
  input m_rdy;	// high when memory is ready to be read
  input re, we;	// enable read or write for memory
  input [15:0] d_addr, wrt_data;	// address and data to write to memory for data
  
  output reg i_rdy, i_we, m_we, m_re, d_dirty_write, d_we, d_re, d_rdy;	// high when enabled
  output reg [63:0] i_wr_data, m_wr_data, d_wr_data;	// data to write to to either cache and memory
  output reg [13:0] m_addr;	// memory address (whether in cache or main memmory)
  output reg [1:0] i_sel, d_sel;	// select for offset of output
  output reg allow_hlt;	// allows a halt to propagate through cpu
  output reg d_toggle;	// toggle to enable reading from either set in the cache
  
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
    i_wr_data = m_rd_data;	// instructon read from memory should always written to i_cache
    m_we  = 1'b0;
    m_re  = 1'b0;
    m_addr = i_addr[15:2];
    m_wr_data = d_rd_data;
    d_dirty_write = 1'b0;
    d_we = 1'b0;
    d_re = 1'b0; // data read from mem is auto put into instr to write to cache
    d_wr_data = d_rd_data;	// data read from memory should always be written to d_cache
    d_sel = d_addr[1:0];
    d_rdy = 1'b0;
	allow_hlt = 1'b0;
    d_toggle = 1'b0;
    
    nextState = IDLE;
    
    case (state)
      IDLE: begin
        // IDLE state: waiting for a request for the i_cache or d_cache
        // allow hlt instruction to propagate through while IDLE
        //$display("State = IDLE");
        // allow a d_cache read in IDLE state. i_re is always set high
        d_re = 1'b1;
        if(we) begin
          // if write enable for d_cache, check if block is a hit or dirty
          //$display("we signal raised");
          nextState=(d_hit)? WRITE_DCACHE : ((d_dirty_read) ? DCACHE_TO_MEM : MEM_TO_DCACHE);
        end else if(re) begin
          // if read enable for d_cache, check if block is hit or dirty
          //$display("re signal raised, no we");
          nextState=(d_hit)? READ_DCACHE : ((d_dirty_read) ? DCACHE_TO_MEM : MEM_TO_DCACHE);
        end else begin
          // if not accessing d_cache, assume an instruction read and check i_cache for hit
          //$display("neither we nor re");
          nextState = (i_hit) ? READ_ICACHE : MEM_TO_ICACHE;
        end
      end
      WRITE_DCACHE: begin
        // WRITE_DCACHE state: write date to d_cache before memory
        // if there is a hit in d_cache, write to that block
        //$display("State = WRITE_DCACHE");
        d_we = 1'b1;
        // set dirty since block will be modified after write
        d_dirty_write = 1'b1;
        // enable toggle to choose between cache sets
        d_toggle = 1'b1;
        // select which offset within block to write to
        case (d_addr[1:0])
          2'b00: d_wr_data[15:0] = wrt_data;
          2'b01: d_wr_data[31:16] = wrt_data;
          2'b10: d_wr_data[47:32] = wrt_data;
          2'b11: d_wr_data[63:48] = wrt_data;
          default: begin
            // default set to prevent latch
            //$display("nothing to see here\n");
          end
        endcase
        // enable read from d_cache before entering next state
        d_rdy = 1'b1;
        // if there is a i_cache hit, read from i_cache. 
        // otherwise default to IDLE
        nextState=(i_hit)? READ_ICACHE: MEM_TO_ICACHE;
      end
      DCACHE_TO_MEM: begin
        // DCACHE_TO_MEM state: write d_cache block back to memory if dirty
        // enable write to memory
        //$display("State = DCACHE_TO_MEM");
        m_we = 1'b1;
        // enable read from d_cache
        d_re = 1'b1;
        // extract memory address from d_tag and index
        m_addr={d_tag,d_addr[7:2]};
        if(!m_rdy) nextState=DCACHE_TO_MEM;
        else begin
          // memory takes 4 cycles to write, stay in state until finished
          // default back to IDLE after write is finished
          nextState=MEM_TO_DCACHE;
        end
      end
      MEM_TO_DCACHE: begin
        // MEM_TO_DCACHE state: read block from memory into d_cache
        // enable memory read
        //$display("State = MEM_TO_DCACHE");
        m_re = 1'b1;
        m_addr = d_addr[15:2];
        // set data read from memory to be written to d_cache
        d_wr_data = m_rd_data;
        if(!m_rdy) nextState=MEM_TO_DCACHE;
        else begin
          // memory takes 4 cycles to read, stay in state until finished
          // default back to IDLE after read is finished
          d_we = 1'b1;
          // choose whether to write to or read from new block in d_cache
          nextState=(we)? WRITE_DCACHE: READ_DCACHE;
        end
      end
      READ_DCACHE: begin
        // READ_DCACHE state: read data block from d_cache
        // set read enable for d_cache
        //$display("State = READ_DCACHE");
        d_re = 1'b1;
        // set data ready for read
        d_rdy = 1'b1;
        // check if we are waiting to read from i_cache
        nextState=(i_hit)? READ_ICACHE: MEM_TO_ICACHE;
      end
      READ_ICACHE: begin
        // READ_ICACHE state: read from i_cache if there is a hit
        //$display("State = READ_ICACHE");
        i_rdy = 1'b1;
        nextState = IDLE;
	allow_hlt = 1'b1;
      end
      MEM_TO_ICACHE: begin
        // MEM_TO_ICACHE state: if instruction miss, read next instr block from memory
        //$display("State = MEM_TO_ICACHE");
        m_re = 1'b1;
        // memory takes 4 cycles to read, stay in state until finished
        // default back to IDLE after read is finished
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

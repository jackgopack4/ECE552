module cache_controller(clk, rst_n, i_rdy, i_sel, i_wr_data, i_we, m_addr, m_re, m_we, m_wr_data, i_addr, i_hit, i_tag, m_rd_data, m_rdy, re, we, d_addr, wrt_data);
  
  input clk, rst_n;
  input [15:0] i_addr;
  input i_hit;
  input [7:0] i_tag;
  input [63:0] m_rd_data;
  input m_rdy;
  input re, we;
  input [15:0] d_addr, wrt_data;
  
  output reg i_rdy, i_we, m_we, m_re;
  output reg [63:0] i_wr_data, m_wr_data;
  output reg [13:0] m_addr;
  output reg [1:0] i_sel;
  
  reg [1:0] state, nextState; // allows 4 states should be enough
  
  // States //
  localparam CACHE_READ 	= 2'b00;
  localparam MEM_READ 	 	= 2'b01;
  localparam WRITE_BACK		= 2'b10;

  
  always @(posedge clk, negedge rst_n) begin
	if(!rst_n) begin
      state <= CACHE_READ;
    end else begin
      state <= nextState;
    end
  end
  
  /* FSM */
  always @ (*) begin
        $display("STATE=%b", state);
    // default values
    i_rdy = 1'b0;
    i_we  = 1'b0;
    m_we  = 1'b0;
    m_re  = 1'b0;
    i_sel = 2'b00;
    nextState = CACHE_READ;
    m_addr = i_addr[15:2];
    i_wr_data = m_rd_data; // data read from mem is auto put into instr to write to cache
    m_wr_data = 64'b0; //switch to d_rd_data in the future
    
    case (state)
    
      CACHE_READ: begin // receive Valid CPU request, transition to compare tag

        // IF A HIT
        if (i_hit) begin
		    $display("icache hit!!");
		      i_sel = i_addr[1:0];
		      i_rdy = 1;
		      nextState = CACHE_READ;
	 
		    // IF NOT A HIT
        end else begin
		    // test output
		    $display("icache miss!!");
	    	// enable mem read
			m_re = 1'b1;
			// enter into mem read for 4 clk cycles
			nextState = MEM_READ;
        end     
      end

      MEM_READ: begin
		// IF MEM IS NOT READY TO BE READ      
        if(!m_rdy) begin
		    // test output
		    $display("still reading mem");
        	nextState = MEM_READ;
        	// mem read must be high until mem is ready
        	m_re = 1'b1;

        // IF MEMORY IS READY TO BE READ
        end else begin
        // test output
        $display("mem finished! change state to cache_read");
        	// enable write to cache
        	i_we = 1'b1;
      		// set state back to cache read
        	nextState = CACHE_READ;
        end 
      end
       
      // currently not using this
      WRITE_BACK: begin
        
        // buffer for cache to write
        nextState = CACHE_READ;
        
      end
      
      default: begin
      end
    endcase
   
  end //end FSM logic
  
  
  
  
  
  
  
  
  
endmodule

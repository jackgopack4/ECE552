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
  
  reg [1:0] state, nextState; // allows 8 states should be enough
  
  // States //
  localparam IDLE 			= 2'b00;
  localparam COMPARE_TAG 	= 2'b01;
  localparam ALLOCATE 		= 2'b10;
  localparam WRITE_BACK 	= 2'b11;
  
  always @(posedge clk, negedge rst_n) begin
	if(!rst_n) begin
      state <= IDLE;
    end else begin
      state <= nextState;
    end
  end
  
  /* FSM */
  always @ (*) begin
    
    // default values
    i_rdy = 1'b0;
    i_we  = 1'b0;
    m_we  = 1'b0;
    m_re  = 1'b0;
    i_sel = 2'b00;
    nextState = IDLE;
    m_addr = i_addr[15:2];
    i_wr_data = m_rd_data;
    m_wr_data = 64'b0; //switch to d_rd_data in the future
    
    case (state)
      IDLE: begin // receive Valid CPU request, transition to compare tag
        
        // do shit here
        if (i_hit) begin
          i_sel = i_addr[1:0];
          i_rdy = 1;
        end else begin
        	// m_addr already at default
			m_re = 1'b1;
			nextState = ALLOCATE;
        end 
          
          
         
        
      end
      
      COMPARE_TAG: begin
        
        // do stuff
        
      end
      
      ALLOCATE: begin
        if(!m_rdy) begin
        	nextState = ALLOCATE;
        	m_re = 1'b1;
        end else begin
        	i_we = 1'b1;
        end
        // do stuff 
        
      end
      
      WRITE_BACK: begin
        
        
      end
      
      default: begin
      end
    endcase
   
  end //end FSM logic
  
  
  
  
  
  
  
  
  
endmodule
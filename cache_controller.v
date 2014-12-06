module cache_controller(clk, rst_n, i_rdy, i_sel, i_wr_data, i_we, m_addr, m_re, m_we, m_wr_data, i_addr, i_hit, i_tag, m_rd_data, m_rdy);
  
  input clk, rst_n;
  input [15:0] i_addr;
  input i_hit;
  input [7:0] i_tag;
  input [63:0] m_rd_data;
  input m_rdy;
  
  output reg i_rdy, i_we, m_we, m_re;
  output reg [63:0] i_wr_data, m_wr_data;
  output reg [13:0] m_addr;
  output reg [1:0] i_sel;
  
  reg [1:0] state, nextState; // allows 8 states should be enough
  reg [63:0] wiped;
  
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
    i_rdy = 0;
    i_we = 0;
    m_we = 0;
    i_sel = 2'b00
    nextState = IDLE;
    m_addr = i_addr[15:2];
    i_wr_data = m_rd_data;
    m_wr_data = 64'b0; //switch to d_rd_data in the future
    
    case (state)
      IDLE: begin
        
        // do shit here
        if (i_hit) begin
          i_sel = i_addr[1:0];
          i_rdy = 1;
        end else if (i_hit && 
          
          
         
        
      end
      
      COMPARE_TAG: begin
        
        // do stuff
        
      end
      
      ALLOCATE: begin
        
        // do stuff 
        
      end
      
      WRITE_BACK: begin
        
        
      end
      
      default: begin
        
        nextStage = IDLE;  
        
      end
   
  end //end FSM logic
  
  
  
  
  
  
  
  
  
endmodule
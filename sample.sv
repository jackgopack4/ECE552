
///////////////////////////////////////
// Create defines for ALU functions //
/////////////////////////////////////
localparam ADD	= 3'b000;
localparam SUB 	= 3'b001;
localparam AND	= 3'b010;
localparam NOR	= 3'b011; 
localparam SLL	= 3'b100;
localparam SRL	= 3'b101;
localparam SRA	= 3'b110;
localparam LHB  = 3'b111;

//////////////////////////////////////////
// Create defines for Opcode encodings //
////////////////////////////////////////
localparam ADDi 	= 4'b0000;
localparam PADDSBi 	= 4'b0001;
localparam SUBi 	= 4'b0010;
localparam ANDi		= 4'b0011;
localparam NORi		= 4'b0100;
localparam SLLi		= 4'b0101;
localparam SRLi		= 4'b0110;
localparam SRAi		= 4'b0111;
localparam LWi		= 4'b1000;
localparam SWi		= 4'b1001;
localparam LHBi		= 4'b1010;
localparam LLBi		= 4'b1011;
localparam BRi		= 4'b1100;
localparam JALi		= 4'b1101;
localparam JRi		= 4'b1110;
localparam HLTi		= 4'b1111;

////////////////////////////////
// Encodings for src0 select //
//////////////////////////////
localparam RF2SRC0 	= 2'b00;
localparam IMM_BR2SRC0 = 2'b01;			// 7-bit SE for branch target
localparam IMM_JMP2SRC0 = 2'b10;		// 12-bit SE for jump target
localparam IMM2SRC0 = 2'b11;			// 4-bit SE Address immediate for LW/SW

////////////////////////////////
// Encodings for src1 select //
//////////////////////////////
localparam RF2SRC1	= 2'b00;
localparam IMM2SRC1 = 2'b01;			// 8-bit data immediate for LLB/LHB
localparam NPC2SRC1 = 2'b10;			// nxt_pc to src1 for JAL instruction



module two_way_cache_controller(clk, rst_n, i_rdy, i_sel, i_wr_data, i_we, m_addr, m_re, m_we, m_wr_data, i_addr, i_hit, i_tag, m_rd_data, m_rdy, re, we, d_addr, wrt_data, d_wr_data, d_dirty_write, d_we, d_re, d_tag, d_hit, d_dirty_read, d_sel, d_rd_data, d_rdy,allow_hlt);
  
  input clk, rst_n;
  input [15:0] i_addr;
  input i_hit, d_hit, d_dirty_read;
  input [8:0] i_tag, d_tag;
  input [63:0] m_rd_data, d_rd_data;
  input m_rdy;
  input re, we;
  input [15:0] d_addr, wrt_data;
  
  output reg i_rdy, i_we, m_we, m_re, d_dirty_write, d_we, d_re, d_rdy;
  output reg [63:0] i_wr_data, m_wr_data, d_wr_data;
  output reg [13:0] m_addr;
  output reg [1:0] i_sel, d_sel;
  output reg allow_hlt;
  
  reg [2:0] state, nextState; // allows 4 states should be enough
  reg [63:0] save_d_rd_data;

  // States //
  localparam IDLE 	       = 3'b000;
  localparam WRITE_DCACHE  = 3'b001;
  localparam DCACHE_TO_MEM = 3'b010;
  localparam MEM_TO_DCACHE = 3'b011;
  localparam READ_DCACHE   = 3'b100;
  localparam READ_ICACHE   = 3'b101;
  localparam MEM_TO_ICACHE = 3'b110;

  
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
    nextState = IDLE;
    
    case (state)
      IDLE: begin
        ////$display("State = IDLE");
        allow_hlt = 1'b1;
        d_re = 1'b1;
        if(we) begin
          ////$display("we signal raised");
          nextState=(d_hit)? WRITE_DCACHE : ((d_dirty_read) ? DCACHE_TO_MEM : MEM_TO_DCACHE);
        end else if(re) begin
          ////$display("re signal raised, no we");
          nextState=(d_hit)? READ_DCACHE : ((d_dirty_read) ? DCACHE_TO_MEM : MEM_TO_DCACHE);
        end else begin
          ////$display("neither we nor re");
          nextState = (i_hit) ? READ_ICACHE : MEM_TO_ICACHE;
        end
      end
      WRITE_DCACHE: begin
        ////$display("State = WRITE_DCACHE");
        d_we = 1'b1;
        d_dirty_write = 1'b1;
        case (d_addr[1:0])
          2'b00: d_wr_data[15:0] = wrt_data;
          2'b01: d_wr_data[31:16] = wrt_data;
          2'b10: d_wr_data[47:32] = wrt_data;
          2'b11: d_wr_data[63:48] = wrt_data;
          default: begin
            ////$display("nothing to see here\n");
          end
        endcase
        d_rdy = 1'b1;
        nextState=(i_hit)? READ_ICACHE: MEM_TO_ICACHE;
      end
      DCACHE_TO_MEM: begin
        ////$display("State = DCACHE_TO_MEM");
        m_we = 1'b1;
        d_re = 1'b1;
        m_addr={d_tag,d_addr[7:2]};
        if(!m_rdy) nextState=DCACHE_TO_MEM;
        else begin
          nextState=MEM_TO_DCACHE;
        end
      end
      MEM_TO_DCACHE: begin
        ////$display("State = MEM_TO_DCACHE");
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
        ////$display("State = READ_DCACHE");
        d_re = 1'b1;
        d_rdy = 1'b1;
        nextState=(i_hit)? READ_ICACHE: MEM_TO_ICACHE;
      end
      READ_ICACHE: begin
        ////$display("State = READ_ICACHE");
        i_rdy = 1'b1;
        nextState = IDLE;
      end
      MEM_TO_ICACHE: begin
        ////$display("State = MEM_TO_ICACHE");
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
module two_way_cache(clk,rst_n,toggle,addr,wr_data,wdirty,we,re,rd_data,tag_out,hit,dirty);

input clk,rst_n;
input [13:0] addr;		// address to be read or written, 2-LSB's are dropped
input [63:0] wr_data;	// 64-bit cache line to write
input toggle;     // set in order to update LRU on any read/write
input wdirty;			// dirty bit to be written
input we;				// write enable for cache line
input re;				// read enable (for power purposes only)

output hit;
output dirty;
output [63:0] rd_data;	// 64-bit cache line read out
output [8:0] tag_out;	// 8-bit tag.  This is needed during evictions

reg [150:0]mem[0:31];	// {LRU,valid1,dirty1,tag1[8:0],wdata1[63:0],
                      //  valid0,dirty0,tag0[8:0],wdata0[63:0]}
reg [6:0] x;
reg [150:0] line;
reg we_del;
reg match_hi, match_lo;

wire we_filt;

//////////////////////////
// Glitch filter on we //
////////////////////////
always @(we)
  we_del <= we;

assign we_filt = we & we_del;

///////////////////////////////////////////////////////
// Model cache write, including reset of valid bits //
/////////////////////////////////////////////////////

always @(clk or we_filt or negedge rst_n)
  if (!rst_n)
    for (x=0; x<32;  x = x + 1)
	  mem[x] = {1'bx,2'b00,{73{1'bx}},2'b00,{73{1'bx}}};		// only valid & dirty bit are cleared, all others are x
  else if (~clk && we_filt) begin
    if(mem[addr[4:0][147]] == 1'b1) begin
      mem[addr[4:0]] = {(toggle)?1'b0:1'bx,1'b1,wdirty,addr[13:5],wr_data,{75{1'bx}}};
    end else begin
      mem[addr[4:0]] = {(toggle)?1'b1:1'bx,{75{1'bx}},1'b1,wdirty,addr[13:5],wr_data};
    end
  end

////////////////////////////////////////////////////////////
// Model cache read including 4:1 muxing of 16-bit words //
//////////////////////////////////////////////////////////
always @(clk or re or addr) begin
  if (clk && re)        // read is on clock high
    line = mem[addr[4:0]];
  ////$display("match_hi=%b\nmatch_lo=%b\nhit=%b\nline[74:0]=%h\nLRU=%b\ntag=%b\nline[147:139]=%b\naddr[13:5]=%b\nline[72:64]=%b\n",match_hi,match_lo,hit,line[74:0],line[150],tag_out,line[147:139],addr[13:5],line[72:64]);  
end

	
/////////////////////////////////////////////////////////////
// If tag bits match and line is valid then we have a hit //
///////////////////////////////////////////////////////////
always@(*) begin
    match_hi = 1'b0;
    match_lo = 1'b0;
  if(line[147:139]==addr[13:5]) match_hi = 1'b1;
  else if(line[72:64]==addr[13:5]) match_lo = 1'b1;
end
assign hit = (((match_hi==1'b1) && (re | we)) ? line[149]:
             (((match_lo==1'b1) && (re | we))   ? line[74] :
                                                           1'b0));
assign dirty   = (match_hi==1'b1)? line[149]&line[148] : line[74]&line[73];						// if line is valid and dirty bit set
assign rd_data = (match_hi==1'b1)? line[138:75] : line[63:0];
assign tag_out = (match_hi==1'b1)? line[147:139] : line[72:64];							// need the tag for evictions
	
endmodule
module two_way_mem_hierarchy(clk, rst_n, instr, i_rdy, d_rdy, rd_data, i_addr, d_addr, re, we, wrt_data, allow_hlt);

  input clk, rst_n, re, we;
  input [15:0] i_addr, d_addr, wrt_data;

  output i_rdy, d_rdy;
  output reg [15:0] instr, rd_data;
  output allow_hlt;
  
  wire [63:0] i_rd_data, i_wr_data;
  wire [8:0] i_tag;
  wire i_hit, i_dirty, i_we, i_toggle;
  wire [1:0] i_sel;

  wire [63:0] d_rd_data, d_wr_data;
  wire [8:0] d_tag;
  wire d_hit, d_dirty, d_we, d_dirty_in, d_re, d_toggle;
  wire [1:0] d_sel;

  wire [13:0] m_addr;
  wire m_re, m_we, m_rdy;
  wire [63:0] m_wr_data, m_rd_data;
  
	two_way_cache iCache( clk,
                rst_n,
                i_toggle,
                i_addr[15:2],
                i_wr_data, //m_rd_data,	
                1'b0,		// dirty bit set to 0
                i_we,		// enable write to cache
                1'b1,		// read enable set high
                i_rd_data,	// i_cache read out data is full block
                i_tag,		// tag of instr
                i_hit,		// high if hit
                i_dirty );	// high if dirty

  two_way_cache dCache( clk,
                rst_n,
                d_toggle,
                d_addr[15:2],
                d_wr_data,
                d_dirty_in,
                d_we,
                d_re,
                d_rd_data,
                d_tag,
                d_hit,
                d_dirty);
	
	two_way_cache_controller controller(.clk(clk),
                              .rst_n(rst_n),
                              .i_rdy(i_rdy),	// high when instr is ready to be read
                              .i_sel(i_sel),	// sel for instr output
                              .i_wr_data(i_wr_data),	// instr to write to cache
                              .i_we(i_we),		// enable write to cache
                              .m_addr(m_addr),	// mem addr (whether in cache or main mem)
                              .m_re(m_re),		// mem read enable
                              .m_we(m_we),		// mem write enable
                              .m_wr_data(m_wr_data),	// mem to write to cache
                              .i_addr(i_addr),	// taking full i_addr into conroller
                              .i_hit(i_hit),	// instr hit
                              .i_tag(i_tag),	// tag
                              .m_rd_data(m_rd_data),	// data read from mem
                              .m_rdy(m_rdy),	// mem ready? for hwat?
                              .re(re),			// read enable? for hwat?
                              .we(we),			// write enable?
                              .d_addr(d_addr),
                              .wrt_data(wrt_data),
                              .d_wr_data(d_wr_data),
                              .d_dirty_write(d_dirty_in),
                              .d_we(d_we),
                              .d_re(d_re),
                              .d_tag(d_tag),
                              .d_hit(d_hit),
                              .d_dirty_read(d_dirty),
                              .d_sel(d_sel),
                              .d_rd_data(d_rd_data),
                              .d_rdy(d_rdy),
                              .allow_hlt(allow_hlt));
	
	unified_mem memory( clk,
                      rst_n,
                      m_addr,	// addr in mem
                      m_re,		// read enable
                      m_we,		// write enable
                      m_wr_data,	// data to write to mem
                      m_rd_data,	// data read from mem
                      m_rdy);		// mem has finished reading/writing


  
  // mux for instruction output
  always@(negedge clk) begin
    if     (i_sel == 2'b00) instr = i_rd_data[15:0];
    else if(i_sel == 2'b01) instr = i_rd_data[31:16];
    else if(i_sel == 2'b10) instr = i_rd_data[47:32];
    else                    instr = i_rd_data[63:48];
  end
  assign i_toggle = 1'b1;
  assign d_toggle = (d_re | d_we);
  always@(posedge clk) begin
    /*//$display("i_addr=%b\ni_wr_data=%b\ni_we=%b\ni_rd_data=%h\ni_tag=%b\ni_hit=%b\ni_dirty=%b\ninstr=%h\ni_sel=%b\n",
                i_addr[15:2],
                i_wr_data, //m_rd_data, 
                i_we,   // enable write to cache
                i_rd_data,  // i_cache read out data is full block
                i_tag,    // tag of instr
                i_hit,    // high if hit
                i_dirty,
                instr,
                i_sel); 
    */
    /*//$display("d_addr=%b, d_wr_data=%h, d_dirt_in=%b, d_we=%b, d_re=%b\nd_rd_data=%h\nd_tag=%b, d_hit=%b, d_dirty=%b",
                d_addr[15:2],
                d_wr_data,
                d_dirty_in,
                d_we,
                d_re,
                d_rd_data,
                d_tag,
                d_hit,
                d_dirty); */
    /*//$display( "m_addr=%b, m_re=%b, m_we=%b\nm_wr_data=%h\nm_rd_data=%h\nm_rdy=%b",
              m_addr, // addr in mem
              m_re,   // read enable
              m_we,   // write enable
              m_wr_data,  // data to write to mem
              m_rd_data,  // data read from mem
              m_rdy); */
  end
  // mux for data read output
  always@(negedge clk) begin
    if     (d_sel == 2'b00) rd_data = d_rd_data[15:0];
    else if(d_sel == 2'b01) rd_data = d_rd_data[31:16];
    else if(d_sel == 2'b10) rd_data = d_rd_data[47:32];
    else                    rd_data = d_rd_data[63:48];
  end

/*
// test output //
always @(*) begin
		//$display("m_rdy=%b, i_rdy=%b, i_hit=%b, i_sel=%b, instr=%h,\n m_rd_data=%h, i_rd_data=%h, i_we=%b,\n i_wr_data=%h, m_re=%b, i_addr=%h\n", m_rdy, i_rdy, i_hit, i_sel, instr, m_rd_data, i_rd_data, i_we, i_wr_data, m_re, i_addr);
		////$display("instr=%h\n",instr);
	end  
*/
  
  
endmodule
module alu(clk,src0,src1,shamt,func,dst,dst_EX_DM,ov,zr,neg, padd,stall_EX_DM);
///////////////////////////////////////////////////////////
// ALU.  Performs ADD, SUB, AND, NOR, SLL, SRL, or SRA  //
// based on func input.  Provides OV and ZR outputs.   //
// Arithmetic is saturating.                          //
///////////////////////////////////////////////////////
// Encoding of func[2:0] is as follows: //
// 000 ==> ADD
// 001 ==> SUB
// 010 ==> AND
// 011 ==> NOR
// 100 ==> SLL
// 101 ==> SRL
// 110 ==> SRA
// 111 ==> reserved

//////////////////////
// include defines //
////////////////////
`include "common_params.inc"

input clk;
input [15:0] src0,src1;
input [2:0] func;			// selects function to perform
input [3:0] shamt;			// shift amount
input stall_EX_DM;
input padd;


output [15:0] dst;			// ID_EX version for branch/jump targets
output reg [15:0] dst_EX_DM;
output ov,zr,neg;

wire [15:0] sum;		// output of adder
wire [15:0] sum_sat;	// saturated sum
wire [15:0] src0_2s_cmp;
wire cin;
wire [15:0] shft_l1,shft_l2,shft_l4,shft_l;		// intermediates for shift left
wire [15:0] shft_r1,shft_r2,shft_r4,shft_r;		// intermediates for shift right

/////////////////////////////////////////////////
// Implement 2s complement logic for subtract //
///////////////////////////////////////////////
assign src0_2s_cmp = (func==SUB) ? ~src0 : src0;	// use 2's comp for sub
assign cin = (func==SUB) ? 1 : 0;					// which is invert and add 1
///////////////////////////////
// Now for PADDSB logic //
/////////////////////////////
wire [15:0] sum_padd;
wire [7:0] left_sum, right_sum, left_sum_sat, right_sum_sat;
wire left_sat_neg, left_sat_pos, right_sat_neg, right_sat_pos;
wire cin_right_to_left, cin_real_right_to_left; 

assign {cin_right_to_left, right_sum} = src0_2s_cmp[7:0] + src1[7:0] + cin;
assign cin_real_right_to_left = padd? 0 : cin_right_to_left;
assign left_sum = src0_2s_cmp[15:8] + src1[15:8] + cin_real_right_to_left;

assign left_sat_neg = (~left_sum[7] && src0_2s_cmp[15] && src1[15]) ? 1 : 0;
assign left_sat_pos = (left_sum[7] && ~src0_2s_cmp[15] && ~src1[15]) ? 1 : 0;
assign right_sat_neg = (~right_sum[7] && src0_2s_cmp[7] && src1[7]) ? 1 : 0;
assign right_sat_pos = (right_sum[7] && ~src0_2s_cmp[7] && ~src1[7]) ? 1 : 0;

assign left_sum_sat = (left_sat_pos) ? 8'h7f:
                      (left_sat_neg) ? 8'h80: left_sum;
assign right_sum_sat = (right_sat_pos) ? 8'h7f:
                       (right_sat_neg) ? 8'h80: right_sum;

assign sum_padd = {left_sum_sat, right_sum_sat};


//////////////////////
// Implement adder //
////////////////////
assign sum = {left_sum, right_sum};

///////////////////////////////
// Now for saturation logic //
/////////////////////////////
assign sat_neg = (src1[15] && src0_2s_cmp[15] && ~sum[15]) ? 1 : 0;
assign sat_pos = (~src1[15] && !src0_2s_cmp[15] && sum[15]) ? 1 : 0;
assign sum_sat = (sat_pos) ? 16'h7fff :
                 (sat_neg) ? 16'h8000 :
				 sum;

assign ov = sat_pos | sat_neg;

				 
///////////////////////////
// Now for left shifter //
/////////////////////////
assign shft_l1 = (shamt[0]) ? {src1[14:0],1'b0} : src1;
assign shft_l2 = (shamt[1]) ? {shft_l1[13:0],2'b00} : shft_l1;
assign shft_l4 = (shamt[2]) ? {shft_l2[11:0],4'h0} : shft_l2;
assign shft_l = (shamt[3]) ? {shft_l4[7:0],8'h00} : shft_l4;

////////////////////////////
// Now for right shifter //
//////////////////////////
assign shft_in = (func==SRA) ? src1[15] : 0;
assign shft_r1 = (shamt[0]) ? {shft_in,src1[15:1]} : src1;
assign shft_r2 = (shamt[1]) ? {{2{shft_in}},shft_r1[15:2]} : shft_r1;
assign shft_r4 = (shamt[2]) ? {{4{shft_in}},shft_r2[15:4]} : shft_r2;
assign shft_r = (shamt[3]) ? {{8{shft_in}},shft_r4[15:8]} : shft_r4;

///////////////////////////////////////////
// Now for multiplexing function of ALU //
/////////////////////////////////////////
assign dst = (padd)? sum_padd:
	     (func==AND) ? src1 & src0 :
	     (func==NOR) ? ~(src1 | src0) :
	     (func==SLL) ? shft_l :
	     ((func==SRL) || (func==SRA)) ? shft_r :
	     (func==LHB) ? {src1[7:0],src0[7:0]} : sum_sat;	 
			 
assign zr = ~|dst;
assign neg = dst[15];

//////////////////////////
// Flop the ALU result //
////////////////////////
always @(posedge clk)
  if (!stall_EX_DM) begin
  	dst_EX_DM <= dst;
  end
endmodule
module br_bool(clk,rst_n,clk_z_ID_EX,clk_nv_ID_EX,br_instr_ID_EX,
               jmp_imm_ID_EX,jmp_reg_ID_EX,cc_ID_EX,zr,ov,neg,
			   flow_change_ID_EX,stall_EX_DM,stall_ID_EX,zr_EX_DM);

//////////////////////////////////////////////////////
// determines branch or not based on cc, and flags //
////////////////////////////////////////////////////
input clk,rst_n;
input clk_z_ID_EX;			// from ID, tells us to flop the zero flag
input clk_nv_ID_EX;			// from ID, tells us to flop the overflow flag
input br_instr_ID_EX;		// from ID, tell us if this is a branch instruction
input jmp_imm_ID_EX;		// from ID, tell us this is jump immediate instruction
input jmp_reg_ID_EX;		// from ID, tell us this is jump register instruction
input [2:0] cc_ID_EX;		// condition code from instr[11:9]
input zr,ov,neg;			// flag bits from ALU
input stall_EX_DM,stall_ID_EX;

output reg flow_change_ID_EX;		// asserted if we should take branch or jumping
output reg zr_EX_DM;				// goes to ID for ADDZ

reg neg_EX_DM,ov_EX_DM;

/////////////////////////
// Flop for zero flag //
///////////////////////
always @(posedge clk, negedge rst_n)
  if (!rst_n)
    zr_EX_DM  <= 0;
  else if (clk_z_ID_EX && !stall_EX_DM) 
    zr_EX_DM  <= zr;

	
/////////////////////////////////////
// Flops for negative and ov flag //
///////////////////////////////////
always @(posedge clk, negedge rst_n)
  if (!rst_n)
    begin
      ov_EX_DM  <= 0;
	  neg_EX_DM <= 0;
	end
  else if (clk_nv_ID_EX && !stall_EX_DM)
    begin
      ov_EX_DM  <= ov;
	  neg_EX_DM <= neg;
	end

//if (!stall_ID_EX) begin
	always @(br_instr_ID_EX,cc_ID_EX,zr_EX_DM,ov_EX_DM,neg_EX_DM,jmp_reg_ID_EX,jmp_imm_ID_EX) begin

	  flow_change_ID_EX = jmp_imm_ID_EX | jmp_reg_ID_EX;	// jumps always change the flow
	  
	  if (br_instr_ID_EX)
		case (cc_ID_EX)
		  3'b000 : flow_change_ID_EX = ~zr_EX_DM;
		  3'b001 : flow_change_ID_EX = zr_EX_DM;
		  3'b010 : flow_change_ID_EX = ~zr_EX_DM & ~neg_EX_DM;
		  3'b011 : flow_change_ID_EX = neg_EX_DM;
		  3'b100 : flow_change_ID_EX = zr_EX_DM | (~zr_EX_DM & ~neg_EX_DM);
		  3'b101 : flow_change_ID_EX = neg_EX_DM | zr_EX_DM;
		  3'b110 : flow_change_ID_EX = ov_EX_DM;
		  3'b111 : flow_change_ID_EX = 1;
		endcase
	end
//end
endmodule
module cpu(clk,rst_n, pc, hlt);

input clk,rst_n;
output [15:0] pc;
output hlt;

wire [15:0] instr;			// instruction from IM
wire [11:0] instr_ID_EX;		// immediate bus
wire [15:0] src0,src1;			// operand busses into ALU
wire [15:0] dst_EX_DM;			// result from ALU
wire [15:0] dst_ID_EX;			// result from ALU for branch destination
wire [15:0] pc_ID_EX;			// nxt_pc to source mux for JR
wire [15:0] pc_EX_DM;			// nxt_pc to store in reg15 for JAL
wire [15:0] iaddr;			// instruction address
wire [15:0] dm_rd_data_EX_DM;		// data memory read data
wire [15:0] rf_w_data_DM_WB;		// register file write data
wire [15:0] p0,p1;			// read ports from RF
wire [3:0] rf_p0_addr;			// address for port 0 reads
wire [3:0] rf_p1_addr;			// address for port 1 reads
wire [3:0] rf_dst_addr_DM_WB;		// address for RF write port
wire [2:0] alu_func_ID_EX;		// specifies operation ALU should perform
wire [1:0] src0sel_ID_EX;		// select for src0 bus
wire [1:0] src1sel_ID_EX;		// select for src1 bus
wire [2:0] cc_ID_EX;			// condition code pipeline from instr[11:9]
wire [15:0] p0_EX_DM;			// data to be stored for SW

wire allow_hlt;

// memory //
wire i_rdy, d_rdy, re, we, stall_pc, stall_IM_ID;
wire [15:0] rd_data, wrt_data, d_addr;

/*
// test output //
 always @(posedge clk) begin
		if(i_rdy) begin
			//$display("iCache valid, instr=%h",instr);
		end else begin
			////$display("iCache invalid, reading from mem");
			//$display("instr=%h\n",instr);
		end
		
		//$display("stall_pc=%b, i_rdy=%b, stall_IM_ID=%b, iaddr=%h, flow_change_ID_EX=%h,zr=%b,rf_w_data_DM_WB=%h, rf_dst_addr_DM_WB=%h\n",stall_pc, i_rdy, stall_IM_ID, iaddr, flow_change_ID_EX,zr,rf_w_data_DM_WB,rf_dst_addr_DM_WB);
	end
*/

///////////////////////////////////
// Instantiate memory hierarchy //
/////////////////////////////////
2way_mem_hierarchy mem_h(.clk(clk), .rst_n(rst_n), .instr(instr), .i_rdy(i_rdy), .d_rdy(d_rdy), .rd_data(dm_rd_data_EX_DM), .i_addr(iaddr), .d_addr(dst_EX_DM), .re(dm_re_EX_DM), .we(dm_we_EX_DM), .wrt_data(p0_EX_DM), .allow_hlt(allow_hlt));



//////////////////////////////////
// Instantiate program counter //
////////////////////////////////
pc iPC(.clk(clk), .rst_n(rst_n), .stall_IM_ID(stall_IM_ID), .pc(iaddr), .dst_ID_EX(dst_ID_EX),
       .pc_ID_EX(pc_ID_EX), .pc_EX_DM(pc_EX_DM), .flow_change_ID_EX(flow_change_ID_EX), .i_rdy(i_rdy),
       .stall_ID_EX(stall_ID_EX), .stall_EX_DM(stall_EX_DM));

// assign when to stall pc
//assign stall_pc = stall_IM_ID ? stall_IM_ID : !i_rdy;
	   
/////////////////////////////////////
// Instantiate instruction memory //
///////////////////////////////////
//IM iIM(.clk(clk), .addr(iaddr), .rd_en(1'b1), .instr(instr));

//////////////////////////////
// Instantiate data memory //
////////////////////////////
//DM iDM(.clk(clk),.addr(dst_EX_DM), .re(dm_re_EX_DM), .we(dm_we_EX_DM), .wrt_data(p0_EX_DM),
//       .rd_data(dm_rd_data_EX_DM));

//////////////////////////////////////////////
// Instantiate register instruction decode //
////////////////////////////////////////////
id	iID(.clk(clk), .rst_n(rst_n), .instr(instr), /*.zr_EX_DM(zr_EX_DM),*/ .br_instr_ID_EX(br_instr_ID_EX),
        .jmp_imm_ID_EX(jmp_imm_ID_EX), .jmp_reg_ID_EX(jmp_reg_ID_EX), .jmp_imm_EX_DM(jmp_imm_EX_DM), .rf_re0(rf_re0),
		.rf_re1(rf_re1), .rf_we_DM_WB(rf_we_DM_WB), .rf_p0_addr(rf_p0_addr), .rf_p1_addr(rf_p1_addr),
		.rf_dst_addr_DM_WB(rf_dst_addr_DM_WB), .alu_func_ID_EX(alu_func_ID_EX),
		.src0sel_ID_EX(src0sel_ID_EX), .src1sel_ID_EX(src1sel_ID_EX), .dm_re_EX_DM(dm_re_EX_DM),
		.dm_we_EX_DM(dm_we_EX_DM), .clk_z_ID_EX(clk_z_ID_EX), .clk_nv_ID_EX(clk_nv_ID_EX),
		.instr_ID_EX(instr_ID_EX), .cc_ID_EX(cc_ID_EX), .stall_IM_ID(stall_IM_ID),
		.stall_ID_EX(stall_ID_EX), .stall_EX_DM(stall_EX_DM), .hlt_DM_WB(hlt_DM_WB),
		.byp0_EX(byp0_EX), .byp0_DM(byp0_DM), .byp1_EX(byp1_EX), .byp1_DM(byp1_DM),
		.flow_change_ID_EX(flow_change_ID_EX),
		.padd_ID_EX(padd_ID_EX), .i_rdy(i_rdy),.stall_DM_WB(stall_DM_WB));
	   
////////////////////////////////
// Instantiate register file //
//////////////////////////////
rf iRF(.clk(clk), .p0_addr(rf_p0_addr), .p1_addr(rf_p1_addr), .p0(p0), .p1(p1),
       .re0(rf_re0), .re1(rf_re1), .dst_addr(rf_dst_addr_DM_WB), .dst(rf_w_data_DM_WB),
 	   .we(rf_we_DM_WB), .hlt(hlt_DM_WB));
	   
///////////////////////////////////
// Instantiate register src mux //
/////////////////////////////////
src_mux ISRCMUX(.clk(clk), .stall_ID_EX(stall_ID_EX), .stall_EX_DM(stall_EX_DM),
                .src0sel_ID_EX(src0sel_ID_EX), .src1sel_ID_EX(src1sel_ID_EX), .p0(p0), .p1(p1),
                .imm_ID_EX(instr_ID_EX), .pc_ID_EX(pc_ID_EX), .p0_EX_DM(p0_EX_DM),
				.src0(src0), .src1(src1), .dst_EX_DM(dst_EX_DM), .dst_DM_WB(rf_w_data_DM_WB),
			    .byp0_EX(byp0_EX), .byp0_DM(byp0_DM), .byp1_EX(byp1_EX), .byp1_DM(byp1_DM));
			    
// assign when to stall srcMux
//assign stall_srcMux1 = stall_ID_EX ? stall_ID_EX : !i_rdy;
//assign stall_srcMux2 = stall_EX_DM ? stall_EX_DM : !i_rdy;

	   
//////////////////////
// Instantiate ALU //
////////////////////
alu iALU(.clk(clk), .src0(src0), .src1(src1), .shamt(instr_ID_EX[3:0]), .func(alu_func_ID_EX),
         .dst(dst_ID_EX), .dst_EX_DM(dst_EX_DM), .ov(ov), .zr(zr), .neg(neg),
	 .padd(padd_ID_EX),.stall_EX_DM(stall_EX_DM));		   


//////////////////////////
// Instantiate dst mux //
////////////////////////
dst_mux iDSTMUX(.clk(clk), .dm_re_EX_DM(dm_re_EX_DM), .dm_rd_data_EX_DM(dm_rd_data_EX_DM),
                .dst_EX_DM(dst_EX_DM), .pc_EX_DM(pc_EX_DM), .rf_w_data_DM_WB(rf_w_data_DM_WB),
				.jmp_imm_EX_DM(jmp_imm_EX_DM),.stall_DM_WB(stall_DM_WB));
	
/////////////////////////////////////////////
// Instantiate branch determination logic //
///////////////////////////////////////////
br_bool iBRL(.clk(clk), .rst_n(rst_n), .clk_z_ID_EX(clk_z_ID_EX), .clk_nv_ID_EX(clk_nv_ID_EX),
             .br_instr_ID_EX(br_instr_ID_EX), .jmp_imm_ID_EX(jmp_imm_ID_EX),
	     .jmp_reg_ID_EX(jmp_reg_ID_EX), .cc_ID_EX(cc_ID_EX), .zr(zr), .ov(ov),
           /*.zr_EX_DM(zr_EX_DM),*/ .neg(neg), .flow_change_ID_EX(flow_change_ID_EX),.stall_EX_DM(stall_EX_DM),
           .stall_ID_EX(stall_ID_EX));	
	



assign pc = iaddr;
assign hlt = hlt_DM_WB && allow_hlt; // waits until last Dcache processed.
   
endmodule
module DM(clk,addr,re,we,wrt_data,rd_data);

/////////////////////////////////////////////////////////
// Data memory.  Single ported, can read or write but //
// not both in a single cycle.  Precharge on clock   //
// high, read/write on clock low.                   //
/////////////////////////////////////////////////////
input clk;
input [15:0] addr;
input re;				// asserted when instruction read desired
input we;				// asserted when write desired
input [15:0] wrt_data;	// data to be written

output reg [15:0] rd_data;	//output of data memory

reg [15:0]data_mem[0:65535];

///////////////////////////////////////////////
// Model read, data is latched on clock low //
/////////////////////////////////////////////
always @(addr,re,clk)
  if (~clk && re && ~we)
    rd_data <= data_mem[addr];
	
////////////////////////////////////////////////
// Model write, data is written on clock low //
//////////////////////////////////////////////
always @(addr,we,clk)
  if (~clk && we && ~re)
    data_mem[addr] <= wrt_data;

endmodule
module dst_mux(clk,dm_re_EX_DM,dm_rd_data_EX_DM,pc_EX_DM,dst_EX_DM,rf_w_data_DM_WB,jmp_imm_EX_DM,stall_DM_WB);
////////////////////////////////////////////////////////////////////////
// Simple 2:1 mux determining if ALU or DM is source for write to RF //
//////////////////////////////////////////////////////////////////////
input clk;
input dm_re_EX_DM;
input jmp_imm_EX_DM;
input [15:0] dm_rd_data_EX_DM;		// input from DM
input [15:0] pc_EX_DM;				// from PC for JAL saving to R15
input [15:0] dst_EX_DM;				// input from ALU
input stall_DM_WB;

output reg[15:0] rf_w_data_DM_WB;		// output to be written to RF

always @(posedge clk)
  if (!stall_DM_WB) begin
    if (dm_re_EX_DM)
      rf_w_data_DM_WB <= dm_rd_data_EX_DM;
    else if (jmp_imm_EX_DM)
      rf_w_data_DM_WB <= pc_EX_DM;
    else
      rf_w_data_DM_WB <= dst_EX_DM;
  end
endmodule
module id(clk,rst_n,instr,/*zr_EX_DM,*/br_instr_ID_EX,jmp_imm_ID_EX,jmp_reg_ID_EX,
jmp_imm_EX_DM,rf_re0,rf_re1,
rf_we_DM_WB,rf_p0_addr,rf_p1_addr,rf_dst_addr_DM_WB,alu_func_ID_EX,src0sel_ID_EX,
src1sel_ID_EX,dm_re_EX_DM,dm_we_EX_DM,clk_z_ID_EX,clk_nv_ID_EX,instr_ID_EX,
cc_ID_EX, stall_IM_ID,stall_ID_EX,stall_EX_DM,hlt_DM_WB,byp0_EX,byp0_DM,
byp1_EX,byp1_DM,flow_change_ID_EX,
padd_ID_EX,i_rdy,stall_DM_WB);
input clk,rst_n, i_rdy;
input [15:0] instr; // instruction to decode and execute direct from IM, flop first
//input zr_EX_DM; // zero flag from ALU (used for ADDZ)
input flow_change_ID_EX;
output reg jmp_imm_ID_EX;
output reg jmp_reg_ID_EX;
output reg br_instr_ID_EX; // set if instruction is branch instruction
output reg jmp_imm_EX_DM; // needed for JAL in dst_mux
output reg rf_re0; // asserted if instruction needs to read operand 0 from RF
output reg rf_re1; // asserted if instruction needs to read operand 1 from RF
output reg rf_we_DM_WB; // set if instruction is writing back to RF
output reg [3:0] rf_p0_addr; // normally instr[3:0] but for LHB and SW it is instr[11:8]
output reg [3:0] rf_p1_addr; // normally instr[7:4]
output reg [3:0] rf_dst_addr_DM_WB; // normally instr[11:8] but for JAL it is forced to 15
output reg [2:0] alu_func_ID_EX; // select ALU operation to be performed
output reg [1:0] src0sel_ID_EX; // select source for src0 bus
output reg [1:0] src1sel_ID_EX; // select source for src1 bus
output reg dm_re_EX_DM; // asserted on loads
output reg dm_we_EX_DM; // asserted on stores
output reg clk_z_ID_EX; // asserted for instructions that should modify zero flag
output reg clk_nv_ID_EX; // asserted for instructions that should modify negative and ov flags
output [11:0] instr_ID_EX; // lower 12-bits needed for immediate based instructions
output [2:0] cc_ID_EX; // condition code bits for branch determination from instr[11:9]
output stall_IM_ID; // asserted for hazards and halt instruction, stalls IM_ID flops
output stall_ID_EX; //reg // asserted for hazards and halt instruction, stalls ID_EX flops
output stall_EX_DM; //reg // asserted for hazards and halt instruction, stalls EX_DM flops
output stall_DM_WB;
output reg hlt_DM_WB; // needed for register dump
output reg byp0_EX,byp0_DM; // bypasing controls for RF_p0
output reg byp1_EX,byp1_DM; // bypassing controls for RF_p1
output reg padd_ID_EX;
reg padd;
////////////////////////////////////////////////////////////////
// Register type needed for assignment in combinational case //
//////////////////////////////////////////////////////////////
reg br_instr;
reg jmp_imm;
reg jmp_reg;
reg rf_we;
reg hlt;
reg [3:0] rf_dst_addr;
reg [2:0] alu_func;
reg [1:0] src0sel,src1sel;
reg dm_re;
reg dm_we;
reg clk_z;
reg clk_nv;
//reg cond_ex;
/////////////////////////////////
// Registers needed for flops //
///////////////////////////////
reg [15:0] instr_IM_ID; // flop capturing the instruction to be decoded
reg	rf_we_ID_EX,rf_we_EX_DM;
reg [3:0] rf_dst_addr_ID_EX,rf_dst_addr_EX_DM;
reg	dm_re_ID_EX;
reg	dm_we_ID_EX;
reg hlt_ID_EX,hlt_EX_DM;
reg [11:0] instr_ID_EX; // only need lower 12-bits for immediate values
reg flow_change_EX_DM; // needed to pipeline flow_change_ID_EX
//reg cond_ex_ID_EX; // needed for ADDZ knock down of rf_we
wire load_use_hazard,flush;
/////////////////////
// include params //
///////////////////
`include "common_params.inc"
///////////////////////////////////
// Flop the instruction from IM //
/////////////////////////////////
always @(posedge clk, negedge rst_n)
if (!rst_n)
instr_IM_ID <= 16'hb000; // LLB R0, #0000
else if (!stall_IM_ID)// || (flow_change_ID_EX && !i_rdy))
instr_IM_ID <= instr; // flop raw instruction from IM
/////////////////////////////////////////////////////////////
// Pipeline control signals needed in EX stage and beyond //
///////////////////////////////////////////////////////////
always @(posedge clk)
if (!stall_ID_EX)
begin
br_instr_ID_EX <= br_instr & !flush;
jmp_imm_ID_EX <= jmp_imm & !flush;
jmp_reg_ID_EX <= jmp_reg & !flush;
rf_we_ID_EX <= rf_we & !load_use_hazard & !flush;
rf_dst_addr_ID_EX <= rf_dst_addr;
alu_func_ID_EX <= alu_func;
src0sel_ID_EX <= src0sel;
src1sel_ID_EX <= src1sel;
dm_re_ID_EX <= dm_re;
dm_we_ID_EX <= dm_we & !load_use_hazard & !flush;
clk_z_ID_EX <= clk_z & !load_use_hazard & !flush;
clk_nv_ID_EX <= clk_nv & !load_use_hazard & !flush;
instr_ID_EX <= instr_IM_ID[11:0];
//cond_ex_ID_EX <= cond_ex;
padd_ID_EX <= padd;
end
//////////////////////////////////////////////////////////////
// Pipeline control signals needed in MEM stage and beyond //
////////////////////////////////////////////////////////////
always @(posedge clk)
if (!stall_EX_DM)
begin
rf_we_EX_DM <= rf_we_ID_EX;//rf_we_ID_EX & (!(cond_ex_ID_EX & !zr_EX_DM)); // ADDZ
rf_dst_addr_EX_DM <= rf_dst_addr_ID_EX;
dm_re_EX_DM <= dm_re_ID_EX;
dm_we_EX_DM <= dm_we_ID_EX;
jmp_imm_EX_DM <= jmp_imm_ID_EX;
end
always @(posedge clk) begin
if (!stall_DM_WB) begin
rf_we_DM_WB <= rf_we_EX_DM;
rf_dst_addr_DM_WB <= rf_dst_addr_EX_DM;
end
end
/////////////////////////////////////////////////////////////
// Flops for bypass control logic (these are ID_EX flops) //
///////////////////////////////////////////////////////////
always @(posedge clk, negedge rst_n)
if (!rst_n)
begin
byp0_EX <= 1'b0;
byp0_DM <= 1'b0;
byp1_EX <= 1'b0;
byp1_DM <= 1'b0;
end
else if (!stall_ID_EX)
begin
byp0_EX <= (rf_dst_addr_ID_EX==rf_p0_addr) ? (rf_we_ID_EX & |rf_p0_addr) : 1'b0;
byp0_DM <= (rf_dst_addr_EX_DM==rf_p0_addr) ? (rf_we_EX_DM & |rf_p0_addr) : 1'b0;
byp1_EX <= (rf_dst_addr_ID_EX==rf_p1_addr) ? (rf_we_ID_EX & |rf_p1_addr) : 1'b0;
byp1_DM <= (rf_dst_addr_EX_DM==rf_p1_addr) ? (rf_we_EX_DM & |rf_p1_addr) : 1'b0;
end
///////////////////////////////////////////
// Flops for pipelining HLT instruction //
/////////////////////////////////////////
always @(posedge clk, negedge rst_n)
if (!rst_n)
begin
hlt_ID_EX <= 1'b0;
hlt_EX_DM <= 1'b0;
hlt_DM_WB <= 1'b0;
//stall_ID_EX <= 1'b0;
//stall_EX_DM <= 1'b0;
end
else
begin
if (!stall_ID_EX) begin
hlt_ID_EX <= hlt & !flush | hlt_ID_EX; // once set stays set
hlt_EX_DM <= hlt_ID_EX;
hlt_DM_WB <= hlt_EX_DM;
end
//stall_ID_EX <= stall_IM_ID;
//stall_EX_DM <= stall_ID_EX;
end
//////////////////////////////////////////
// Have to pipeline flow_change so can //
// flush the 2 following instructions //
///////////////////////////////////////
always @(posedge clk, negedge rst_n)
if (!rst_n)
flow_change_EX_DM <= 1'b0;
else if(!stall_EX_DM)// || (flow_change_ID_EX && !i_rdy))
flow_change_EX_DM <= flow_change_ID_EX;
assign flush = flow_change_ID_EX | flow_change_EX_DM | hlt_ID_EX | hlt_EX_DM;
////////////////////////////////
// Load Use Hazard Detection //
//////////////////////////////
assign load_use_hazard = (((rf_dst_addr_ID_EX==rf_p0_addr) && rf_re0) ||
((rf_dst_addr_ID_EX==rf_p1_addr) && rf_re1)) ? dm_re_ID_EX : 1'b0;
assign stall_IM_ID = hlt & !flush | hlt_ID_EX | load_use_hazard | !i_rdy;
assign stall_ID_EX = !i_rdy; // hlt_EX_DM;
assign stall_EX_DM = !i_rdy;//1'b0;//!i_rdy; // hlt_EX_DM;
assign stall_DM_WB = !i_rdy;//1'b0;//!i_rdy; // hlt_EX_DM;
//assign stall_IM_ID = hlt & !flush | hlt_ID_EX | load_use_hazard | !i_rdy;
//assign stall_ID_EX = stall_IM_ID//!i_rdy;//1'b0;//!i_rdy; // hlt_EX_DM;
//assign stall_EX_DM = !i_rdy;//1'b0;//!i_rdy; // hlt_EX_DM;
assign cc_ID_EX = instr_ID_EX[11:9];
//////////////////////////////////////////////////////////////
// default to most common state and override base on instr //
////////////////////////////////////////////////////////////
always @(instr_IM_ID) begin
br_instr = 0;
jmp_imm = 0;
jmp_reg = 0;
rf_re0 = 0;
rf_re1 = 0;
rf_we = 0;
rf_p0_addr = instr_IM_ID[3:0];
rf_p1_addr = instr_IM_ID[7:4];
rf_dst_addr = instr_IM_ID[11:8];
alu_func = ADD;
src0sel = RF2SRC0;
src1sel = RF2SRC1;
dm_re = 0;
dm_we = 0;
clk_z = 0;
clk_nv = 0;
hlt = 0;
// cond_ex = 0;
padd = 0;
case (instr_IM_ID[15:12])
ADDi : begin
rf_re0 = 1;
rf_re1 = 1;
rf_we = 1;
clk_z = 1;
clk_nv = 1;
end
PADDSBi : begin
rf_re0 = 1;
rf_re1 = 1;
rf_we = 1; // potentially knocked down in next pipe reg
//clk_z = 1;
//clk_nv = 1;
//cond_ex = 1; // this is a conditionally executing instruction
padd = 1;
end
SUBi : begin
rf_re0 = 1;
rf_re1 = 1;
rf_we = 1;
alu_func = SUB;
clk_z = 1;
clk_nv = 1;
end
ANDi : begin
rf_re0 = 1;
rf_re1 = 1;
rf_we = 1;
alu_func = AND;
clk_z = 1;
end
NORi : begin
rf_re0 = 1;
rf_re1 = 1;
rf_we = 1;
alu_func = NOR;
clk_z = 1;
end	
SLLi : begin
rf_re1 = 1;
rf_we = 1;
alu_func = SLL;
clk_z = 1;
end
SRLi : begin
rf_re1 = 1;
rf_we = 1;
alu_func = SRL;
clk_z = 1;
end	
SRAi : begin
rf_re1 = 1;
rf_we = 1;
alu_func = SRA;
clk_z = 1;
end
LWi : begin
src0sel = IMM2SRC0; // sign extended address offset
rf_re1 = 1;
rf_we = 1;
dm_re = 1;
end
SWi : begin
src0sel = IMM2SRC0; // sign extended address offset
rf_re1 = 1; // read register that contains address base
rf_re0 = 1; // read register to be stored
rf_p0_addr = instr_IM_ID[11:8]; // register to be stored is encoded in [11:8]
dm_we = 1;
end
LHBi : begin
rf_re0 = 1;
rf_p0_addr = instr_IM_ID[11:8]; // need to preserve lower byte, access it so can be recycled
src1sel = IMM2SRC1; // access 8-bit immediate.
rf_we = 1;
alu_func = LHB;
end
LLBi : begin
rf_re0 = 1; // access zero from reg0 and ADD
rf_p0_addr = 4'h0; // reg0 contains zero
src1sel = IMM2SRC1; // access 8-bit immediate
rf_we = 1;
end
BRi : begin
src0sel = IMM_BR2SRC0; // 8-bit SE immediate
src1sel = NPC2SRC1; // nxt_pc is routed to source 1
br_instr = 1;
end
JALi : begin
src0sel = IMM_JMP2SRC0; // 12-bit SE immediate
src1sel = NPC2SRC1; // nxt_pc is routed to source 1
rf_we = 1;
rf_dst_addr = 4'hF; // for JAL we write nxt_pc to reg15
jmp_imm = 1;
end
JRi : begin
rf_re0 = 1; // access zero from reg0 and ADD
rf_p0_addr = 4'h0;
rf_re1 = 1; // read register to jump to on src1
jmp_reg = 1;
end
HLTi : begin
hlt = 1;
end
endcase
end
endmodule
module pc(clk,rst_n,stall_IM_ID,dst_ID_EX,
          pc,pc_ID_EX,flow_change_ID_EX,pc_EX_DM,i_rdy,stall_ID_EX, stall_EX_DM);
////////////////////////////////////////////////////////////////////////////\
// This module implements the program counter logic. It normally increments \\
// the PC by 1, but when a branch is taken will add the 9-bit immediate      \\
// field to the PC+1.  In case of a jmp_imm it will add the 12-bit immediate //
// field to the PC+1.  In the case of a jmp_reg it will use the register    //
// port zero (p0) register access as the new value of the PC.  It also     //
// provides PC+1 as nxt_pc for JAL instructions.                          //
///////////////////////////////////////////////////////////////////////////
input clk,rst_n, i_rdy;
input flow_change_ID_EX;			// asserted from branch boolean on jump or taken branch
input stall_IM_ID, stall_ID_EX, stall_EX_DM;					// asserted if we need to stall the pipe
input [15:0] dst_ID_EX;				// branch target address comes in on this bus

output [15:0] pc;					// the PC, forms address to instruction memory
output reg [15:0] pc_ID_EX;			// needed in EX stage for Branch instruction
output reg [15:0] pc_EX_DM;			// needed in dst_mux for JAL instruction

reg [15:0] pc,pc_IM_ID;

wire [15:0] nxt_pc;

/////////////////////////////////////
// implement incrementer for PC+1 //
///////////////////////////////////
assign nxt_pc = pc + 1;

////////////////////////////////
// Implement the PC register //
//////////////////////////////
always @(posedge clk, negedge rst_n)
  if (!rst_n)
    pc <= 16'h0000;			// can't pass sumLoop... but passed everything else!
  else if (!stall_IM_ID) // || !i_rdy)	// all stalls stall the PC
    if (flow_change_ID_EX)
      pc <= dst_ID_EX;
    else
	  pc <= nxt_pc;
	  
////////////////////////////////////////////////
// Implement the PC pipelined register IM_ID //
//////////////////////////////////////////////
always @(posedge clk)
  if (!stall_IM_ID)// || (flow_change_ID_EX && !i_rdy))
    pc_IM_ID <= nxt_pc;		// pipeline PC points to next instruction
	
////////////////////////////////////////////////
// Implement the PC pipelined register ID_EX //
//////////////////////////////////////////////
always @(posedge clk)
  if (!stall_ID_EX)
    pc_ID_EX <= pc_IM_ID;	// pipeline it down to EX stage for jumps
	
////////////////////////////////////////////////
// Implement the PC pipelined register EX_DM //
//////////////////////////////////////////////
always @(posedge clk)
  if (!stall_EX_DM)
    pc_EX_DM <= pc_ID_EX;	// pipeline it down to DM stage for saved register for JAL

endmodule
module rf(clk,p0_addr,p1_addr,p0,p1,re0,re1,dst_addr,dst,we,hlt);
//////////////////////////////////////////////////////////////////
// Triple ported register file.  Two read ports (p0 & p1), and //
// one write port (dst).  Data is written on clock high, and  //
// read on clock low //////////////////////////////////////////
//////////////////////

input clk;
input [3:0] p0_addr, p1_addr;			// two read port addresses
input re0,re1;							// read enables (power not functionality)
input [3:0] dst_addr;					// write address
input [15:0] dst;						// dst bus
input we;								// write enable
input hlt;								// not a functional input.  Used to dump register contents when
										// test is halted.

output reg [15:0] p0,p1;  				//output read ports

integer indx;

reg [15:0]mem[0:15];

/*
reg [3:0] dst_addr_lat;					// have to capture dst_addr from previous cycle
reg [15:0] dst_lat;						// have to capture write data from previous cycle
reg we_lat;								// have to capture we from previous cycle
*/

//////////////////////////////////////////////////////////
// Register file will come up uninitialized except for //
// register zero which is hardwired to be zero.       //
///////////////////////////////////////////////////////
initial begin
  mem[0] = 16'h0000;					// reg0 is always 0,
end

/*
//////////////////////////////////////////////////
// dst_addr, dst, & we all need to be latched  //
// on clock low of previous cycle to maintain //
// in clock high write of next cycle.        //
//////////////////////////////////////////////
always @(clk,dst_addr,dst,we)
  if (~clk)
    begin
	  dst_addr_lat <= dst_addr;
	  dst_lat      <= dst;
	  we_lat       <= we;
	end
*/

//////////////////////////////////
// RF is written on clock high //
////////////////////////////////
always @(clk,we,dst_addr,dst)
  if (clk && we && |dst_addr)
    mem[dst_addr] <= dst;
	
//////////////////////////////
// RF is read on clock low //
////////////////////////////
always @(clk,re0,p0_addr)
  if (~clk && re0)
    p0 <= mem[p0_addr];
	
//////////////////////////////
// RF is read on clock low //
////////////////////////////
always @(clk,re1,p1_addr)
  if (~clk && re1)
    p1 <= mem[p1_addr];
	
////////////////////////////////////////
// Dump register contents at program //
// halt for debug purposes          //
/////////////////////////////////////
always @(posedge hlt)	//hlt
  for(indx=0; indx<16; indx = indx+1)
    //$display("R%1h = %h",indx,mem[indx]);
	
endmodule
  
module src_mux(clk,stall_ID_EX,stall_EX_DM,src0sel_ID_EX,src1sel_ID_EX,p0,p1,
               imm_ID_EX,pc_ID_EX,p0_EX_DM,src0,src1,dst_EX_DM,dst_DM_WB,
			   byp0_EX,byp0_DM,byp1_EX,byp1_DM);

input clk;
input stall_ID_EX,stall_EX_DM;					// stall signal
input [1:0] src0sel_ID_EX,src1sel_ID_EX;		// mux selectors for src0 and src1 busses
input [15:0] p0;					// port 0 from register file
input [15:0] p1;					// port 1 from register file
input [15:0] pc_ID_EX;				// Next PC for JAL instruction
input [11:0] imm_ID_EX;				// immediate from instruction stream goes on src0
input [15:0] dst_EX_DM;				// EX_DM results for bypassing RF reads
input [15:0] dst_DM_WB;				// DM_WB results for bypassing RF reads
input byp0_EX,byp1_EX;				// From ID, selects EX results to bypass RF sources
input byp0_DM,byp1_DM;				// From ID, selects DM results to bypass RF sources
output reg [15:0] p0_EX_DM;			// need to output this as data for SW instructions
output [15:0] src0,src1;			// source busses

/////////////////////////////////
// registers needed for flops //
///////////////////////////////
reg [15:0] p0_ID_EX,p1_ID_EX;		// need to flop register file outputs to form _ID_EX versions

wire[15:0] RF_p0,RF_p1;				// output of bypass muxes for RF sources
/////////////////////
// include params //
///////////////////
`include "common_params.inc"

/////////////////////////////////////////////////
// Flop the read ports from the register file //
///////////////////////////////////////////////
always @(posedge clk)
  if (!stall_ID_EX)
    begin
	  p0_ID_EX <= p0;
	  p1_ID_EX <= p1;
	end
	
/////////////////////////////
// Bypass Muxes for port0 //
///////////////////////////
assign RF_p0 = (byp0_EX) ? dst_EX_DM :		// EX gets priority because it represents more recent data
               (byp0_DM) ? dst_DM_WB :
			   p0_ID_EX;
	
/////////////////////////////
// Bypass Muxes for port1 //
///////////////////////////
assign RF_p1 = (byp1_EX) ? dst_EX_DM :		// EX gets priority because it represents more recent data
               (byp1_DM) ? dst_DM_WB :
			   p1_ID_EX;	
			   
////////////////////////////////////////////////////
// Need to pipeline the data to be stored for SW //
//////////////////////////////////////////////////
always @(posedge clk)
  if (!stall_EX_DM)
	p0_EX_DM <= RF_p0;
	
assign src0 = (src0sel_ID_EX == RF2SRC0) ? RF_p0 : 
              (src0sel_ID_EX == IMM_BR2SRC0) ? {{7{imm_ID_EX[8]}},imm_ID_EX[8:0]} :		// branch immediates
			  (src0sel_ID_EX == IMM_JMP2SRC0) ? {{4{imm_ID_EX[11]}},imm_ID_EX[11:0]} :	// JMP immediates
              {{12{imm_ID_EX[3]}},imm_ID_EX[3:0]};		// for address immediates for DM operations

assign src1 = (src1sel_ID_EX == RF2SRC1) ? RF_p1 : 
              (src1sel_ID_EX == NPC2SRC1) ? pc_ID_EX :	// for JAL
			  {{8{imm_ID_EX[7]}},imm_ID_EX[7:0]};			// for LHB/LLB (sign extended 8-bit immediate
			  
endmodule
module t_cpu();
    
reg clk,rst_n;
reg [15:0] i;

wire [15:0] pc;

//////////////////////
// Instantiate CPU //
////////////////////
cpu iCPU(.clk(clk), .rst_n(rst_n), .hlt(hlt), .pc(pc));

initial begin
  clk = 0;
  //$display("rst assert\n");
  rst_n = 0;
  @(posedge clk);
  @(negedge clk);
  rst_n = 1;
  //$display("rst deassert\n");
end 
  
always
  #5 clk = ~clk;
  
always @(posedge clk, negedge rst_n)begin
if (~rst_n) begin
i <= 0;
end else begin
i <= i + 1;
end
////$display("\n\nclk=%d%d%d%d%d%d%d%d%d%d%d%d%d\n\n", i,i,i,i,i,i,i,i,i,i,i,i,i);
if (i == 100) begin
  //$finish();
end
end
  
initial begin
  @(posedge hlt);
  @(posedge clk);
  $stop();
end    
  
endmodule
module unified_mem(clk,rst_n,addr,re,we,wdata,rd_data,rdy);

/////////////////////////////////////////////////////////////////////
// Unified memory with 4-clock access times for reads & writes.   //
// Organized as 16384 64-bit words (i.e. same width a cache line //
//////////////////////////////////////////////////////////////////
input clk,rst_n;
input re,we;
input [13:0] addr;			// 2 LSB's are dropped since accessing as four 16-bit words
input [63:0] wdata;

output reg [63:0] rd_data;
output reg rdy;				// deasserted when memory operation completed

reg [15:0]mem[0:65535];			// entire memory space at 16-bits wide

//////////////////////////
// Define states of SM //
////////////////////////
localparam IDLE 	= 2'b00;
localparam WRITE 	= 2'b01;
localparam READ    	= 2'b10;

reg [13:0] addr_capture;								// capture the address at start of read
reg [1:0] state,nxt_state;								// state register
reg [1:0] wait_state_cnt;								// counter for 4-clock access time
reg clr_cnt,int_we,int_re;								// state machine outputs

////////////////////////////////
// initial load of instr.hex //
//////////////////////////////
initial
  $readmemh("instr.hex",mem);	//SumLoop		MultRoutine
  
/////////////////////////////////////////////////
// Capture address at start of read or write  //
// operation to ensure address is held       //
// stable during entire access time.        //
/////////////////////////////////////////////
always @(posedge clk)
  if (re | we)
    addr_capture <= addr;			// this is actual address used to access memory

//////////////////////////
// Model memory writes //
////////////////////////
always @(clk,int_we)
  if (clk & int_we)				// write occurs on clock high during 4th clock cycle
    begin
      mem[{addr_capture,2'b00}] <= wdata[15:0];
	  mem[{addr_capture,2'b01}] <= wdata[31:16];
	  mem[{addr_capture,2'b10}] <= wdata[47:32];
	  mem[{addr_capture,2'b11}] <= wdata[63:48];
	end
	
/////////////////////////
// Model memory reads //
///////////////////////
always @(clk,int_re)
  if (clk & int_re)				// reads occur on clock high during 4th clock cycle
    rd_data = {mem[{addr_capture,2'b11}],mem[{addr_capture,2'b10}],mem[{addr_capture,2'b01}],mem[{addr_capture,2'b00}]};
	 
	
////////////////////////
// Infer state flops //
//////////////////////
always @(posedge clk, negedge rst_n)
  if (!rst_n)
    state <= IDLE;
  else
    state <= nxt_state;
	
/////////////////////////
// wait state counter //
///////////////////////
always @(posedge clk, negedge rst_n)
  if (!rst_n)
    wait_state_cnt <= 2'b00;
  else
    if (clr_cnt)
      wait_state_cnt <= 2'b00;
	else
      wait_state_cnt <= wait_state_cnt + 1;
	
always @(state,re,we,wait_state_cnt)
begin
////////////////////////////
// default outputs of SM //
//////////////////////////
clr_cnt = 1;		// hold count in reset
int_we = 0;		// wait till 4th clock
int_re = 0;		// wait till 4th clock
nxt_state = IDLE;
	
case (state)
	IDLE : if (we) begin
	       clr_cnt = 0;
	       rdy = 0;
	       nxt_state = WRITE;
	       end
	       else if (re) begin
	       clr_cnt = 0;
	       rdy = 0;
	       nxt_state = READ;
	       end
	       else rdy = 1;
	  
	WRITE : if (&wait_state_cnt) begin
	        int_we = 1;		// write completes and next state is IDLE
		rdy = 1;
		end
		else begin
		clr_cnt = 0;
		rdy = 0;
		nxt_state = WRITE;
		end
	  
	default : if (&wait_state_cnt) begin	// this state is READ
		  int_re = 1;	// read completes and next state is IDLE
		  rdy = 1;
		  end
		  else begin
		  clr_cnt = 0;
		  rdy = 0;
		  nxt_state = READ;
		  end
endcase
end

endmodule
	
	

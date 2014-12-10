module cache(clk,rst_n,toggle,addr,wr_data,wdirty,we,re,rd_data,tag_out,hit,dirty);

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
  //$display("match_hi=%b\nmatch_lo=%b\nhit=%b\nline[74:0]=%h\nLRU=%b\ntag=%b\nline[147:139]=%b\naddr[13:5]=%b\nline[72:64]=%b\n",match_hi,match_lo,hit,line[74:0],line[150],tag_out,line[147:139],addr[13:5],line[72:64]);  
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
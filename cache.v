module cache(clk,rst_n,addr,wr_data,wdirty,we,re,rd_data,tag_out,hit,dirty);

input clk,rst_n;
input [13:0] addr;		// address to be read or written, 2-LSB's are dropped
input [63:0] wr_data;	// 64-bit cache line to write
input wdirty;			// dirty bit to be written
input we;				// write enable for cache line
input re;				// read enable (for power purposes only)

output hit;
output dirty;
output [63:0] rd_data;	// 64-bit cache line read out
output [7:0] tag_out;	// 8-bit tag.  This is needed during evictions

reg [73:0]mem[0:63];	// {valid,dirty,tag[7:0],wdata[63:0]}
reg [6:0] x;
reg [73:0] line;
reg we_del;

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
    for (x=0; x<64;  x = x + 1)
	  mem[x] = {2'b00,{72{1'bx}}};		// only valid & dirty bit are cleared, all others are x
  else if (~clk && we_filt)
    mem[addr[5:0]] = {1'b1,wdirty,addr[13:6],wr_data};

////////////////////////////////////////////////////////////
// Model cache read including 4:1 muxing of 16-bit words //
//////////////////////////////////////////////////////////
always @(clk or re or addr)
  if (clk && re)				// read is on clock high
    line = mem[addr[5:0]];
    
	
/////////////////////////////////////////////////////////////
// If tag bits match and line is valid then we have a hit //
///////////////////////////////////////////////////////////
assign hit = ((line[71:64]==addr[13:6]) && (re | we)) ? line[73] : 1'b0;
assign dirty = line[73]&line[72];						// if line is valid and dirty bit set
assign rd_data = line[63:0];
assign tag_out = line[71:64];							// need the tag for evictions


always @(clk) begin
$display("line[71:64]=%h, addr[13:6]=%h\n", line, addr);
end
	
endmodule

module two_way_cache(clk, rst_n, toggle, addr, wr_data, wdirty, we, re, rd_data, tag_out, hit, dirty);
// Two-way set associative cache with 32 cache lines
// 151-bit cache lines, two 64-bit data with 9-bit tag lines, dirty, and valid with one overall LRU bit
// Functioning for large majority of tests including set associative specific tests
// Authors: David Hartman and John Peterson
// Date modified: 12 Dec 2014

input clk, rst_n;
input [13:0] addr; // chopped address for cache block
input [63:0] wr_data; // data to write into the cache
input wdirty; // write the dirty bit
input we; // perform cache write
input re; // perform cache read
input toggle; // update LRU bit (added from direct-mapped cache)

output reg hit;
output dirty;
output reg [63:0] rd_data;
output reg [8:0] tag_out; // 9-bit tag (one more bit required with one less index bit)

reg [74:0]mem1[0:31]; // Upper block in 32-index cache block
reg [74:0]mem0[0:31]; // lower block in 32-index cache block
reg [31:0] LRU; // LRU bits for 32-index cache block
reg [5:0] x; // counter variable for reset

reg [74:0] mem1_line; // register for upper cache block read
reg [74:0] mem0_line; // register for lower cache block read

wire hit1, hit0; // upper and lower hit signal

reg we_del;
wire we_filt;

always @(we) // glitch filter for we
  	we_del <= we;
assign we_filt = we & we_del;

// 2-way cache write. prefer lower half on reset.
always @(clk or we_filt or negedge rst_n) begin
	if(!rst_n) begin
		for(x=0;x<32;x=x+1)begin
			mem1[x] = {2'b00, {73{1'bx}}};
			mem0[x] = {2'b00, {73{1'bx}}};
			LRU[x] = 1'b0;
		end
	end
	else if(~clk && we_filt) begin
		// write the cache, based on which was LRU
		if(LRU[addr[4:0]] == 1'b0) begin
			//$display("LRU = low, writing");
			mem0[addr[4:0]] = {1'b1,wdirty,addr[13:5],wr_data};
			if(toggle == 1'b1) LRU[addr[4:0]] = 1'b1;
		end
		else begin
			//$display("LRU = high, writing");
			mem1[addr[4:0]] = {1'b1,wdirty,addr[13:5],wr_data};
			if(toggle == 1'b1) LRU[addr[4:0]] = 1'b0;
		end
		//if(toggle == 1'b1) LRU[addr[4:0]] = !LRU[addr[4:0]]; wanted to toggle LRU always, but had problems
	end
end

// 2-way cache read.
always @(clk or re or addr) begin
	if(clk && re) begin
		mem1_line = mem1[addr[4:0]];
		mem0_line = mem0[addr[4:0]];
	end
end

// assign hit values to match tag while writing or reading and data valid
assign hit1 = ((mem1_line[72:64]==addr[13:5]) && (re | we)) ? mem1_line[74] : 1'b0;
assign hit0 = ((mem0_line[72:64]==addr[13:5]) && (re | we)) ? mem0_line[74] : 1'b0;
// assign dirty output if LRU half of desired block is dirty and valid
assign dirty = (LRU[addr[4:0]]) ? (mem1_line[74]&mem1_line[73]) : (mem0_line[74]&mem0_line[73]);
always @(*) begin
	hit = (hit1 | hit0);
	//$display("hit1=%b, hit0=%b", hit1, hit0);
	if(hit1==1'b1) begin
		//$display("Accessing high");
			rd_data = mem1_line[63:0];
			tag_out = mem1_line[72:64];
	end else if(hit0 == 1'b1) begin
		//$display("Accessing low");
			rd_data = mem0_line[63:0];
			tag_out = mem0_line[72:64];
	end
end

endmodule
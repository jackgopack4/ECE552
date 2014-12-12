module two_way_cache(clk, rst_n, toggle, addr, wr_data, wdirty, we, re, rd_data, tag_out, hit, dirty);

input clk, rst_n;
input [13:0] addr;
input [63:0] wr_data;
input wdirty;
input we;
input re;
input toggle;

output reg hit;
output dirty;
output reg [63:0] rd_data;
output reg [8:0] tag_out;

reg [74:0]mem1[0:31];
reg [74:0]mem0[0:31];
reg [31:0] LRU;
reg [5:0] x;

reg [74:0] mem1_line;
reg [74:0] mem0_line;

wire hit1, hit0;

reg we_del;
wire we_filt;

always @(we)
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
		//if(toggle == 1'b1) LRU[addr[4:0]] = !LRU[addr[4:0]];
	end
end

// 2-way cache read.
always @(clk or re or addr) begin
	if(clk && re) begin
		mem1_line = mem1[addr[4:0]];
		mem0_line = mem0[addr[4:0]];
	end
end

// assign outputs
assign hit1 = ((mem1_line[72:64]==addr[13:5]) && (re | we)) ? mem1_line[74] : 1'b0;
assign hit0 = ((mem0_line[72:64]==addr[13:5]) && (re | we)) ? mem0_line[74] : 1'b0;

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

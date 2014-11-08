module PC(nextAddr, clk, rst, programCounter);

input [15:0] nextAddr;
input clk, rst;
output reg [15:0] programCounter;

always @(posedge clk) begin
	if (rst) begin
		programCounter <= 4'h0000;
	end else begin
		programCounter <= nextAddr;
	end

end

endmodule

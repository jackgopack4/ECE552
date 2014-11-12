module sign_extenderBranch(
	input signed [3:0] in,
	output signed [15:0] out);

  assign out = in; // sign-extends

endmodule

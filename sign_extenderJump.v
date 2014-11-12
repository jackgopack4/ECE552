module sign_extenderJump(
	input signed [11:0] in,
	output signed [15:0] out);

  assign out = in; // sign-extends

endmodule

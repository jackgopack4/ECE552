module loadmod(Out, A, B, Sel);

  input [15:0] A, B;
  input Sel;
  output [15:0] Out;

  assign Out = (Sel)? B :
	              {B[7:0], A[7:0]};

endmodule

module arithmod(Out, V, N, Z, A, B, Sel);
  input [15:0] A, B;
  input [1:0] Sel;
  
  output [15:0] Out;
  output V, N, Z;

  wire cout;
  wire [15:0] adder_out, paddsb_out, adder_result, B_nor, sel_nor;
  wire sign;
  split16 splitSel(sel_nor, Sel[1]);
  assign B_nor = (sel_nor ^ B);
  add16bit Adder(cout, adder_out, A, B_nor, Sel[1]);
  overflow check(V, sign, A[15], B_nor[15], adder_out[15], Sel[1]);
  assign adder_result = (V) ? ((sign) ? 16'h8000 : 16'h7FFF):
	                       adder_out;
  assign Out = (Sel[0]) ? paddsb_out :
	                  adder_result;
  assign N = (adder_result[15]);
  assign Z = (adder_result == 16'h0000);

endmodule

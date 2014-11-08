module FA(c_out, sum, a, b, c_in);
	output sum, c_out;
	input a, b, c_in;
	wire w1, w2, w3;

	HA AH1(.sum(w1), .c_out(w2), .a(a), .b(b));
	HA AH2(.sum(sum), .c_out(w3), .a(c_in), .b(w1));
	or carry_bit(c_out, w2, w3);
endmodule

module HA(c_out, sum, a, b);
	output sum, c_out;
	input a, b;

	xor sum_bit(sum, a, b);
	and carry_bit(c_out, a, b);
endmodule

module add16bit(cout, sum, a, b, cin);

  output [15:0] sum;
  output cout;
  input cin;
  input [15:0] a, b;
  wire cout0;

  add8bit A0(cout0, sum[7:0], a[7:0], b[7:0], cin);
  add8bit A1(cout, sum[15:8], a[15:8], b[15:8], cout0);

endmodulemodule add8bit(cout, sum, a, b, cin);
    output [7:0] sum;
    output cout;
    input cin;
    input [7:0] a, b;

    wire cout0, cout1, cout2, cout3, cout4, cout5, cout6;

    FA A0(cout0, sum[0], a[0], b[0], cin);
    FA A1(cout1, sum[1], a[1], b[1], cout0);
    FA A2(cout2, sum[2], a[2], b[2], cout1);
    FA A3(cout3, sum[3], a[3], b[3], cout2);
    FA A4(cout4, sum[4], a[4], b[4], cout3);
    FA A5(cout5, sum[5], a[5], b[5], cout4);
    FA A6(cout6, sum[6], a[6], b[6], cout5);
    FA A7(cout, sum[7], a[7], b[7], cout6);
    
endmodule

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
module loadmod(Out, A, B, Sel);

  input [15:0] A, B;
  input Sel;
  output [15:0] Out;

  assign Out = (Sel)? B :
	              {B[7:0], A[7:0]};

endmodule
module logicmod(Out, Z, A, B, Sel);

  input [15:0] A, B;
  input Sel;
  output [15:0] Out;
  output Z;

  wire [15:0] andwire, norwire;

  assign andwire = (A & B);
  assign norwire = ~(A | B);
  assign Out = (Sel) ? norwire : andwire;
  assign Z = (Out == 16'h0000);

endmodule
module overflow(Ov, Sign,  A_hi, B_hi, Sum_hi, Sub);

  input A_hi, B_hi, Sum_hi, Sub;
  output Ov, Sign;
  wire t0, t1;
  wire ov_add, ov_sub;

  xor3 add(.a(A_hi), .b(B_hi), .c(Sum_hi), .out(t0));
  xor3 sub(.a(A_hi), .b(!B_hi), .c(Sum_hi), .out(t1));

  assign ov_add = (t0 & !Sub);
  assign ov_sub = (t1 & Sub);

  assign Ov = (ov_add | ov_sub);
  assign Sign = (A_hi & (B_hi ^ Sub)); //Negative overflow is high

endmodule
module paddsb(Sum, A, B);

  input [15:0] A, B;
  output [15:0] Sum;

  wire cout_hi, cout_lo;
  wire [7:0] adder_out_hi, adder_out_lo;

  wire Ov_hi, Ov_lo, Sign_hi, Sign_lo;

  add8bit add_hi(cout_hi, adder_out_hi, A[15:8], B[15:8], 1'b0);
  add8bit add_lo(cout_lo, adder_out_lo, A[7:0],  B[7:0],  1'b0);

  overflow check_hi(Ov_hi, Sign_hi, A[15], B[15], add_hi[7], 1'b0);
  overflow check_lo(Ov_lo, Sign_lo, A[7],  B[7],  add_lo[7], 1'b0);

  assign Sum[15:8] = (Ov_hi) ? ((Sign_hi) ? 8'b10000000 : 8'b01111111):
	                       adder_out_hi;
  assign Sum[7:0]  = (Ov_lo) ? ((Sign_lo) ? 8'b10000000 : 8'b01111111):
	                       adder_out_lo;

endmodule
module shifter(dst, src, m, shamt);

  localparam SLL = 2'b01;
  localparam SRL = 2'b10;
  localparam SRA = 2'b11;

  output [15:0] dst;
  input  [15:0] src;
  input  [1:0]  m;
  input  [3:0]  shamt;

  wire   [15:0] shift0, shift1, shift2, shift3;
  wire   [15:0] sel0, sel1, sel2;

  assign sel0 = (shamt[0]) ? shift0 : src;
  assign sel1 = (shamt[1]) ? shift1 : sel0;
  assign sel2 = (shamt[2]) ? shift2 : sel1;
  assign dst  = (shamt[3]) ? shift3 : sel2;

  assign shift0 = (m==SLL) ? {src[14:0],1'b0}   :
	          (m==SRL) ? {1'b0,src[15:1]}   :
		             {src[15],src[15:1]};

  assign shift1 = (m==SLL) ? {sel0[13:0],2'b00}        :
	          (m==SRL) ? {2'b00,sel0[15:2]}        :
                             {sel0[15],sel0[15],sel0[15:2]};
  
  assign shift2 = (m==SLL) ? {sel1[11:0],4'h0}         :
	          (m==SRL) ? {4'h0,sel1[15:4]}         :
		             {sel1[15],sel1[15],sel1[15],sel1[15],sel1[15:4]};

  assign shift3 = (m==SLL) ? {sel2[7:0],8'h0} :
	          (m==SRL) ? {8'h0,sel2[15:8]} :
		             {sel2[15],sel2[15],sel2[15],sel2[15],sel2[15],sel2[15],sel2[15],sel2[15],sel2[15:8]};

endmodule
module shiftmod(Out, Z, A, Op, Shamt);

  input [15:0] A;
  input [1:0] Op;
  input [3:0] Shamt;

  output [15:0] Out;
  output Z;

  shifter shift_it(Out, A, Op, Shamt);
  assign Z = (A == 16'h0000);

endmodule
module split16(Out, In);
input In;
output [15:0] Out;
assign Out = {In, In, In, In, In, In, In, In, In, In, In, In, In, In, In, In};
endmodule
module xor3(out, a, b, c);
  input a, b, c;
  output out;
  assign out = (a && b && c)    ? 1'b0:
	       (!a && !b && !c) ? 1'b0:
	                          1'b1;
endmodule
module ALU( dst,
	    ov,
	    zr,
	    neg,
	    src0,
	    src1,
	    op,
	    shamt);

  output [15:0] dst;
  output ov, zr, neg; // Signals for flags
  input [15:0] src0, src1;
  input [3:0] op;     // op code determines control
  input [3:0] shamt;  // how much src0 is shifted

  wire [15:0] arithout, logicout, shiftout, loadout;
  wire [1:0] Sel;
  wire v_arith, n_arith, z_arith, z_logic, z_shift;

  arithmod am(arithout, v_arith, n_arith, z_arith, src0, src1, op[1:0]);
  logicmod lm(logicout, z_logic, src0, src1, op[2]);
  shiftmod sm(shiftout, z_shift, src0, op[1:0], shamt);
  loadmod  dm(loadout, src0, src1, op[0]);

  assign Sel = (op[3:1] == 3'b000 | op == 4'h4)  ? 2'b00 :
	       ((op == 4'b0101 | op == 4'b0100)  ? 2'b01 :
	       ((op == 0101 | op[3:1] == 3'b011) ? 2'b10 :
	                                           2'b11));

  assign ov  = (v_arith);
  assign neg = (n_arith);
  assign zr = (Sel == 2'b00) ? z_arith :
	      ((Sel == 2'b01) ? z_logic :
	      z_shift);


endmodule

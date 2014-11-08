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

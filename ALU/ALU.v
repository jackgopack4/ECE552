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

  

endmodule

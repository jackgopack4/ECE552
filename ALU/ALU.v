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

  assign ov = (Sel == 2'b00) ? v_arith  : 1'b0;
  assign neg = (Sel == 2'b00) ? n_arith : 1'b0;
  assign zr = (Sel == 2'b00)  ? z_arith :
	      ((Sel == 2'b01) ? z_logic :
	      ((Sel == 2'b10) ? z_shift :
	                        1'b0));

  assign dst = (Sel == 2'b00)  ? arithout : 
	       ((Sel == 2'b01) ? logicout :
	       ((Sel == 2'b10) ? shiftout :
	                         loadout));

endmodule

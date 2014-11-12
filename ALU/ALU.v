module ALU( dst,
	    ov,
	    zr,
	    neg,
	    src0,
	    src1,
	    op,
	    shamt);

  output [15:0] dst;
  output reg ov, zr, neg; // Signals for flags
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

  assign Sel = (op[3:1] == 3'b000 || op == 4'b0010)  ? 2'b00 :
	       ((op == 4'b0011 || op == 4'b0100)  ? 2'b01 :
	       ((op == 4'b0101 || op[3:1] == 3'b011) ? 2'b10 :
	                                           2'b11));

//  always@(Sel) $display("Sel changed to: %b", Sel);
//  always@(op) $display("op changed to: %b", op);
  always@(src0, src1, op, shamt) begin
    ov  = 0;
    zr  = 0;
    neg = 0;
    case(op) begin
      4'b0000: begin
	if(v_arith == 1'b1) ov  = 1;
	if(n_arith == 1'b1) neg = 1;
	if(z_arith == 1'b1) zr  = 1;
      end
      4'b0010: begin
        if(v_arith == 1'b1) ov  = 1;
        if(n_arith == 1'b1) neg = 1;
        if(z_arith == 1'b1) zr  = 1;
      end
      4'b0011: begin
        if(z_logic == 1'b1) zr = 1;
      end
      4'b0100: begin
        if(z_logic == 1'b1) zr = 1;
      end
      4'b0101: begin
        if(z_shift == 1'b1) zr = 1;
      end
      4'b0110: begin
        if(z_shift == 1'b1) zr = 1;
      end
      4'b0111: begin
        if(z_shift == 1'b1) zr = 1;
      end
      default: begin
	      ov = 0;
	      neg = 0;
	      zr = 0;
      end
    endcase
  end

  assign dst = (Sel == 2'b00)  ? arithout : 
	       ((Sel == 2'b01) ? logicout :
	       ((Sel == 2'b10) ? shiftout :
	                         loadout));

endmodule

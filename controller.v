module controller(OpCode, RegDst, Branch, MemRead, MemToReg, ALUOp, MemWrite,
					ALUSrc, RegWrite);

input [5:0] OpCode; 
output RegDst, Branch, MemRead, MemToReg, ALUOp, MemWrite, ALUSrc, RegWrite;					

   localparam ADD     = 4'b0000;
   localparam PADDSB  = 4'b0001;
   localparam SUB     = 4'b0010;
   localparam AND     = 4'b0011;
   localparam NOR     = 4'b0100;
   localparam SLL     = 4'b0101;
   localparam SRL     = 4'b0110;
   localparam SRA     = 4'b0111;
   localparam LW      = 4'b1000;
   localparam SW      = 4'b1001;
   localparam LLB     = 4'b1010;
   localparam LHB     = 4'b1011;
   
   localparam B 	  = 4'b1100;
   localparam JAL 	  = 4'b1101;
   localparam JR 	  = 4'b1110;
   localparam HLT	  = 4'b1111;

// could do if (OpCode[5] = 0 -> R-type, etc...


/*
   always @(OpCode) begin
   case (OpCode)
   ADD: 
   // R-type
   RegDst   = 1;
   Branch   = 0;
   MemRead  = 0;
   MemToReg = 0;
   ALUOp    = 1;
   MemWrite = 0;
   ALUSrc   = 0;
   RegWrite = 1;
   
   PADDSB:
   // R-type
   RegDst   = 1;
   Branch   = 0;
   MemRead  = 0;
   MemToReg = 0;
   ALUOp    = 1;
   MemWrite = 0;
   ALUSrc   = 0;
   RegWrite = 1;
   
   SUB:
   // R-type
   RegDst   = 1;
   Branch   = 0;
   MemRead  = 0;
   MemToReg = 0;
   ALUOp    = 1;
   MemWrite = 0;
   ALUSrc   = 0;
   RegWrite = 1;
   
   AND:
   // R-type
   RegDst   = 1;
   Branch   = 0;
   MemRead  = 0;
   MemToReg = 0;
   ALUOp    = 1;
   MemWrite = 0;
   ALUSrc   = 0;
   RegWrite = 1;
   
   NOR:
   // R-type
   RegDst   = 1;
   Branch   = 0;
   MemRead  = 0;
   MemToReg = 0;
   ALUOp    = 1;
   MemWrite = 0;
   ALUSrc   = 0;
   RegWrite = 1;
   
   SLL:
   // R-type
   RegDst   = 1;
   Branch   = 0;
   MemRead  = 0;
   MemToReg = 0;
   ALUOp    = 1;
   MemWrite = 0;
   ALUSrc   = 0;
   RegWrite = 1;
   
   SRL:
   // R-type
   RegDst   = 1;
   Branch   = 0;
   MemRead  = 0;
   MemToReg = 0;
   ALUOp    = 1;
   MemWrite = 0;
   ALUSrc   = 0;
   RegWrite = 1;
   
   SRA:
   // R-type
   RegDst   = 1;
   Branch   = 0;
   MemRead  = 0;
   MemToReg = 0;
   ALUOp    = 1;
   MemWrite = 0;
   ALUSrc   = 0;
   RegWrite = 1;
   
   LW:
   // LW-type
   RegDst   = 0;
   Branch   = 0;
   MemRead  = 1;
   MemToReg = 1;
   ALUOp    = 0;
   MemWrite = 0;
   ALUSrc   = 1;
   RegWrite = 1;
   
   SW:
   // SW-type
   RegDst   = 0;	// don't care
   Branch   = 0;
   MemRead  = 0;
   MemToReg = 0;	//don't care
   ALUOp    = 0;
   MemWrite = 1;
   ALUSrc   = 1;
   RegWrite = 0;
   
   LLB:
   // R-type?
   RegDst   = 1;
   Branch   = 0;
   MemRead  = 0;
   MemToReg = 0;
   ALUOp    = 1;
   MemWrite = 0;
   ALUSrc   = 0;
   RegWrite = 1;
   
   LHB:
   // R-type?
   RegDst   = 1;
   Branch   = 0;
   MemRead  = 0;
   MemToReg = 0;
   ALUOp    = 1;
   MemWrite = 0;
   ALUSrc   = 0;
   RegWrite = 1;
   
   B:
   // not sure about these
   RegDst   = 1;
   Branch   = 0;
   MemRead  = 0;
   MemToReg = 0;
   ALUOp    = 1;
   MemWrite = 0;
   ALUSrc   = 0;
   RegWrite = 1;
   
   JAL:
   // not sure about these
   RegDst   = 1;
   Branch   = 0;
   MemRead  = 0;
   MemToReg = 0;
   ALUOp    = 1;
   MemWrite = 0;
   ALUSrc   = 0;
   RegWrite = 1;
   
   JR:
   // not sure about these
   RegDst   = 1;
   Branch   = 0;
   MemRead  = 0;
   MemToReg = 0;
   ALUOp    = 1;
   MemWrite = 0;
   ALUSrc   = 0;
   RegWrite = 1;
   
   HLT:
   // not sure about these
   RegDst   = 1;
   Branch   = 0;
   MemRead  = 0;
   MemToReg = 0;
   ALUOp    = 1;
   MemWrite = 0;
   ALUSrc   = 0;
   RegWrite = 1;
   
   default:
   RegDst   = 0;
   Branch   = 0;
   MemRead  = 0;
   MemToReg = 0;
   ALUOp    = 0;
   MemWrite = 0;
   ALUSrc   = 0;
   RegWrite = 0;
 
   endcase
   */


endmodule

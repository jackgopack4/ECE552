module cpu_pipeline(clk, rst_n, hlt, pc);

/// initialize inputs/outputs ///
input clk, rst_n;
output [15:0] pc;
output hlt;


///////////////////////
// Initialize Wires //
/////////////////////

// PC //
reg [15:0] nextAddr; 
wire reg [15:0] programCounter;
reg [15:0] pcInc;	// to mediate between PC and nextAddr

// Instruction Memory //
wire reg [15:0] instruction;
wire rd_en;			 // asserted when instruction read desired

// Register Memory //
wire [15:0] readData1, readData2;
wire re0, re1;	// read enables (power not functionality)
wire [3:0] dst_addr;	// write address (reg)
wire [15:0] dst;	// data to be written to register file, write data (dst bus)
wire we;			// write enable
wire hlt;			// not a functional input.  Used to dump register contents when test is halted.
//^^ hlt is wire AND output?	

// ALU //
wire [15:0] ALUResult;
wire ov, zr, neg;
wire [3:0] shamt;
assign shamt = instruction[3:0];
wire [15:0] src1Wire;

// Data Memory //
//wire re;	// asserted when instruction read desired
//wire we;	// asserted when write desired
wire [15:0] rd_data;	//output of data memory -> register write data

// Controller //
wire 	RegDst,		// 1: write back instr[11:8] to register, 0: (sw don't care) lw into register @ instr[3:0]
		Branch, 	// 1: take branch, 0: don't take branch
		MemRead, 	// 1: enable read memory, 0: disable memory read
		MemToReg, 	// 1: write data from memory to registers, 0: write data from ALU to registers
		ALUOp, 		// Can be used to combine LW/SW with ADD, or branch instructions (we are currently not using this?)
		MemWrite,	// 1: enable memory write, 0: disable memory write
		ALUSrc, 	// 0: readData2 -> ALU 1: sign-extended -> ALU
		RegWrite,	// 1: enable right back to register, 0: don't write back to register
		PCSrc;		// 0: take next pc addr, 1: enable branch (sign-ext << 2)


/////////////////////////
// Initialize Modules //
///////////////////////

// Program Counter //
always @(posedge clk) begin
	pcInc <= programCounter + 1;
	if (PCSrc) begin
	// not sure if this would work...
	nextAddr <= pcInc + ($signed(instruction[3:0]) << 2);
	end else begin
	nextAddr <= pcInc;
	end
	// let's see what's going on!
	$display("programCounter=%d, rd_en=%b, instruction=%b, instruction[7:4]=%b, readData1=%b,
	readData2=%b, instruction[3:0]=%b, src1Wire=%b, ALUResult=%b", programCounter, rd_en, instruction, instruction[7:4],
	readData1, readData2, instruction[3:0], src1Wire, ALUResult);
end


// PC //
PC pcMod(.nextAddr(nextAddr), .clk(clk), .rst(rst_n), .programCounter(programCounter));


// Instruction Memory //
IM instMem(.clk(clk), .addr(programCounter), .rd_en(rd_en), .instr(instruction));
assign rd_en = 1'b1;


// Registers //
// po,p1 are 'output reg' in rfSC
rfSC registers(.clk(clk), .p0_addr(instruction[7:4]), .p1_addr(instruction[3:0]),
				.p0(readData1), .p1(readData2), .re0(re0), .re1(re1),
				.dst_addr(dst_addr), .dst(dst), .we(RegWrite), .hlt(hlt));
// Power not functionality?
assign re0 = 1'b1;
assign re1 = 1'b1;
// MUX: Write Register MUX. Changes for lw
assign dst_addr = RegDst ? instruction[11:8] : instruction[3:0];


// ALU //
ALU_test alu(.src0(readData1), .src1(src1Wire), .op(instruction[15:12]), 
.dst(ALUResult), .ov(ov), .zr(zr), .neg(neg), .shamt(shamt));
// MUX: lw/sw instruction use the sign-extended value for src1 input
assign src1Wire = ALUSrc ? $signed(instruction[3:0]) : readData2;
// obvioulsy $sign is bad design... but quick for now


// Data Memory //
DM dataMem(.clk(clk), .addr(ALUResult), .re(MemRead), .we(MemWrite), 
		   .wrt_data(readData2), .rd_data(rd_data));
// MUX: data to be written back to registers
assign dst = MemToReg ? rd_data : ALUResult;


/////////////////////////
// Pipeline SHIT      //
///////////////////////

// **************************************************************************************
//TODO: need to pass through Branch, ALUOp, ALUSrc, PCSrc still.
//		follow MemToReg as a reference
//TODO: also need to connect the block outputs to the correct modules above
// **************************************************************************************

  // IM block input 
  //input [15:0] instr;

  // global clock and reset for DFFs
  //input clk, rst_n;

  // Mem block outputs
  wire DM_re_ID_EX_DM, DM_we_ID_EX_DM;

  // ID block wires
  wire [15:0] instr_IM_ID;
  wire DM_re, DM_we, RF_we;
  wire [3:0] RF_dst_addr;
  
  // EX block wires
  wire DM_re_ID_EX, DM_we_ID_EX;
  wire RF_we_ID_EX;
  wire WB_MemToReg_ED_EX;
  wire [3:0] RF_dst_addr_ID_EX;

  // MEM block
  wire RF_we_EX_DM;
  wire WB_MemToReg_EX_DM;
  wire [3:0] RF_dst_addr_EX_DM;
  
  // WB block outputs
  wire RF_we_DM_WB, WB_MemToReg_DM_WB;
  wire [3:0] RF_dst_addr_DM_WB;

  ////**** d -> [FLOP] -> q ****////
  
  // IF/ID BLOCK
  flop16b ID_flop(.q(instr_IM_ID), .d(instruction), .clk(clk), .rst_n(rst_n));
  //block_ID ID_block(DM_re, DM_we, RF_we, RF_dst_addr, instr_IM_ID);
  
  // Controller //
   controller ctrl(.OpCode(instr_IM_ID[15:12]), .RegDst(RegDst), .Branch(Branch), 
   .MemRead(DM_re), .MemToReg(MemToReg), .ALUOp(ALUOp), .MemWrite(DM_we),
   .ALUSrc(ALUSrc), .RegWrite(RF_we), .PCSrc(PCSrc));

   // RegDst decides which value this will be
   assign RF_dst_addr = dst_addr;
   

  // ID/EX BLOCK
  flop2b ex_2_DM_re_ID_EX({DM_re_ID_EX,DM_we_ID_EX}, {DM_re,DM_we}, clk, rst_n);
  flop1b ex_1_RF_we_ID_EX(RF_we_ID_EX, RF_we, clk, rst_n);
  flop1b ex_1_WB_MemToReg(WB_MemToReg_ED_EX, MemToReg, clk, rst_n);
  flop4b ex_4(RF_dst_addr_ID_EX, RF_dst_addr, clk, rst_n);

  // EX/MEM BLOCK
  flop2b mem_2({DM_re_ID_EX_DM,DM_we_ID_EX_DM}, {DM_re_ID_EX,DM_we_ID_EX}, clk, rst_n);
  flop1b mem_1(RF_we_EX_DM, RF_we_ID_EX, clk, rst_n);
  flop1b mem_1_WB_MemToReg(WB_MemToReg_EX_DM, WB_MemToReg_ED_EX, clk, rst_n);
  flop4b mem_4(RF_dst_addr_EX_DM, RF_dst_addr_ID_EX, clk, rst_n);

  // MEM/WB BLOCK
  flop1b wb_1(RF_we_DM_WB, RF_we_EX_DM, clk, rst_n);
  flop1b wb_1_WB_MemToReg(WB_MemToReg_DM_WB, WB_MemToReg_EX_DM, clk, rst_n);
  flop4b wb_4(RF_dst_addr_DM_WB, RF_dst_addr_EX_DM, clk, rst_n);



endmodule
module cpu(clk, rst_n, hlt, pc);

////////////////////////////////
// initialize inputs/outputs //
//////////////////////////////
input clk, rst_n;
output [15:0] pc;
output hlt;

///////////////////////
// Initialize Wires //
/////////////////////

// PC //
wire [15:0] nextAddr; 
wire [15:0] programCounter, PCaddOut, PCNext;
wire [15:0] pcInc;			// to mediate between PC and nextAddr

// Instruction Memory //
wire reg [15:0] instruction;
wire rd_en;			 		// asserted when instruction read desired

// Register Memory //
wire [15:0] readData1, readData2;
wire re0, re1;				// read enables (power not functionality)
wire [3:0] dst_addr, readReg1, readReg2;	// write address (reg)
wire [15:0] dst;			// data to be written to register file, write data (dst bus)
wire we;					// write enable

// ALU //
wire [15:0] ALUResult;
wire ov, zr, neg;
wire [3:0] shamt;
assign shamt = instruction[3:0];
wire [15:0] src2Wire;

// Branch
wire Yes;					// Yes is 1 if branch condition is satisfied

// Sign Extension
wire [15:0] signOutALU, signOutJump, signOutBranch, signOutMem, signOutBJ;

// Flag Register
wire zrOut, negOut, ovOut, change_z, change_v, change_n;

// Data Memory //
wire [15:0] rd_data;		//output of data memory -> register write data

// Controller //
wire 	RegDst,		// 1: write back instr[11:8] to register, 0: (sw don't care) lw into register @ instr[3:0]
		Branch, 	// 1: take branch, 0: don't take branch
		MemRead, 	// 1: enable read memory, 0: disable memory read
		MemToReg, 	// 1: write data from memory to registers, 0: write data from ALU to registers
		MemWrite,	// 1: enable memory write, 0: disable memory write
		ALUSrc, 	// 0: readData2 -> ALU 1: sign-extended -> ALU
		RegWrite,	// 1: enable right back to register, 0: don't write back to register
		PCSrc,		// 0: take next pc addr, 1: enable branch (sign-ext)
		LoadHigh,	// 1: take in [11:8] as read data 1, 0: take normal read data 1
		JumpR,		// 1: if taking Jump register
		JumpAL,		// 1: if taking Jump and link
		StoreWord;	// 1: enable [11:8] -> read register for SW, 0: otherwise


/////////////////////////
// Initialize Modules //
///////////////////////

////////////************ Display ************//////////////
always @(posedge clk) begin
	// let's see what's going on!
	$display("programCounter=%d\n instruction#=%b\n readReg1=%d\n readReg2=%d\n readData1->ALU=%h\n src2Wire->ALU=%h\n ALUResult=%h -> dst_addrWriteReg=%d\n dst_RegWrite=%h\n readData2==dstWriteData=%h\n ALUSrc=%b\n Branch=%b, Yes=%b, PCSrc=%b\n nextAddr=%d\n negOut=%b, ovOut=%b, zrOut=%b\n signOutBranch=%d\n pcInc=%d\n signOutMem=%h", 
	      programCounter,     instruction,      readReg1,     readReg2,     readData1,          src2Wire,          ALUResult,      dst_addr,             dst,            readData2,                   ALUSrc,     Branch,    Yes,    PCSrc,     nextAddr,      negOut,   ovOut,    zrOut,    signOutBranch,     pcInc, signOutMem);
	$display("***************************\nRegDst=%b, Branch=%b, MemRead=%b, MemToReg=%b, MemWrite=%b, ALUSrc=%b, 
   RegWrite=%b, LoadHigh=%b, JumpR=%b, JumpAL=%b StoreWord=%b\n***************************\n\n", RegDst, Branch, MemRead, MemToReg, MemWrite,
   ALUSrc, RegWrite, LoadHigh, JumpR, JumpAL, StoreWord);
end


////////////************ PC ************//////////////
PC pcMod(.nextAddr(nextAddr), 
		.clk(clk), 
		.rst(rst_n), 
		.programCounter(programCounter));
assign pcInc = programCounter + 1;
// Adds Branch/Jump signed address to pcInc +1
alu2 PCAdd(PCaddOut, pcInc, signOutBJ);		
// Mux to choose which signed address to add to pcInc
assign signOutBJ = JumpAL ? signOutJump : signOutBranch;	
// Mux to choose what next addr should be. Chooses between branch, jump, jump and link, halt,  or PCSrc
assign nextAddr = JumpAL ? PCaddOut : (JumpR ? readData1 : (hlt ? programCounter : (PCSrc ? PCaddOut : pcInc)));	//PCNext;
// Sign-extenders 
sign_extenderALU signExtenALU(instruction[7:0], signOutALU);
sign_extenderJump signExtenJUMP(instruction[11:0], signOutJump);
sign_extenderBranch signExtenBranch(instruction[8:0], signOutBranch);
sign_extenderMem signExtenMem(instruction[3:0], signOutMem);
// Branch 
branch_met BranchPred(.Yes(Yes), .ccc(instruction[11:9]), .N(negOut), .V(ovOut), .Z(zrOut), .clk(clk));
Take conditional branch if condition is met (Yes) and it is a branch instruction
assign PCSrc = (Yes && Branch);
// Flags (should reflect for the next cycle
flags flagReg(.zr(zrOut), .neg(negOut), .ov(ovOut), .change_z(change_z), .change_n(change_n), 
.change_v(change_v), .z_in(zr), .n_in(neg), .v_in(ov), .rst_n(rst_n), .clk(clk));


////////////************ Instruction Memory ************//////////////
IM instMem(.clk(clk), 
		   .addr(programCounter), 
		   .rd_en(rd_en), 
		   .instr(instruction));
assign rd_en = 1'b1;


////////////************ Registers ************//////////////
rfSC registers(.clk(clk), 
			   .p0_addr(readReg1), 
			   .p1_addr(readReg2),
			   .p0(readData1), 
			   .p1(readData2), 
			   .re0(re0), 
			   .re1(re1),
			   .dst_addr(dst_addr), 
			   .dst(dst), 
			   .we(RegWrite), 
			   .hlt(hlt));
// Power not functionality
assign re0 = 1'b1;
assign re1 = 1'b1;
// MUX for choosing which registers addr to load from register memory
assign readReg1 = LoadHigh ? instruction[11:8] : instruction[7:4]; 
assign readReg2 = StoreWord ? instruction[11:8] : instruction[3:0];
// MUX for write register addr. JumpAL stores in R13, RegDst controls addr
assign dst_addr = JumpAL ? 4'b1111 : (RegDst ? instruction[11:8] : instruction[3:0]);


////////////************ ALU ************//////////////
ALU alu(.src0(readData1), 
		.src1(src2Wire), 
		.op(instruction[15:12]), 
		.dst(ALUResult), 
		.ov(ov), 
		.zr(zr), 
		.neg(neg), 
		.shamt(shamt), 
		.change_v(change_v),
		.change_z(change_z), 
		.change_n(change_n));
// MUX: lw/sw instruction use the sign-extended value for src1 input
assign src2Wire = StoreWord ? signOutMem : (ALUSrc ? signOutALU : readData2);



////////////************ Data Memory ************//////////////
DM dataMem(.clk(clk), 
		   .addr(ALUResult), 
		   .re(MemRead), 
		   .we(MemWrite), 
		   .wrt_data(readData2), 
		   .rd_data(rd_data));
// MUX: data to be written back to registers. If JumpAL, store pc +1, otherwise store ALUResult or mem
assign dst = JumpAL ? pcInc : (MemToReg ? rd_data : ALUResult);


////////////************ Controller ************//////////////
controller ctrl(.OpCode(instruction[15:12]), 
				.RegDst(RegDst), 
				.Branch(Branch), 
				.MemRead(MemRead), 
				.MemToReg(MemToReg), 
				.MemWrite(MemWrite),
				.ALUSrc(ALUSrc), 
				.RegWrite(RegWrite), 
				.rst_n(rst_n), 
				.LoadHigh(LoadHigh), 
				.Halt(hlt),
				.StoreWord(StoreWord), 
				.JumpAL(JumpAL), 
				.JumpR(JumpR));




/////////////////////////
// Pipeline SHIT      //
///////////////////////

// **************************************************************************************
//TODO: need to pass through Branch, ALUOp, ALUSrc, PCSrc still.
//		follow MemToReg as a reference
//TODO: also need to connect the block outputs to the correct modules above
// **************************************************************************************

  // IF block input 
  //input [15:0] instr;

  // global clock and reset for DFFs
  //input clk, rst_n;

  // Mem block outputs
  wire DM_re_ID_EX_DM, DM_we_ID_EX_DM;

  // ID block wires
  wire [15:0] instr_IF_ID;
  wire DM_re, DM_we, RF_we;
  wire [3:0] RF_dst_addr;
  
  // EX block wires
  wire DM_re_ID_EX, DM_we_ID_EX;
  wire RF_we_ID_EX;
  wire DM_MemToReg_ED_EX;
  wire [3:0] RF_dst_addr_ID_EX;

  // MEM block
  wire RF_we_EX_DM;
  wire DM_MemToReg_EX_DM;
  wire [3:0] RF_dst_addr_EX_DM;
  
  // WB block outputs
  wire RF_we_DM_WB, DM_MemToReg_DM_WB;
  wire [3:0] RF_dst_addr_DM_WB;

  ////**** d -> [FLOP] -> q ****////
  
  // IF/ID BLOCK
  flop16b ID_flop(.q(instr_IF_ID), .d(instruction), .clk(clk), .rst_n(rst_n));
  flop16b f16_EX_pcInc_IF_ID(EX_pcInc_IF_ID, pcInc, clk, rst_n);
  flop16b 
  
  //block_ID ID_block(DM_re, DM_we, RF_we, RF_dst_addr, instr_IF_ID);
  
  // NEW Controller //
   controller ctrl(.OpCode(instr_IF_ID[15:12]), // ID
				   .RegDst(RegDst), 			// ID
				   .LoadHigh(LoadHigh), 		// ID
				   .StoreWord(StoreWord), 		// ID
				   .JumpAL(JumpAL), 			// EX
				   .ALUSrc(ALUSrc),				// EX
				   .Branch(Branch), 			// DM
				   .MemRead(MemRead), 			// DM
				   .MemWrite(MemWrite),			// DM
				   .Halt(hlt),					// DM
				   .JumpR(JumpR)				// DM
				   .MemToReg(MemToReg), 		// WB
				   .RegWrite(RegWrite), 		// WB
				   .rst_n(rst_n));				
   
   // OLD CONTROLLER //
   controller ctrl(.OpCode(instruction[15:12]), 
				.RegDst(RegDst), 
				.Branch(Branch), 
				.MemRead(MemRead), 
				.MemToReg(MemToReg), 
				.MemWrite(MemWrite),
				.ALUSrc(ALUSrc), 
				.RegWrite(RegWrite), 
				.rst_n(rst_n), 
				.LoadHigh(LoadHigh), 
				.Halt(hlt),
				.StoreWord(StoreWord), 
				.JumpAL(JumpAL), 
				.JumpR(JumpR));
				
   // RegDst decides which value this will be
   //assign RF_dst_addr = dst_addr;
  // Example Format:
  //     Destination_Signal_From_To(Output, Input, clk, rst_n);
  // ID/EX BLOCK //
  flop1b f1_EX_ALUSrc_ID_EX(EX_ALUSrc_ID_EX, ALUSrc, clk, rst_n);// ALUSrc
  flop16b f16_EX_signOutBranch_ID_EX(EX_signOutBranch_ID_EX, signOutBranch, clk, rst_n);
  flop16b f16_EX_signOutALU_ID_EX(EX_signOutALU_ID_EX, signOutALU, clk, rst_n);
  flop16b f16_EX_signOutMem_ID_EX(EX_signOutMem_ID_EX, signOutMem, clk, rst_n);
  flop16b f16_EX_signOutJump_ID_EX(EX_signOutJump_ID_EX, signOutJump, clk, rst_n); // Signextended value
  flop4b f4_EX_shamt_ID_EX(EX_shamt_ID_EX, instr_IF_ID[3:0], clk, rst_n);
  flop16b f16_EX_readData1_ID_EX(EX_readData1_ID_EX, readData1, clk, rst_n);// readData1
  flop4b f4_EX_opCode_ID_EX(EX_opCode_ID_EX, instr_IF_ID[15:12], clk, rst_n);
  flop1b f1_DM_Branch_ID_EX(DM_Branch_ID_EX, Branch, clk, rst_n); // Branch
  flop1b f1_DM_MemRead_ID_EX(DM_MemRead_ID_EX, MemRead, clk, rst_n);	// MemRead
  flop1b f1_DM_MemWrite_ID_EX(DM_MemWrite_ID_EX, MemWrite, clk, rst_n);		// MemWrite
  flop1b f1_DM_Halt_ID_EX(DM_Halt_ID_EX, Halt, clk, rst_n); // Halt
  flop1b f1_DM_JumpR_ID_EX(DM_JumpR_ID_EX, JumpR, clk, rst_n); // JumpR
  flop1b f1_DM_JumpAL_ID_EX(DM_JumpAL_ID_EX, JumpAL, clk, rst_n);
  flop1b f1_DM_MemToReg(DM_MemToReg_ID_EX, MemToReg, clk, rst_n);		// MemToReg
  flop16b f16_DM_readData2_ID_EX(DM_readData2_ID_EX, readData2, clk, rst_n); // readData2
  flop16b f16_DM_pcInc_ID_EX(DM_pcInc_ID_EX, DM_pcInc_IF_ID, clk, rst_n);
  flop4b f4_DM_ccc_ID_EX(DM_ccc_ID_EX, {1'b0, instr_IF_ID[11:9]}, clk, rst_n);
  flop1b f1_WB_RegWrite_ID_EX(WB_RegWrite_ID_EX, RF_we, clk, rst_n);			// RegWrite
  flop4b f4_WB_dst_addr_ID_EX(WB_dst_addr_ID_EX, dst_addr, clk, rst_n);		// Register Write Addr (dst_addr)
  
  // EX/DM BLOCK //
  flop1b f1_DM_Branch_EX_DM(DM_Branch_EX_DM, DM_Branch_ID_EX, clk, rst_n); // Branch
  flop1b f1_DM_MemRead_EX_DM(DM_MemRead_EX_DM, DM_MemRead_ID_EX, clk, rst_n);	// MemRead
  flop1b f1_DM_MemWrite_EX_DM(DM_MemWrite_EX_DM, DM_MemWrite_ID_EX, clk, rst_n);	// MemWrite
  flop1b f1_DM_Halt_EX_DM(DM_Halt_EX_DM, DM_Halt_ID_EX, clk, rst_n); // Halt
  flop1b f1_DM_JumpR_EX_DM(DM_JumpR_EX_DM, DM_JumpR_ID_EX, clk, rst_n); // JumpR
  flop1b f1_DM_JumpAL_EX_DM(DM_JumpAL_EX_DM, DM_JumpAL_ID_EX, clk, rst_n); // JumpAL
  flop1b f1_DM_MemToReg_EX_DM(DM_MemToReg_EX_DM, DM_MemToReg_ID_EX, clk, rst_n); // MemToReg
  flop16b f16_DM_readData2_EX_DM(DM_readData2_EX_DM, DM_readData2_ID_EX, clk, rst_n); // readData2
  flop16b f16_DM_ALUResult_EX_DM(DM_ALUResult_EX_DM, ALUResult, clk, rst_n);
  flop16b f16_DM_pcInc_EX_DM(DM_pcInc_EX_DM, DM_pcInc_ID_EX, clk, rst_n);
  flop16b f16_DM_pcAddOut_EX_DM(DM_pcAddOut_EX_DM, pcAddOut, clk, rst_n);
  flop1b f1_DM_zrOut_EX_DM(DM_zrOut_EX_DM, zrOut, clk, rst_n);
  flop1b f1_DM_negOut_EX_DM(DM_negOut_EX_DM, negOut, clk, rst_n);
  flop1b f1_DM_ovOut_EX_DM(DM_ovOut_EX_DM, ovOut, clk, rst_n);
  flop4b f4_DM_ccc_EX_DM(DM_ccc_EX_DM, DM_ccc_ID_EX, clk, rst_n);
  flop1b f1_WB_RegWrite_EX_DM(WB_RegWrite_EX_DM, WB_RegWrite_ID_EX, clk, rst_n); // RegWrite
  flop4b f4_WB_dst_addr_EX_DM(WB_dst_addr_EX_DM, WB_dst_addr_ID_EX, clk, rst_n); // dst_addr

  // DM/WB BLOCK //
  flop1b f1_WB_RegWrite_DM_WB(WB_RegWrite_DM_WB, WB_RegWrite_EX_DM, clk, rst_n); // RegWrite
  flop4b f4_WB_dst_addr_DM_WB(WB_dst_addr_DM_WB, WB_dst_addr_EX_DM, clk, rst_n); // dst_addr
  flop16b f16_WB_dst_DM_WB(WB_dst_DM_WB, dst, clk, rst_n); // data to be written to register

endmodule
module cpu(clk, rst_n, hlt, pc);

/// initialize inputs/outputs ///
input clk, rst_n;
output [15:0] pc;
output hlt;


///////////////////////
// Initialize Wires //
/////////////////////

// PC //
reg [15:0] nextAddr; 
wire [15:0] programCounter, PCaddOut, PCNext;
reg [15:0] pcInc;	// to mediate between PC and nextAddr

// Instruction Memory //
wire reg [15:0] instruction;
wire rd_en;			 // asserted when instruction read desired

// Register Memory //
wire [15:0] readData1, readData2;
wire re0, re1;	// read enables (power not functionality)
wire [3:0] dst_addr, readReg1;	// write address (reg)
wire [15:0] dst;	// data to be written to register file, write data (dst bus)
wire we;			// write enable
wire hlt;			// not a functional input.  Used to dump register contents when test is halted.
//^^ hlt is wire AND output?	

// ALU //
wire [15:0] ALUResult;
wire ov, zr, neg;
wire [3:0] shamt;
assign shamt = instruction[3:0];
wire [15:0] src2Wire;

// Branch
wire Yes;

// Sign Extension
wire [15:0] signOutALU, signOutJump, signOutBranch;

// Flag Register
wire zrOut, negOut, ovOut, change_z, change_v, change_n;

// Data Memory //
//wire re;	// asserted when instruction read desired
//wire we;	// asserted when write desired
wire [15:0] rd_data;	//output of data memory -> register write data

// Controller //
wire 	RegDst,		// 1: write back instr[11:8] to register, 0: (sw don't care) lw into register @ instr[3:0]
		Branch, 	// 1: take branch, 0: don't take branch
		MemRead, 	// 1: enable read memory, 0: disable memory read
		MemToReg, 	// 1: write data from memory to registers, 0: write data from ALU to registers
		//ALUOp, 		// Can be used to combine LW/SW with ADD, or branch instructions (we are currently not using this?)
		MemWrite,	// 1: enable memory write, 0: disable memory write
		ALUSrc, 	// 0: readData2 -> ALU 1: sign-extended -> ALU
		RegWrite,	// 1: enable right back to register, 0: don't write back to register
		PCSrc,		// 0: take next pc addr, 1: enable branch (sign-ext)
		// PCSrc is not connected to the control
		LoadHigh,	// 1: take in [11:8] as read data 1, 0: take normal read data 1
		Jump;


/////////////////////////
// Initialize Modules //
///////////////////////

// PC //
PC pcMod(.nextAddr(nextAddr), .clk(clk), .rst(rst_n), .programCounter(programCounter));

// Program Counter
always @(posedge clk) begin
	if (~rst_n) begin
	pcInc <= 16'h0000;
	//end
	//else if (PCSrc) begin
	// not sure if this would work...
	//pcInc <= (pcInc + 1 + signOutBranch); //<< 2);
	//nextAddr <= pcInc;
	//end else if (Jump) begin
	//pcInc <= (pcInc + 1 + signOutJump); //<< 2);
	//nextAddr <= pcInc;
	end else begin
	pcInc <= programCounter + 1;
	nextAddr <= PCNext;
	end
	// let's see what's going on!
	$display("programCounter=%d, instruction=%b, readData1=%b,
	src2Wire=%b, ALUResult=%b, dst_addr=%b, dst=%b, ALUSrc=%b, Branch=%b, Yes=%b, PCSrc=%b, nextAddr=%b,
	negOut=%b, ovOut=%b, zrOut=%b, signOutBranch=%b, pcInc=%b\n", programCounter, instruction,
	readData1, src2Wire, ALUResult, dst_addr, dst, ALUSrc, Branch, Yes, PCSrc, nextAddr,
	negOut, ovOut, zrOut, signOutBranch, pcInc);
	//$display("programCounter=%d, ALUResult=%b, dst_addr=%b, dst=%b \n", programCounter, ALUResult,
	//dst_addr, dst);
	
end

// PC Adder
alu2 PCAdd(PCaddOut, pcInc, signOutBranch);
assign PCNext = PCSrc ? PCaddOut : pcInc;


// Sign-extender
sign_extenderALU signExtenALU(instruction[7:0], signOutALU);
sign_extenderJump signExtenJUMP(instruction[11:0], signOutJump);
sign_extenderBranch signExtenBranch(instruction[8:0], signOutBranch);

// Branch 
branch_met BranchPred(.Yes(Yes), .ccc(instruction[11:9]), .N(negOut), .V(ovOut), .Z(zrOut));
assign PCSrc = (Yes && Branch);

// Flags
flags flagReg(.zr(zrOut), .neg(negOut), .ov(ovOut), .change_z(change_z), .change_n(change_n), 
.change_v(change_v), .z_in(zr), .n_in(neg), .v_in(ov));

// Instruction Memory //
IM instMem(.clk(clk), .addr(programCounter), .rd_en(rd_en), .instr(instruction));
assign rd_en = 1'b1;


// Registers //
// po,p1 are 'output reg' in rfSC
rfSC registers(.clk(clk), .p0_addr(readReg1), .p1_addr(instruction[3:0]),
				.p0(readData1), .p1(readData2), .re0(re0), .re1(re1),
				.dst_addr(dst_addr), .dst(dst), .we(RegWrite), .hlt(hlt));
// Power not functionality?
assign re0 = 1'b1;
assign re1 = 1'b1;
// MUX: Write Register MUX. Changes for lw
assign dst_addr = RegDst ? instruction[11:8] : instruction[3:0];
assign readReg1 = LoadHigh ? instruction[11:8] : instruction[7:4]; 



// ALU //
ALU alu(.src0(readData1), .src1(src2Wire), .op(instruction[15:12]), 
.dst(ALUResult), .ov(ov), .zr(zr), .neg(neg), .shamt(shamt), .change_v(change_v),
.change_z(change_z), .change_n(change_n));
// MUX: lw/sw instruction use the sign-extended value for src1 input
assign src2Wire = ALUSrc ? signOutALU : readData2;



// Data Memory //
DM dataMem(.clk(clk), .addr(ALUResult), .re(MemRead), .we(MemWrite), 
		   .wrt_data(readData2), .rd_data(rd_data));
// MUX: data to be written back to registers
assign dst = MemToReg ? rd_data : ALUResult;


// Controller //
controller ctrl(.OpCode(instruction[15:12]), .RegDst(RegDst), .Branch(Branch), 
.MemRead(MemRead), .MemToReg(MemToReg), .MemWrite(MemWrite),
.ALUSrc(ALUSrc), .RegWrite(RegWrite), .rst_n(rst_n), .LoadHigh(LoadHigh), .Jump(Jump));


/////////////////////////
// Pipeline SHIT      //
///////////////////////

// Instruction Memory Read //



// Register Read //



// Execute //



// Read/Write Data Memory //



// Write Back to Registers //




endmodule

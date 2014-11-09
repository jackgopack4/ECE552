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
		RegWrite;	// 1: enable right back to register, 0: don't write back to register


/////////////////////////
// Initialize Modules //
///////////////////////

// PC //
PC pcMod(.nextAddr(nextAddr), .clk(clk), .rst(rst_n), .programCounter(programCounter));

always @(posedge clk) begin
	pcInc <= programCounter + 1;
	nextAddr <= pcInc;
	// let's see what's going on!
	$display("programCounter=%d, rd_en=%b, instruction=%b, instruction[7:4]=%b, readData1=%b,
	readData2=%b, instruction[3:0]=%b, src1Wire=%b, ALUResult=%b", programCounter, rd_en, instruction, instruction[7:4], 
	readData1, readData2, instruction[3:0], src1Wire, ALUResult);
end

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



// Data Memory //
DM dataMem(.clk(clk), .addr(ALUResult), .re(MemRead), .we(MemWrite), 
		   .wrt_data(readData2), .rd_data(rd_data));
// MUX: data to be written back to registers
assign dst = MemToReg ? rd_data : ALUResult;


// Controller //
controller ctrl(.OpCode(instruction[15:12]), .RegDst(RegDst), .Branch(Branch), 
.MemRead(MemRead), .MemToReg(MemToReg), .ALUOp(ALUOp), .MemWrite(MemWrite),
.ALUSrc(ALUSrc), .RegWrite(RegWrite));


/////////////////////////
// Pipeline SHIT      //
///////////////////////

// Instruction Memory Read //



// Register Read //



// Execute //



// Read/Write Data Memory //



// Write Back to Registers //

































endmodule

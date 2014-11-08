module cpu(clk, rst_n, hlt, pc);

/// initialize inputs/outputs ///
input clk, rst_n;
output [15:0] pc;
output hlt;


///////////////////////
// Initialize Wires //
/////////////////////

// PC //
wire [15:0] nextAddr, programCounter;

// Instruction Memory //
wire [15:0] instruction;
wire rd_en;			 // asserted when instruction read desired

// Registers //
wire [15:0] readData1, readData2;
wire re0, re1;	// read enables (power not functionality)
reg [15:0] dst_addr;	// write address
wire [15:0] dst;	// dst bus
wire we;			// write enable
wire hlt;			// not a functional input.  Used to dump register contents when test is halted.
//^^ hlt is wire AND output?	

// ALU //
wire [15:0] ALUResult;
wire ov, zr, neg;

// Data Memory //
wire re;	// asserted when instruction read desired
//wire we;	// asserted when write desired
reg [15:0] rd_data;	//output of data memory

/////////////////////////
// Initialize Modules //
///////////////////////

// PC //
PC pcMod(.nextAddr(nextAddr), .clk(clk), .rst(rst), .programCounter(programCounter));

// Instruction Memory //
IM instMem(.clk(clk), .addr(programCounter), .rd_en(rd_en), .instr(instruction));		

// Registers //
// po,p1 are 'output reg' in rfSC
rfSC registers(.clk(clk), .p0_addr(instruction[7:4]), .p1_addr(instruction[3:0]),
				.p0(readData1), .p1(readData2), .re0(re0), .re1(re1),
				.dst_addr(dst_addr), .dst(dst), .we(we), .hlt(hlt));

// ALU //
ALU alu(.src0(readData1), .src1(readData2), .func(instruction[5:0]), 
		.dst(ALUResult), .ov(ov), .zr(zr), .neg(neg));

// Data Memory //
DM dataMem(.clk(clk), .addr(ALUResult), .re(re), .we(we), 
		   .wrt_data(readData2), .rd_data(rd_data));




// Instruction Memory Read //



// Register Read //



// Execute //



// Read/Write Data Memory //



// Write Back to Registers //

































endmodule

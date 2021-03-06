`timescale 1ns / 10ps
module stimulus(
);

localparam N = 16;

localparam FLAGS_COUNT = 4;
localparam POSITIVE = 3;
localparam OVERFLOW = 2;
localparam CARRY    = 1;
localparam ZERO     = 0;

localparam ALU_OP_COUNT = 4;
localparam ALU_ADD = 1;
localparam ALU_SUB = 2;
localparam ALU_MUL = 3;
localparam ALU_DIV = 4;
localparam ALU_AND = 5;
localparam ALU_OR  = 6;
localparam ALU_XOR = 7;
localparam ALU_NEG = 8;
localparam ALU_SHL = 9;
localparam ALU_SHR = 10;

reg [N-1:0] a, b;
wire [N-1:0] high, result;
wire [FLAGS_COUNT-1:0] flags;
reg [3:0]opcode;
wire finished;
reg CLK;
reg start;

ALU #(.N(16))alu(
	// clock
	CLK, 
	// first operand
	a,
	// second operand
	b,
	// operation code (see localparams below)
	opcode,
	// result
	result,
	// flags 
	flags,
	// upper bits of MUL, or remainder of DIV
	high,
	// if high, start multiplication/division; otherwise must be low
	start,
	// goes high when multiplication/division is finished
	finished
);


integer i;

always begin
  #10 CLK = ~CLK;
	#10 $display("result: %2x, finished: %x", result, finished);

end
 
initial begin
	$dumpfile("test.vcd");
	$dumpvars(0, stimulus);
	// Initialize Inputs
	CLK = 0;
	#10 a = 5; b = 6; opcode = ALU_DIV; start = 1;
	$monitor("a:%d, b: %d, res: %d", a, b, result, finished);
/*		
	for (i = 0; i < 325; i = i + 1) begin
		#10 CLK = !CLK; 
		#10 $display("result: %2x", result);
		if (i == 11) begin
			irq[0] <= 1'b1;
			$display("$$$$$$$$$$$$$$$$$ SENDING IRQ $$$$$$$$$$$$$$$$$$$$$$");
		end
		else if (i == 13) begin 
			irq[0] <= 0;
		end
	end
*/		
  #500 $stop;
end  



endmodule
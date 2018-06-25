
module ALU#(
	parameter N = 5'd16,
	ALU_OP_COUNT = 4,
	FLAGS_COUNT = 4
	)(
	input wire CLK, 
	// first operand
	input wire [N-1:0]a,
	// second operand
	input wire [N-1:0]b,
	// operation code (see localparams below)
	input wire [ALU_OP_COUNT-1:0] opcode,
	// result
	output reg [N-1:0]result,
	// flags 
	output reg [FLAGS_COUNT-1:0] flags,
	// upper bits of MUL, or remainder of DIV
	output reg [N-1:0] high,
	// if high, start multiplication/division; otherwise must be low
	input wire start,
	// goes high when multiplication/division is finished
	output reg finished
);

localparam POSITIVE = 3;
localparam OVERFLOW = 2;
localparam CARRY    = 1;
localparam ZERO     = 0;

localparam ALU_ADD = 4'd1;
localparam ALU_SUB = 4'd2;
localparam ALU_MUL = 4'd3;
localparam ALU_DIV = 4'd4;
localparam ALU_AND = 4'd5;
localparam ALU_OR  = 4'd6;
localparam ALU_XOR = 4'd7;
localparam ALU_NEG = 4'd8;
localparam ALU_SHL = 4'd9;
localparam ALU_SHR = 4'd10;

reg [N*2-1:0] tmp;
wire [N-1:0] dres, dhigh, dcount;
wire dfinished, dstart;


div #(.N(N))div1 (
	CLK,
	a, b, 
	dres, dhigh, 
	start,
	dfinished
);

always @(negedge CLK) begin
	case (opcode)
		ALU_ADD: begin 
			{flags[CARRY], tmp[N-1:0]} = a + b; 
		end
		ALU_SUB: begin 
			{flags[CARRY], tmp[N-1:0]} = a - b; 
		end
		ALU_MUL: begin 
			{high, tmp[N-1:0]} = a * b;
			finished <= 1'b1;
		end
		ALU_DIV: begin 
			/*
			if (start) begin
				tmp[N-1:0] = a / b;
				high = a % b;
				finished <= 1'b0;
			end
			else if (!start && !finished) begin
				finished <= 1'b1;
			end
			else begin
				finished <= 1'b0;
			end
			*/
			
			if (!start && dfinished) begin
				tmp[N-1:0] = dres;
				high = dhigh;
				finished <= 1'b1; 
			end
			else begin
			   finished <= 1'b0; 
			end
			
		end
		ALU_AND: tmp = a & b;
		ALU_OR : tmp = a | b;
		ALU_XOR: tmp = a ^ b;
		ALU_NEG: tmp = ~a;
		ALU_SHL: tmp = a << b;
		ALU_SHR: tmp = a >> b;
	endcase
	
	// ZERO
	flags[ZERO] <= tmp == 0;
	
	// OVERFLOW
	flags[OVERFLOW] <= (opcode == ALU_ADD) && (a[N-1] == b[N-1]) && (tmp[N-1] != a[N-1]);
	
	// POSITIVE
	flags[POSITIVE] <= !tmp[N-1];

	result <= tmp[N-1:0];

	//$display("[[[[flagsPOCZ=%2b, res=%4x, h=%4x, d=%4x, finished=%x]]]]", flags, tmp, dhigh, dres, dfinished);
end


endmodule

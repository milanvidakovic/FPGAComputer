
module cpu#(parameter N = 5'd16)(

//////////// CLOCK //////////
	input	CLOCK_50,
	
	input reset,  // reset signal

	input [N-1:0] data,
	output [N-1:0] data_to_write,
	output [N-1:0] addr,
	output memrd, 
	output memwr, 
	input [7:0] irq,
	output [7:0] LED,
	input [7:0] rx_data,     // UART RECEIVED BYTE 
	output tx_send,          // UART TX send signal
	input tx_busy,           // UART TX busy signal
	output [7:0] tx_data     // UART TX data
);


//=======================================================
//  PARAMETER declarations
//=======================================================
localparam SP = 8;
localparam H  = 9;


localparam FLAGS_COUNT = 4;
localparam POSITIVE = 3;
localparam OVERFLOW = 2;
localparam CARRY    = 1;
localparam ZERO     = 0;

localparam ALU_OP_COUNT = 4;
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
localparam DIV_CYCLES = 3;


//=======================================================
//  REG/WIRE declarations
//=======================================================

reg [N-1:0] regs[9:0];  // 0-7 are r0-r7; 8 is sp, 9 is high
reg [N-1:0] pc, mbr, ir;
reg [N-1:0] mc_count, irq_count;
reg [7:0] irq_r;
reg [7:0] noirq;
reg [7:0] rx_data_r;

//=======================================================
//  misc. declarations
//=======================================================

// alu operation
reg [FLAGS_COUNT-1:0] f, f_from_alu;
reg [ALU_OP_COUNT-1:0] alu_op;
wire [N-1:0] alu_a, alu_b, alu_res, alu_high;
wire div_finished;
reg start_div;
reg [3:0] div_counter;

//=======================================================
//  Structural coding
//=======================================================



reg halt;

ALU #(.N(N))alu(
	// clock
	CLOCK_50, 
	// first operand
	alu_a,
	// second operand
	alu_b,
	// operation code (see localparams below)
	alu_op,
	// result
	alu_res,
	// flags 
	f_from_alu,
	// upper bits of MUL, or remainder of DIV
	alu_high,
	// if high, start multiplication/division; otherwise must be low
	start_div,
	// goes high when multiplication/division is finished
	div_finished
);

always @ (posedge CLOCK_50) begin
	if (reset) begin
		// RESET
		LED[7:0] <= 0;
		halt <= 0;
		pc <= 0;
		mbr <= 0;
		f <= 0;
		// reset registers
		regs[0] <= 0;
		regs[1] <= 0;
		regs[2] <= 0;
		regs[3] <= 0;
		regs[4] <= 0;
		regs[5] <= 0;
		regs[6] <= 0;
		regs[7] <= 0;
		regs[SP] <= 0; // sp
		regs[H] <= 0; // high
		// IR stuff
		ir <= 0;
		mc_count <= 0;
		// ALU stuff
		alu_op <= 0;
		// bus stuff
		// we will start reading from the PC immediately
		memrd <= 1;
		addr <= 0;
		memwr <= 0;
		irq_r <= 0;
		irq_count <= 0;
		noirq <= 0;
	end
	else if ((~noirq & irq) && (irq_count == 0)) begin
		$display("&&&&&&&&&&&&&&&&&&&&&&&&&& IRQ HAPPENED: %d",  irq);
		LED[3:0] <= irq[3:0];
		LED[7:4] <= 0;
		if (irq[0]) begin
			noirq[0] <= 1; // mask IRQ0 off - no IRQ0
			irq_r[0] <= 1;
		end
		if (irq[1]) begin
			rx_data_r <= rx_data;
			noirq[1] <= 1; // mask IRQ1 off - no IRQ1
			irq_r[1] <= 1;
		end
		//noirq <= 255;
		irq_count <= 1;
	end
	else if (irq_count && (ir == 0)) begin
		$display("####################################################IRQ HAPPENED: %d",  irq_r);
		LED[4] <= 1;
		case (irq_count)
			1: begin
				$display("1. IRQ PUSH PC");
				// push to the stack the return value
				memwr <= 1'b1;
				memrd <= 1'b0;
				addr <= regs[SP];
				regs[SP] <= regs[SP] + 1'b1;
				// the return value is in the pc and it is already pointing to the next instruction
				data_to_write <= pc;
				irq_count <= 2;
			end
			2: begin
				$display("2. IRQ PUSH FLAGS");
				// push to the stack flags
				memwr <= 1'b1;
				memrd <= 1'b0;
				addr <= regs[SP];
				regs[SP] <= regs[SP] + 1'b1;
				// back up the flags register
				data_to_write <= f;
				irq_count <= 3;
			end
			3: begin
				$display("3. JUMP TO IRQ SERVICE #%d", irq_r);
				memwr <= 1'b0;
				memrd <= 1'b1;
				if (irq_r[0]) begin
					$display("3.1 JUMP TO IRQ #0 SERVICE");
					pc <= 16'd4;
					addr <= 16'd4;
				end 
				if (irq_r[1]) begin
					LED[7] <= 1;
					$display("3.1 JUMP TO IRQ #2 SERVICE");
					pc <= 16'd8;
					addr <= 16'd8;
				end
				irq_count <= 0;
				irq_r <= 0;
			end
		endcase
	end
//	else if (halt) begin
//	end
	else begin
		case (ir) 
			0: begin
					LED[2] <= 1;
					$display("FETCH STEP 0");
					// begin of the fetch; we have sent the memrd signal to the memory
					// in the next cycle, we will fetch the instruction from the data bus (came from the memory)
					if (mc_count == 'hff) begin
						// first power on, memory not ready yet
						memwr <= 1'b0;
						memrd <= 1'b1;
						ir <= 1;
						mc_count <= 0;
						pc <= pc + 1'b1; // pc moves to the next word (could be the operand, could be the next instruction)
						addr <= pc;
					end
					else begin
						// reset key, or normal fetch
						memwr <= 1'b0;
						memrd <= 1'b1;
						ir <= data;
						mc_count <= 0;
						pc = pc + 1'b1; // pc moves to the next word (could be the operand, could be the next instruction)
						addr <= pc;
						// at this moment, data bus holds the operand, or the next instruction
					end
				end
			1: begin
					$display("FETCH STEP 1");
					ir <= data;
					addr <= pc;
					// at this moment, data bus holds the operand, or the next instruction
				end
			default: begin
				case ({ir[3:0]}) 
					// NOP/HALT GROUP - 0
					4'b0000: begin
						if (ir[15:4] == 'hfff) begin
							// HALT
							$display("HALT");
							halt <= 1;
							pc <= pc - 1'b1;
							addr <= pc - 1'b1;
							ir <= 0;
							LED[5] <= 1;
						end
					end // end of NOP/HALT group
					
					// MOV/IN/OUT GROUP - 1
					4'b0001: begin 
						case (ir[5:4]) 
							2'b00: begin
								// MOV regx, regy
								$display("%2x: MOV r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								regs[ir[11:8]] <= regs[ir[15:12]];
								ir <= 0;     // initiate fetch
								addr <= pc;
								// pc already points to the next instruction
							end
							2'b01: begin
								// MOV reg, xx
								$display("%2x: MOV r%-d, %4d",ir[3:0], (ir[11:8]), data);
								regs[ir[11:8]] <= data;
								ir <= 0;      // initiate fetch
								pc = pc + 1'b1;  // move to the next instruction
								addr = pc;
							end
							2'b10: begin
								// IN reg, [xx]
								$display("%2x: IN r%-d, [%4d]",ir[3:0], (ir[11:8]), data);
								//LED[3] <= 1;
								case (mc_count)
									0: begin
											mbr <= data;  // remember the address of IO port
											mc_count <= 1;
											pc <= pc + 1'b1;  // move to the next instruction
									end
									1: begin
										case (mbr)
											64: begin    // UART RX DATA
												regs[ir[11:8]] <= {8'b0, rx_data_r};
											end
											65: begin   // UART TX BUSY
												regs[ir[11:8]] <= tx_busy;
											end
										endcase // end of case(mbr)
										ir <= 0;      // initiate fetch
										addr <= pc;							
									end
								endcase  // end of case (mc_count)
							end // end of IN reg, [xx]
							2'b11: begin
								// OUT [xx], reg
								$display("%2x: OUT [%4d], r%-d",ir[3:0], data, (ir[15:12]));
								LED[3] <= 1;
								case (mc_count) 
									0: begin
										pc <= pc + 1'b1;  // move to the next instruction
										
										case (data)
											66: begin  // UART TX data 
												tx_data <= regs[ir[15:12]];
												tx_send <= 1;
											end
											67: begin  // LEDs
												LED[7:0] <= regs[ir[15:12]];
											end
										endcase  // end of case (data)
										
										mc_count <= 1;
									end
									1: begin
										ir <= 0;      // initiate fetch
										addr <= pc;							
										tx_send <= 0;
									end
									endcase
							end // end of OUT [xx], reg
						endcase // end of case ir[5:4]
					end // end of MOV/IN/OUT GROUP
					// LOAD GROUP - 2
					4'b0010: begin
						case (ir[5:4]) 
							// LD regx, [regy]
							2'b00: begin
								$display("%2x: LD r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
											// step 1: we try to read memory from the regy adress
											addr <= regs[ir[15:12]];
											mc_count <= 1;
									end
									1: begin
										// step 2: we get the memory content from the data bus and put it in the regx
										regs[ir[11:8]] <= data;
										ir <= 0;     // initiate fetch
										addr <= pc;
										// pc already points to the next instruction
									end
								endcase // mc_count
							end	// end of LD regx, [regy]
							// LD regx, [xx]
							2'b01: begin
								$display("%2x: LD r%-d, [%x]", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
											// step 1: we try to read memory from the xx adress
											addr <= data;
											mc_count <= 1;
											pc <= pc + 1'b1;  // move to the next instruction
									end
									1: begin
										// step 2: we get the memory content from the data bus and put it in the regx
										regs[ir[11:8]] <= data;
										ir <= 0;     // initiate fetch
										addr <= pc;  // put the pc to the data bus
										// pc points to the next instruction
									end
								endcase // mc_count
							end	// end of LD regx, [xx]
							// LD regx, [regy + xx]
							2'b10: begin
								$display("%2x: LD r%-d, [r%-d + %x]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
								case (mc_count)
									0: begin
											// step 1: we try to read memory from the regy+xx adress
											addr <= data + regs[ir[15:12]];
											mc_count <= 1;
											pc <= pc + 1'b1;  // move to the next instruction
									end
									1: begin
										// step 2: we get the memory content from the data bus and put it in the regx
										regs[ir[11:8]] <= data;
										ir <= 0;     // initiate fetch
										addr <= pc;
										// pc already points to the next instruction
									end
								endcase // mc_count
							end	 // end of LD regx, [regy + xx]
							// LD regx, [regy + regz + xx]
/*							
							2'b11: begin
								$display("%2x: LD r%-d, [r%-d + r%-d + %x]", ir[3:0], (ir[10:8]), (ir[14:12]), (ir[7:6]), data);
								case (mc_count)
									0: begin
											// step 1: we try to read memory from the regy+regz+xx adress
											addr <= data + regs[ir[15:12]] + regs[ir[7:6]];
											mc_count <= 1;
											pc <= pc + 1;  // move to the next instruction
									end // end of step 0
									1: begin
										// step 2: we get the memory content from the data bus and put it in the regx
										regs[ir[11:8]] <= data;
										ir <= 0;     // initiate fetch
										addr <= pc;
										// pc already points to the next instruction
									end // end of step 1
								endcase // mc_count
							end	// end of LD regx, [regy + regz + xx]
*/							
						endcase // ir [5:4] type of LOAD
					end // end of LOAD GROUP
					
					// STORE GROUP - 3
					4'b0011: begin
						case (ir[5:4]) 
							// ST [regy], regx
							2'b00: begin
								$display("%2x: ST [r%-d], r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
										// step 1: we try to write into the memory defined by the regy 
										memrd <= 0;
										memwr <= 1;
										// put regy to the addr
										addr <= regs[ir[11:8]];
										data_to_write <= regs[ir[15:12]];
										mc_count = 1;
									end
									1: begin
										// step 2: initiate fetch
										ir <= 0;    
										memrd <= 1;
										memwr <= 0;
										addr <= pc;
										// pc already points to the next instruction
									end
								endcase // mc_count
							end	
							// ST [xx], reg
							2'b01: begin
								$display("%2x: ST [%x], r%-d", ir[3:0], data, (ir[11:8]));
								case (mc_count)
									0: begin
										// step 1: we try to obtain  memory content from the next cell
										// xx is already on the data bus
										mbr <= data;
										mc_count <= 1;
										// move pc to the next instruction
										pc <= pc + 1'b1;
									end
									1: begin
										// then we try to write to the xx
										addr <= mbr;
										memrd <= 0;
										memwr <= 1;
										data_to_write <= regs[ir[11:8]];
										mc_count <= 2;
									end
									2: begin
										// step 2: initiate fetch
										ir <= 0;     
										memrd <= 1;
										memwr <= 0;
										addr <= pc;
										// pc already points to the next instruction
									end
								endcase // mc_count
							end	 // end of ST [xx], reg
							// ST [reg + xx], reg
							2'b10: begin
								$display("%2x: ST [r%-d + %x], r%d", ir[3:0], (ir[11:8]), data, (ir[15:12]));
								case (mc_count)
									0: begin
										// step 1: we try to obtain  memory content from the next cell
										// xx is already on the data bus
										mbr <= data;
										mc_count <= 1;
										// move pc to the next instruction
										pc <= pc + 1'b1;
									end
									1: begin
										// then we try to write to the reg + xx
										addr <= regs[ir[11:8]] + mbr;
										memrd <= 0;
										memwr <= 1;
										data_to_write <= regs[ir[15:12]];
										mc_count <= 2;
									end
									2: begin
										// step 2: initiate fetch
										ir <= 0;     
										memrd <= 1;
										memwr <= 0;
										addr <= pc;
										// pc already points to the next instruction
									end
								endcase // mc_count
							end	 // end of ST [reg + xx], reg
							// ST [regx + regy + xx], regz
/*							
							2'b11: begin
								$display("%2x: ST [r%-d + r%-d + %x], r%-d", ir[3:0], (ir[11:8]), (ir[7:6]), data, (ir[15:12]));
								case (mc_count)
									0: begin
										// step 1: we try to obtain  memory content from the next cell
										// xx is already on the data bus
										mbr <= data;
										mc_count <= 1;
										// move pc to the next instruction
										pc <= pc + 1;
									end
									1: begin
										// then we try to write to the regx + regy + xx
										addr <= regs[ir[11:8]]  + regs[ir[7:6]] + mbr;
										memrd <= 0;
										memwr <= 1;
										data_to_write <= regs[ir[15:12]];
										mc_count <= 2;
									end
									2: begin
										// step 2: initiate fetch
										ir <= 0;     
										memrd <= 1;
										memwr <= 0;
										addr <= pc;
										// pc already points to the next instruction
									end
								endcase // mc_count
							end	 // end of ST [regx + regy + xx], reg
*/							
						endcase // ir [5:4] type of STORE
					end	// end of STORE GROUP

					// JUMP GROUP - 4
					4'b0100: begin
						case (ir[7:4]) 
							// J xx
							4'b0000: begin
								$display("%2x: J %x", ir[3:0], data);
								pc <= data;
								addr <= data;
								ir <= 0;
							end // end of J xx
							// JZ xx
							4'b0001: begin
								$display("%2x: JZ %x", ir[3:0], data);
								if (f[ZERO] == 1) begin
									pc <= data;
									addr <= data;
								end
								else begin
									pc = pc + 1'b1;
									addr = pc;
								end
								ir <= 0;
							end // end of JZ xx
							// JNZ xx
							4'b0010: begin
								$display("%2x: JNZ %x", ir[3:0], data);
								if (f[ZERO] == 0) begin
									pc <= data;
									addr <= data;
								end
								else begin
									pc = pc + 1'b1;
									addr = pc;
								end
								ir <= 0;
							end // end of JNZ xx
							// JC xx
							4'b0011: begin
								$display("%2x: JC %x", ir[3:0], data);
								if (f[CARRY] == 1) begin
									pc <= data;
									addr <= data;
								end
								else begin
									pc = pc + 1'b1;
									addr = pc;
								end
								ir <= 0;
							end // end of JC xx
							// JNC xx
							4'b0100: begin
								$display("%2x: JNC %x", ir[3:0], data);
								if (f[CARRY] == 0) begin
									pc <= data;
									addr <= data;
								end
								else begin
									pc = pc + 1'b1;
									addr = pc;
								end
								ir <= 0;
							end // end of JNC xx
							// JO xx
							4'b0101: begin
								$display("%2x: JO %x", ir[3:0], data);
								if (f[OVERFLOW] == 1) begin
									pc <= data;
									addr <= data;
								end
								else begin
									pc = pc + 1'b1;
									addr = pc;
								end
								ir <= 0;
							end // end of JO xx
							// JNO xx
							4'b0110: begin
								$display("%2x: JNO %x", ir[3:0], data);
								if (f[OVERFLOW] == 0) begin
									pc <= data;
									addr <= data;
								end
								else begin
									pc = pc + 1'b1;
									addr = pc;
								end
								ir <= 0;
							end // end of JNO xx
							// JP xx
							4'b0111: begin
								$display("%2x: JP %x", ir[3:0], data);
								if (f[POSITIVE] == 1) begin
									pc <= data;
									addr <= data;
								end
								else begin
									pc = pc + 1'b1;
									addr = pc;
								end
								ir <= 0;
							end // end of JP xx
							// JNP xx
							4'b1000: begin
								$display("%2x: JNP %x", ir[3:0], data);
								if (f[POSITIVE] == 0) begin
									pc <= data;
									addr <= data;
								end
								else begin
									pc = pc + 1'b1;
									addr = pc;
								end
								ir <= 0;
							end // end of JNP xx
							
						endcase	
					end // end of JUMP GROUP

					// CALL/RET GROUP - 5
					4'b0101: begin
						case (ir[7:4]) 
							// CALL xx
							4'b0000: begin
								$display("%2x: CALL %x", ir[3:0], data);
								case (mc_count)
									0: begin
										// step 1: we try to obtain  memory content from the next cell
										// xx is already on the data bus
										mbr <= data;
										// push to the stack the return value
										memwr <= 1'b1;
										memrd <= 1'b0;
										addr <= regs[SP];
										regs[SP] <= regs[SP] + 1'b1;
										// the return value is in the pc and it is already pointing to the next instruction
										data_to_write <= pc + 1'b1;
										// move to the next step
										mc_count <= 1;
									end
									1: begin
										// jump to the location in mbr
										memwr <= 1'b0;
										memrd <= 1'b1;
										pc <= mbr;
										addr <= mbr;
										ir <= 0;
									end
								endcase
							end // end of CALL xx
							// RET
							4'b1001: begin
								$display("%2x: RET", ir[3:0]);
								case (mc_count)
									0: begin
										// step 1: we try to read the return value from the stack
										regs[SP] = regs[SP] - 1'b1;
										addr <= regs[SP];
										memrd <= 1'b1;
										memwr <= 1'b0;
										// move to the next step
										mc_count <= 1;
									end
									1: begin
										// jump to the location from stack
										pc <= data;
										addr <= data;
										ir <= 0;
									end
								endcase
							end // end of RET
							// IRET
							4'b1010: begin
								$display("%2x: IRET", ir[3:0]);
								case (mc_count)
									0: begin
										// step 1: we try to read the flags from the stack
										regs[SP] = regs[SP] - 1'b1;
										addr <= regs[SP];
										memrd <= 1'b1;
										memwr <= 1'b0;
										// move to the next step
										mc_count <= 1;
									end
									1: begin
										f <= data[FLAGS_COUNT-1:0];
										// step 2: we try to read the return value from the stack
										regs[SP] = regs[SP] - 1'b1;
										addr <= regs[SP];
										// move to the next step
										mc_count <= 2;
									end
									2: begin
										// jump to the location from stack
										pc <= data;
										addr <= data;
										ir <= 0;
										LED[6] <= 1;
										// ENABLE IRQ
										//noirq <= 0;
										if (noirq[0] == 1) begin
											noirq[0] <= 0;
										end
										if (noirq[1] == 1) begin
											noirq[1] <= 0;
										end
										
									end
								endcase
							end // end of IRET
							
						endcase	
					end // end of CALL/RET GROUP
					
					// ADD/SUB GROUP - 6
					4'b0110: begin
						case (ir[7:4]) 
							// ADD regx, regy
							4'b0000: begin
								$display("%2x: ADD r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
										alu_op <= ALU_ADD;
										alu_a <= regs[ir[11:8]];
										alu_b <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of ADD regx, regy
							// ADD reg, xx
							4'b0001: begin
								$display("%2x: ADD r%-d, %-d", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										alu_op <= ALU_ADD;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 1;
										pc <= pc + 1'b1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of ADD reg, xx
							// ADD regx, [regy]
							4'b0010: begin
								$display("%2x: ADD r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy adress
										addr <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_ADD;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of ADD regx, [regy]
							// ADD reg, [xx]
							4'b0011: begin
								$display("%2x: ADD r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= data;
										mc_count <= 1;
										pc <= pc + 1'b1;  // move to the next instruction
									end
									1: begin
										alu_op <= ALU_ADD;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of ADD reg, [xx]
							// ADD regx, [regy + xx]
							4'b0100: begin
								$display("%2x: ADD r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= regs[ir[15:12]] + data;
										mc_count <= 1;
										pc <= pc + 1'b1;  // move to the next instruction
									end
									1: begin
										alu_op <= ALU_ADD;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of ADD regx, [regy + xx]
							// SUB regx, regy
							4'b0110: begin
								$display("%2x: SUB r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
										alu_op <= ALU_SUB;
										alu_a <= regs[ir[11:8]];
										alu_b <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of SUB regx, regx
							// SUB reg, xx
							4'b0111: begin
								$display("%2x: SUB r%-d, %-d", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										alu_op <= ALU_SUB;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 1;
										pc <= pc + 1'b1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of SUB reg, xx
							// SUB regx, [regy]
							4'b1000: begin
								$display("%2x: SUB r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy adress
										addr <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_SUB;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of SUB regx, [regy]
							// SUB reg, [xx]
							4'b1001: begin
								$display("%2x: SUB r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= data;
										mc_count <= 1;
										pc <= pc + 1'b1;  // move to the next instruction
									end
									1: begin
										alu_op <= ALU_SUB;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of SUB reg, [xx]
							// SUB regx, [regy + xx]
							4'b1010: begin
								$display("%2x: SUB r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= regs[ir[15:12]] + data;
										mc_count <= 1;
										pc <= pc + 1'b1;  // move to the next instruction
									end
									1: begin
										alu_op <= ALU_SUB;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of SUB regx, [regy + xx]

						endcase	
					end // end of ADD/SUB GROUP

					// AND/OR/XOR GROUP - 7
					4'b0111: begin
						case (ir[7:4]) 
							// AND regx, regy
							4'b0000: begin
								$display("%2x: AND r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
										alu_op <= ALU_AND;
										alu_a <= regs[ir[11:8]];
										alu_b <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of AND regx, regy
							// AND reg, xx
							4'b0001: begin
								$display("%2x: AND r%-d, %-d", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										alu_op <= ALU_AND;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 1;
										pc <= pc + 1'b1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of AND reg, xx
							// AND regx, [regy]
							4'b0010: begin
								$display("%2x: AND r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy adress
										addr <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_AND;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of AND regx, [regy]
							// AND reg, [xx]
							4'b0011: begin
								$display("%2x: AND r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= data;
										mc_count <= 1;
										pc <= pc + 1'b1;  // move to the next instruction
									end
									1: begin
										alu_op <= ALU_AND;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of AND reg, [xx]
							// AND regx, [regy + xx]
							4'b0100: begin
								$display("%2x: ADD r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= regs[ir[15:12]] + data;
										mc_count <= 1;
										pc <= pc + 1'b1;  // move to the next instruction
									end
									1: begin
										alu_op <= ALU_AND;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of AND regx, [regy + xx]
							// OR regx, regy
							4'b0101: begin
								$display("%2x: OR r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
										alu_op <= ALU_OR;
										alu_a <= regs[ir[11:8]];
										alu_b <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of OR regx, regy
							// OR reg, xx
							4'b0110: begin
								$display("%2x: OR r%-d, %-d", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										alu_op <= ALU_OR;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 1;
										pc <= pc + 1'b1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of OR reg, xx
							// OR regx, [regy]
							4'b0111: begin
								$display("%2x: OR r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy adress
										addr <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_OR;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of OR regx, [regy]
							// OR reg, [xx]
							4'b1000: begin
								$display("%2x: OR r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= data;
										mc_count <= 1;
										pc <= pc + 1'b1;  // move to the next instruction
									end
									1: begin
										alu_op <= ALU_OR;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of OR reg, [xx]
							// OR regx, [regy + xx]
							4'b1001: begin
								$display("%2x: OR r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= regs[ir[15:12]] + data;
										mc_count <= 1;
										pc <= pc + 1'b1;  // move to the next instruction
									end
									1: begin
										alu_op <= ALU_OR;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of OR regx, [regy + xx]
							// XOR regx, regy
							4'b1010: begin
								$display("%2x: XOR r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
										alu_op <= ALU_XOR;
										alu_a <= regs[ir[11:8]];
										alu_b <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of XOR regx, regy
							// XOR reg, xx
							4'b1011: begin
								$display("%2x: XOR r%-d, %-d", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										alu_op <= ALU_XOR;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 1;
										pc <= pc + 1'b1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of XOR reg, xx
							// XOR regx, [regy]
							4'b1100: begin
								$display("%2x: XOR r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy adress
										addr <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_XOR;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of XOR regx, [regy]
							// XOR reg, [xx]
							4'b1101: begin
								$display("%2x: XOR r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= data;
										mc_count <= 1;
										pc <= pc + 1'b1;  // move to the next instruction
									end
									1: begin
										alu_op <= ALU_XOR;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of XOR reg, [xx]
							// XOR regx, [regy + xx]
							4'b1110: begin
								$display("%2x: XOR r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= regs[ir[15:12]] + data;
										mc_count <= 1;
										pc <= pc + 1'b1;  // move to the next instruction
									end
									1: begin
										alu_op <= ALU_XOR;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of XOR regx, [regy + xx]
						endcase // end of case(ir[7:4])
					end  // end of AND/OR/XOR GROUP

					// CMP GROUP - 8
					4'b1000: begin 
						case (ir[7:4])
							// CMP regx, regy
							4'b0000: begin
								$display("%2x: CMP r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
										alu_op <= ALU_SUB;
										alu_a <= regs[ir[11:8]];
										alu_b <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of CMP regx, regy
							// CMP reg, xx
							4'b0001: begin
								$display("%2x: CMP r%-d, %-d", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										alu_op <= ALU_SUB;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 1;
									end
									1: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										pc = pc + 1'b1;
										addr = pc;
									end
								endcase
							end // end of CMP regx, regy
							// CMP regx, [regy]
							4'b0010: begin
								$display("%2x: CMP r%-d, [r%-d]", ir[3:0], (ir[11:8]), ir[15:12]);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy adress
										addr <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_SUB;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										pc = pc + 1'b1;
										addr = pc;
									end
								endcase
							end // end of CMP regx, [regy]
							// CMP regx, [XX]
							4'b0011: begin
								$display("%2x: CMP r%-d, [%d]", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_SUB;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										pc = pc + 1'b1;
										addr = pc;
									end
								endcase
							end // end of CMP regx, [XX]
							// CMP regx, [regy + XX]
							4'b0100: begin
								$display("%2x: CMP r%-d, [r%-d + %d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= regs[ir[15:12]] + data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_SUB;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										pc = pc + 1'b1;
										addr = pc;
									end
								endcase
							end // end of CMP regx, [regy + XX]
						endcase // ir[7:4]
					end // end of CMP GROUP

					// INC/DEC GROUP - 9
					4'b1001: begin 
						case (ir[7:4])
							// INC reg
							4'b0000: begin
								$display("%2x: INC r%-d", ir[3:0], (ir[11:8]));
								case (mc_count)
									0: begin
										alu_op <= ALU_ADD;
										alu_a <= regs[ir[11:8]];
										alu_b <= 1;
										mc_count <= 1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of INC reg
							// INC [reg]
							4'b0001: begin
								$display("%2x: INC [r%-d]", ir[3:0], (ir[11:8]));
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy adress
										addr <= regs[ir[11:8]];
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_ADD;
										alu_a <= data;
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the reg
										memrd <= 0;
										memwr <= 1;
										// put regy to the addr
										addr <= regs[ir[11:8]];
										data_to_write <= alu_res;
										mc_count <= 3;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1;
										memwr <= 0;
										addr <= pc;
									end
								endcase
							end // end of INC [reg]
							// INC [XX]
							4'b0010: begin
								$display("%2x: INC [%d]", ir[3:0], data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= data;
										mbr <= data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_ADD;
										alu_a <= data;
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the XX
										memrd <= 0;
										memwr <= 1;
										// put regy to the addr
										addr <= mbr;
										data_to_write <= alu_res;
										mc_count <= 3;
										pc <= pc + 1'b1;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1;
										memwr <= 0;
										addr <= pc;
									end
								endcase
							end // end of INC [XX]
							// INC [reg + XX]
							4'b0011: begin
								$display("%2x: INC [r%-d + %d]", ir[3:0], ir[11:8], data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= regs[ir[11:8]] + data;
										mbr <= regs[ir[11:8]] + data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_ADD;
										alu_a <= data;
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the XX
										memrd <= 0;
										memwr <= 1;
										// put regy + data to the addr
										addr <= mbr;
										data_to_write <= alu_res;
										mc_count <= 3;
										pc <= pc + 1'b1;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1;
										memwr <= 0;
										addr <= pc;
									end
								endcase
							end // end of INC [reg]
							// DEC reg
							4'b0100: begin
								$display("%2x: DEC r%-d", ir[3:0], (ir[11:8]));
								case (mc_count)
									0: begin
										alu_op <= ALU_SUB;
										alu_a <= regs[ir[11:8]];
										alu_b <= 1;
										mc_count <= 1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase // mc_count
							end // end of DEC reg
							// DEC [reg]
							4'b0101: begin
								$display("%2x: DEC [r%-d]", ir[3:0], (ir[11:8]));
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy adress
										addr <= regs[ir[11:8]];
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_SUB;
										alu_a <= data;
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the reg
										memrd <= 0;
										memwr <= 1;
										// put regy to the addr
										addr <= regs[ir[11:8]];
										data_to_write <= alu_res;
										mc_count <= 3;
										pc <= pc + 1'b1;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1;
										memwr <= 0;
										addr <= pc;
									end
								endcase
							end // end of DEC [reg]
							// DEC [XX]
							4'b0110: begin
								$display("%2x: DEC [%d]", ir[3:0], data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= data;
										mbr <= data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_SUB;
										alu_a <= data;
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the XX
										memrd <= 0;
										memwr <= 1;
										// put regy to the addr
										addr <= mbr;
										data_to_write <= alu_res;
										mc_count <= 3;
										pc <= pc + 1'b1;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1;
										memwr <= 0;
										addr <= pc;
									end
								endcase
							end // end of DEC [XX]
							// DEC [reg + XX]
							4'b0111: begin
								$display("%2x: DEC [r%-d + %d]", ir[3:0], ir[11:8], data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= regs[ir[11:8]] + data;
										mbr <= regs[ir[11:8]] + data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_SUB;
										alu_a <= data;
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the XX
										memrd <= 0;
										memwr <= 1;
										// put regy + data to the addr
										addr <= mbr;
										data_to_write <= alu_res;
										mc_count <= 3;
										pc <= pc + 1'b1;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1;
										memwr <= 0;
										addr <= pc;
									end
								endcase
							end // end of DEC [reg]
							// NEG reg
							4'b1000: begin
								$display("%2x: NEG r%-d", ir[3:0], (ir[11:8]));
								case (mc_count)
									0: begin
										alu_op <= ALU_NEG;
										alu_a <= regs[ir[11:8]];
										alu_b <= 1;
										mc_count <= 1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of NEG reg
							// NEG [reg]
							4'b1001: begin
								$display("%2x: NEG [r%-d]", ir[3:0], (ir[11:8]));
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy adress
										addr <= regs[ir[11:8]];
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_NEG;
										alu_a <= data;
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the reg
										memrd <= 0;
										memwr <= 1;
										// put regy to the addr
										addr <= regs[ir[11:8]];
										data_to_write <= alu_res;
										mc_count <= 3;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1;
										memwr <= 0;
										addr <= pc;
									end
								endcase
							end // end of NEG [reg]
							// NEG [XX]
							4'b1010: begin
								$display("%2x: NEG [%d]", ir[3:0], data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= data;
										mbr <= data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_NEG;
										alu_a <= data;
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the XX
										memrd <= 0;
										memwr <= 1;
										// put regy to the addr
										addr <= mbr;
										data_to_write <= alu_res;
										mc_count <= 3;
										pc <= pc + 1'b1;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1;
										memwr <= 0;
										addr <= pc;
									end
								endcase
							end // end of NEG [XX]
							// NEG [reg + XX]
							4'b1011: begin
								$display("%2x: NEG [r%-d + %d]", ir[3:0], ir[11:8], data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx adress
										addr <= regs[ir[11:8]] + data;
										mbr <= regs[ir[11:8]] + data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_NEG;
										alu_a <= data;
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the XX
										memrd <= 0;
										memwr <= 1;
										// put regy + data to the addr
										addr <= mbr;
										data_to_write <= alu_res;
										mc_count <= 3;
										pc <= pc + 1'b1;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1;
										memwr <= 0;
										addr <= pc;
									end
								endcase
							end // end of NEG [reg + xx]
							
						endcase // ir[7:4]
					end // end of CMP GROUP

					// MUL/DIV GROUP - 10 (0xa)
					4'b1010: begin
						case (ir[7:4]) 
							// MUL regx, regy
							4'b0000: begin
								$display("%2x: MUL r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
										alu_op <= ALU_MUL;
										alu_a <= regs[ir[11:8]];
										alu_b <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										regs[9] <= alu_high;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of MUL regx, regy
							// MUL reg, xx
							4'b0001: begin
								$display("%2x: MUL r%-d, %-d", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										alu_op <= ALU_MUL;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 1;
										pc <= pc + 1'b1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										regs[H] <= alu_high;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of MUL reg, xx
							// MUL regx, [regy]
							4'b0010: begin
								$display("%2x: MUL r%-d, [r%-d]", ir[3:0], (ir[11:8]), ir[15:12]);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy adress
										addr <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_MUL;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										regs[H] <= alu_high;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of MUL regx, [regy]
							// MUL regx, [XX]
							4'b0011: begin
								$display("%2x: MUL r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the data
										addr <= data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_MUL;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
										pc <= pc + 1'b1;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										regs[H] <= alu_high;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of MUL regx, [XX]
							// MUL regx, [regy + XX]
							4'b0100: begin
								$display("%2x: MUL r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the data
										addr <= regs[ir[15:12]] + data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_MUL;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
										pc <= pc + 1'b1;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										regs[H] <= alu_high;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of MUL regx, [regy]
							// DIV regx, regy
							4'b0110: begin
								$display("%2x: DIV r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
										alu_op <= ALU_DIV;
										alu_a <= regs[ir[11:8]];
										alu_b <= regs[ir[15:12]];
										start_div <= 1'b1;
										mc_count <= 1;
									end
									1: begin
										start_div <= 1'b0;
										if (div_finished) begin
											if (div_counter < DIV_CYCLES) begin
												div_counter <= div_counter + 1'b1;
												start_div <= 1'b1;
											end
											else begin
												regs[ir[11:8]] <= alu_res;
												f <= f_from_alu;
												regs[H] <= alu_high;
												ir <= 0;     // initiate fetch
												addr <= pc;
											end
										end // if (div_finished)
									end // step 1
								endcase // mc_count
							end // end of DIV regx, regy
							// DIV reg, xx
							4'b0111: begin
								$display("%2x: DIV r%-d, %-d", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										alu_op <= ALU_DIV;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										pc <= pc + 1'b1;
										start_div <= 1'b1;
										div_counter <= 0;
										mc_count <= 1'b1;
									end
									1: begin
										start_div <= 1'b0;
										if (div_finished) begin
											if (div_counter < DIV_CYCLES) begin
												div_counter <= div_counter + 1'b1;
												start_div <= 1'b1;
											end
											else begin
												regs[ir[11:8]] <= alu_res;
												f <= f_from_alu;
												regs[H] <= alu_high;
												ir <= 0;     // initiate fetch
												addr <= pc;
											end
										end // if (div_finished)
									end // step 1
								endcase
							end // end of DIV reg, xx
							// DIV regx, [regy]
							4'b1000: begin
								$display("%2x: DIV r%-d, [r%-d]", ir[3:0], (ir[11:8]), ir[15:12]);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy adress
										addr <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_DIV;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										start_div <= 1'b1;
										mc_count <= 2;
									end
									2: begin
										start_div <= 1'b0;
										if (div_finished) begin
											if (div_counter < DIV_CYCLES) begin
												div_counter <= div_counter + 1'b1;
												start_div <= 1'b1;
											end
											else begin
												regs[ir[11:8]] <= alu_res;
												f <= f_from_alu;
												regs[H] <= alu_high;
												ir <= 0;     // initiate fetch
												addr <= pc;
											end
										end // if (div_finished)
									end // step 2
								endcase
							end // end of DIV regx, [regy]
							// DIV regx, [XX]
							4'b1001: begin
								$display("%2x: DIV r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the data
										addr <= data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_DIV;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										start_div <= 1'b1;
										pc <= pc + 1'b1;
										mc_count <= 2;
									end
									2: begin
										start_div <= 1'b0;
										if (div_finished) begin
											if (div_counter < DIV_CYCLES) begin
												div_counter <= div_counter + 1'b1;
												start_div <= 1'b1;
											end
											else begin
												regs[ir[11:8]] <= alu_res;
												f <= f_from_alu;
												regs[H] <= alu_high;
												ir <= 0;     // initiate fetch
												addr <= pc;
											end
										end // if (div_finished)
									end // step 2
								endcase
							end // end of DIV regx, [XX]
							// DIV regx, [regy + XX]
							4'b1010: begin
								$display("%2x: DIV r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the data
										addr <= regs[ir[15:12]] + data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_DIV;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										start_div <= 1'b1;
										pc <= pc + 1'b1;
										mc_count <= 2;
									end
									2: begin
										start_div <= 1'b0;
										if (div_finished) begin
											if (div_counter < DIV_CYCLES) begin
												div_counter <= div_counter + 1'b1;
												start_div <= 1'b1;
											end
											else begin
												regs[ir[11:8]] <= alu_res;
												f <= f_from_alu;
												regs[H] <= alu_high;
												ir <= 0;     // initiate fetch
												addr <= pc;
											end
										end // if (div_finished)
									end // step 2
								endcase
							end // end of MUL regx, [regy]

						endcase	
					end // end of MUL/DIV GROUP

					// PUSH/POP GROUP - 11 (0x0b)
					4'b1011: begin
						case (ir[7:4]) 
							// PUSH reg
							4'b0000: begin
								$display("%2x: PUSH r%-d", ir[3:0], (ir[11:8]));
								case (mc_count)
									0: begin
										addr <= regs[SP];
										memrd <= 0;
										memwr <= 1;
										data_to_write <= regs[ir[11:8]];
										// move sp to the next location
										regs[SP] <= regs[SP] + 1'b1;
										// next step
										mc_count <= 1;
									end
									1: begin
										// step 2: initiate fetch
										ir <= 0;     
										memrd <= 1;
										memwr <= 0;
										addr <= pc;
										// pc already points to the next instruction
									end
							endcase
							end // end of PUSH reg
							// PUSH xx
							4'b0001: begin
								$display("%2x: PUSH %d", ir[3:0], data);
								case (mc_count)
									0: begin
										// step 1: we try to obtain  memory content from the next cell
										// xx is already on the data bus
										mbr <= data;
										// move pc to the next instruction
										pc <= pc + 1'b1;
										// move to the next step
										mc_count <= 1;
									end
									1: begin
										addr <= regs[SP];
										memrd <= 0;
										memwr <= 1;
										data_to_write <= mbr;
										// move sp to the next location
										regs[SP] <= regs[SP] + 1'b1;
										// next step
										mc_count <= 2;
									end
									2: begin
										// step 2: initiate fetch
										ir <= 0;     
										memrd <= 1;
										memwr <= 0;
										addr <= pc;
										// pc already points to the next instruction
									end
								endcase // mc_count
							end // end of DIV regx, regy
							// POP reg
							4'b0010: begin
								$display("%2x: POP r%-d", ir[3:0], (ir[11:8]));
								case (mc_count)
									0: begin
										// step 1: we try to read from the stack
										regs[SP] = regs[SP] - 1'b1;
										addr <= regs[SP];
										// move to the next step
										mc_count <= 1;
									end
									1: begin
										regs[ir[11:8]] <= data;
										// initiate fetch
										addr <= pc;
										ir <= 0;
									end
								endcase // mc_count
							end // end of DIV regx, regy

						endcase	
					end // end of MUL/DIV GROUP

					// SHL/SHR GROUP - 12 (0x0c)
					4'b1100: begin
						case (ir[7:4]) 
							// SHL regx, regy
							4'b0000: begin
								$display("%2x: SHL r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
										alu_op <= ALU_SHL;
										alu_a <= regs[ir[11:8]];
										alu_b <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of SHL regx, regy
							// SHL reg, xx
							4'b0001: begin
								$display("%2x: SHL r%-d, %-d", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										alu_op <= ALU_SHL;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 1;
										pc <= pc + 1'b1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of SHL reg, xx
							// SHL regx, [regy]
							4'b0010: begin
								$display("%2x: SHL r%-d, [r%-d]", ir[3:0], (ir[11:8]), ir[15:12]);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the data
										addr <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_SHL;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of SHL regx, [regy]
							// SHL regx, [XX]
							4'b0011: begin
								$display("%2x: SHL r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the data
										addr <= data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_SHL;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
										pc <= pc + 1'b1;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of SHL regx, [XX]
							// SHL regx, [regy + XX]
							4'b0100: begin
								$display("%2x: SHL r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy + data
										addr <= regs[ir[15:12]] + data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_SHL;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
										pc <= pc + 1'b1;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of SHL regx, [regy + XX]
							// SHR regx, regy
							4'b0110: begin
								$display("%2x: SHR r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								case (mc_count)
									0: begin
										alu_op <= ALU_SHR;
										alu_a <= regs[ir[11:8]];
										alu_b <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of SHR regx, regx
							// SHR reg, xx
							4'b0111: begin
								$display("%2x: SHR r%-d, %-d", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										alu_op <= ALU_SHR;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 1;
										pc <= pc + 1'b1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of SHR reg, xx
							// SHR regx, [regy]
							4'b1000: begin
								$display("%2x: SHR r%-d, [r%-d]", ir[3:0], (ir[11:8]), ir[15:12]);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the data
										addr <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_SHR;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of SHR regx, [regy]
							// SHR regx, [XX]
							4'b1001: begin
								$display("%2x: SHR r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the data
										addr <= data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_SHR;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
										pc <= pc + 1'b1;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of SHL regx, [XX]
							// SHR regx, [regy + XX]
							4'b1010: begin
								$display("%2x: SHR r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy + data
										addr <= regs[ir[15:12]] + data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_SHR;
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
										pc <= pc + 1'b1;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc;
									end
								endcase
							end // end of SHR regx, [regy + XX]

						endcase	
					end // end of SHL/SHR GROUP

					
				endcase	// {ir[3:0]}  - opcode
			end
		endcase // ir - fetch vs. opcode
	end
	$display("high=%4x\tpc=%4x", h, pc);
	$display("f=%2x\t\tsp=%4x", f, sp);
	$display("r0=%4x\t\tir=%4x", r0, ir);
	$display("r1=%4x\t\tmc_count=%x", r1, mc_count);
	$display("r2=%4x", r2);
	$display("r3=%4x\t\taddr=%4x", r3,addr);
	$display("r4=%4x\t\tdata=%4x/%4x", r4, data, data_to_write);
	$display("r5=%4x\t\tmr/mw=%x/%x", r5, memrd, memwr);
	$display("r6=%4x\t\tmbr=%x", r6, mbr);
	$display("r7=%4x\n", r7);

end

initial begin
	// RESET
	halt <= 0;
	pc <= 0;
	mbr <= 0;
	f <= 0;
	// reset registers
	regs[0] <= 0;
	regs[1] <= 0;
	regs[2] <= 0;
	regs[3] <= 0;
	regs[4] <= 0;
	regs[5] <= 0;
	regs[6] <= 0;
	regs[7] <= 0;
	regs[SP] <= 0; // sp
	regs[H] <= 0; // high
	// IR stuff
	ir <= 0;
	// this value is assigned at the startup
	mc_count <= 'hff;
	// ALU stuff
	alu_op <= 0;
	// bus stuff
	// we will start reading from the PC immediately
	memrd <= 1;
	addr <= 0;
	memwr <= 0;
	irq_r <= 0;
	irq_count <= 0;
	noirq <= 0;
end

endmodule

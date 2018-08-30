
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
	output [7:0] tx_data,    // UART TX data
	output [1:0] vga_mode,	 // VGA mode: 0-text; 1-320x240
	input [7:0] ps2_data		 // keyboard data
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


//=======================================================
//  REG/WIRE declarations
//=======================================================

reg [N-1:0] regs[9:0];  // 0-7 are r0-r7; 8 is sp, 9 is high
reg [N-1:0] pc, mbr, ir;
reg [N-1:0] mc_count, irq_count;
reg [7:0] irq_r;
reg [7:0] noirq;
reg [7:0] rx_data_r, ps2_data_r;

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
		`ifdef DEBUG
		LED[7:0] <= 0;
		`endif
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
		`ifdef DEBUG
		$display("&&&&&&&&&&&&&&&&&&&&&&&&&& IRQ HAPPENED: %d",  irq);
		LED[3:0] <= irq[3:0];
		LED[7:4] <= 0;
		`endif
		if (irq[0]) begin
			noirq[0] <= 1; // mask IRQ0 off - no IRQ0
			irq_r[0] <= 1;
		end
		if (irq[1]) begin
			rx_data_r <= rx_data;
			noirq[1] <= 1; // mask IRQ1 off - no IRQ1
			irq_r[1] <= 1;
		end
		if (irq[2]) begin
			ps2_data_r <= ps2_data;
			noirq[2] <= 1; // mask IRQ2 off - no IRQ2
			irq_r[2] <= 1;
		end
		irq_count <= 1;
	end
	else if (irq_count && (ir == 0)) begin
		`ifdef DEBUG
		$display("####################################################IRQ HAPPENED: %d",  irq_r);
		LED[4] <= 1;
		`endif
		case (irq_count)
			1: begin
				`ifdef DEBUG
				$display("1. IRQ PUSH PC");
				`endif
				// push to the stack the return value
				memwr <= 1'b1;
				memrd <= 1'b0;
				addr <= regs[SP] >> 1;
				regs[SP] <= regs[SP] + 2'd2;
				// the return value is in the pc and it is already pointing to the next instruction
				data_to_write <= pc;
				irq_count <= 2;
			end
			2: begin
				`ifdef DEBUG
				$display("2. IRQ PUSH FLAGS");
				`endif
				// push to the stack flags
				memwr <= 1'b1;
				memrd <= 1'b0;
				addr <= regs[SP] >> 1;
				regs[SP] <= regs[SP] + 2'd2;
				// back up the flags register
				data_to_write <= f;
				irq_count <= 3;
			end
			3: begin
				`ifdef DEBUG
				$display("3. JUMP TO IRQ SERVICE #%d", irq_r);
				`endif
				memwr <= 1'b0;
				memrd <= 1'b1;
				if (irq_r[0]) begin
					`ifdef DEBUG
					$display("3.1 JUMP TO IRQ #0 SERVICE");
					`endif
					pc <= 16'd8;
					addr <= 16'd4;
				end 
				if (irq_r[1]) begin
					`ifdef DEBUG
					LED[7] <= 1;
					$display("3.1 JUMP TO IRQ #1 SERVICE");
					`endif
					pc <= 16'd16;
					addr <= 16'd8;
				end
				if (irq_r[2]) begin
					`ifdef DEBUG
					LED[7] <= 1;
					$display("3.1 JUMP TO IRQ #2 SERVICE");
					`endif
					pc <= 16'd24;
					addr <= 16'd12;
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
			16'h0: begin
					`ifdef DEBUG
					LED[2] <= 1;
					`endif
					// begin of the fetch; we have sent the memrd signal to the memory
					// in the next cycle, we will fetch the instruction from the data bus (came from the memory)
					if (mc_count == 16'h00ff) begin
						`ifdef DEBUG
						$display("FETCH STEP 0");
						`endif
						// first power on, memory not ready yet
						memwr <= 1'b0;
						memrd <= 1'b1;
						ir <= 0;
						mc_count <= 16'hffff;
						pc <= pc + 2'd2; // pc moves to the next word (could be the operand, could be the next instruction)
						addr <= pc >> 1;
					end
					else if (mc_count == 16'hffff) begin
						`ifdef DEBUG
						$display("FETCH STEP 1");
						`endif
						mc_count <= 16'd0;
						ir <= data;
						addr <= pc >> 1;
						// at this moment, data bus holds the operand, or the next instruction
					end
					else begin
						`ifdef DEBUG
						$display("FETCH STEP 0");
						`endif
						// reset key, or normal fetch
						memwr <= 1'b0;
						memrd <= 1'b1;
						ir <= data;
						mc_count <= 0;
						pc <= pc + 2'd2; // pc moves to the next word (could be the operand, could be the next instruction)
						addr <= (pc + 2'd2) >> 1;
						// at this moment, data bus holds the operand, or the next instruction
					end
				end
			default: begin
				case ({ir[3:0]}) 
					// GROUP - 0 (NOP, MOV, IN, OUT, PUSH, POP, RET, IRET, SWAP)
					4'b0000: begin
						case (ir[7:4]) 
							4'b0000: begin
								// NOP
								`ifdef DEBUG
								$display("%2x: NOP", ir[3:0]);
								`endif
							end
							4'b0001: begin
								// MOV regx, regy
								`ifdef DEBUG
								$display("%2x: MOV r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								`endif
								regs[ir[11:8]] <= regs[ir[15:12]];
								ir <= 0;     // initiate fetch
								addr <= pc >> 1;
								// pc already points to the next instruction
							end
							4'b0010: begin
								// MOV reg, xx
								`ifdef DEBUG
								$display("%2x: MOV r%-d, %4d",ir[3:0], (ir[11:8]), data);
								`endif
								regs[ir[11:8]] <= data;
								ir <= 0;      // initiate fetch
								pc <= pc + 2'd2;  // move to the next instruction
								addr <= (pc + 2'd2) >> 1;
							end
							4'b0011: begin
								// IN reg, [xx]
								`ifdef DEBUG
								$display("%2x: IN r%-d, [%4d]",ir[3:0], (ir[11:8]), data);
								`endif
								//LED[3] <= 1;
								case (mc_count)
									0: begin
											mbr <= data;  // remember the address of IO port
											mc_count <= 1;
											pc <= pc + 2'd2;  // move to the next instruction
									end
									1: begin
										case (mbr)
											64: begin    // UART RX DATA
												regs[ir[11:8]] <= {8'b0, rx_data_r};
											end
											65: begin   // UART TX BUSY
												regs[ir[11:8]] <= tx_busy;
											end
											68: begin    // keyboard data
												regs[ir[11:8]] <= {8'b0, ps2_data_r};
											end
										endcase // end of case(mbr)
										ir <= 0;      // initiate fetch
										addr <= pc >> 1;							
									end
									default: begin
									end
								endcase  // end of case (mc_count)
							end // end of IN reg, [xx]
							// OUT [xx], reg
							4'b0100: begin
								`ifdef DEBUG
								$display("%2x: OUT [%4d], r%-d",ir[3:0], data, (ir[15:12]));
								LED[3] <= 1;
								`endif
								case (mc_count) 
									0: begin
										pc <= pc + 2'd2;  // move to the next instruction
										case (data)
											66: begin  // UART TX data 
												tx_data <= regs[ir[15:12]];
												tx_send <= 1;
											end
											67: begin  // LEDs
												LED[7:0] <= regs[ir[15:12]];
											end
											128: begin  // graphics mode: 0 - text; 1 - 320x240
												vga_mode <= regs[ir[15:12]];
											end
											default: begin
											end
										endcase  // end of case (data)
										mc_count <= 1;
									end
									1: begin
										ir <= 0;      // initiate fetch
										addr <= pc >> 1;							
										tx_send <= 0;
									end
									default: begin
									end
									endcase
							end // end of OUT [xx], reg
							// PUSH reg
							4'b0101: begin
								`ifdef DEBUG
								$display("%2x: PUSH r%-d", ir[3:0], (ir[11:8]));
								`endif
								case (mc_count)
									0: begin
										addr <= regs[SP] >> 1;
										memrd <= 0;
										memwr <= 1;
										data_to_write <= regs[ir[11:8]];
										// move sp to the next location
										regs[SP] <= regs[SP] + 2'd2;
										// next step
										mc_count <= 1;
									end
									1: begin
										// step 2: initiate fetch
										ir <= 0;     
										memrd <= 1;
										memwr <= 0;
										addr <= pc >> 1;
										// pc already points to the next instruction
									end
									default: begin
									end
							endcase
							end // end of PUSH reg
							// PUSH xx
							4'b0110: begin
								`ifdef DEBUG
								$display("%2x: PUSH %d", ir[3:0], data);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to obtain  memory content from the next cell
										// xx is already on the data bus
										mbr <= data;
										// move pc to the next instruction
										pc <= pc + 2'd2;
										// move to the next step
										mc_count <= 1;
									end
									1: begin
										addr <= regs[SP] >> 1;
										memrd <= 0;
										memwr <= 1;
										data_to_write <= mbr;
										// move sp to the next location
										regs[SP] <= regs[SP] + 2'd2;
										// next step
										mc_count <= 2;
									end
									2: begin
										// step 2: initiate fetch
										ir <= 0;     
										memrd <= 1;
										memwr <= 0;
										addr <= pc >> 1;
										// pc already points to the next instruction
									end
									default: begin
									end
								endcase // mc_count
							end // end of DIV regx, regy
							// POP reg
							4'b0111: begin
								`ifdef DEBUG
								$display("%2x: POP r%-d", ir[3:0], (ir[11:8]));
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read from the stack
										regs[SP] <= regs[SP] - 2'd2;
										addr <= (regs[SP] - 2'd2) >> 1;
										// move to the next step
										mc_count <= 1;
									end
									1: begin
										regs[ir[11:8]] <= data;
										// initiate fetch
										addr <= pc >> 1;
										ir <= 0;
									end
									default: begin
									end
								endcase // mc_count
							end // end of POP reg
							// RET
							4'b1000: begin
								`ifdef DEBUG
								$display("%2x: RET", ir[3:0]);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read the return value from the stack
										regs[SP] <= regs[SP] - 2'd2;
										addr <= (regs[SP] - 2'd2) >> 1;
										memrd <= 1'b1;
										memwr <= 1'b0;
										// move to the next step
										mc_count <= 1;
									end
									1: begin
										// jump to the location from stack
										pc <= data;
										addr <= data >> 1;
										ir <= 0;
									end
									default: begin
									end
								endcase
							end // end of RET
							// IRET
							4'b1001: begin
								`ifdef DEBUG
								$display("%2x: IRET", ir[3:0]);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read the flags from the stack
										regs[SP] = regs[SP] - 2'd2;
										addr <= regs[SP] >> 1;
										memrd <= 1'b1;
										memwr <= 1'b0;
										// move to the next step
										mc_count <= 1;
									end
									1: begin
										f <= data[FLAGS_COUNT-1:0];
										// step 2: we try to read the return value from the stack
										regs[SP] <= regs[SP] - 2'd2;
										addr <= (regs[SP] - 2'd2) >> 1;
										// move to the next step
										mc_count <= 2;
									end
									2: begin
										// jump to the location from stack
										pc <= data;
										addr <= data >> 1;
										ir <= 0;
										`ifdef DEBUG
										LED[6] <= 1;
										`endif
										// ENABLE IRQ
										//noirq <= 0;
										if (noirq[0] == 1) begin
											noirq[0] <= 0;
										end
										if (noirq[1] == 1) begin
											noirq[1] <= 0;
										end
										if (noirq[2] == 1) begin
											noirq[2] <= 0;
										end
									end
									default: begin
									end
								endcase
							end // end of IRET
							// SWAP regx, regy
							4'b1010: begin
								// SWAP regx, regy
								`ifdef DEBUG
								$display("%2x: SWAP r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								`endif
								regs[ir[11:8]] <= regs[ir[15:12]];
								regs[ir[15:12]] <= regs[ir[11:8]];
								ir <= 0;     // initiate fetch
								addr <= pc >> 1;
								// pc already points to the next instruction
							end // END OF SWAP
							4'b1111: begin
								`ifdef DEBUG
								$display("HALT");
								LED[5] <= 1;
								`endif
								halt <= 1;
								pc <= pc - 2'd2;
								addr <= (pc - 2'd2) >> 1;
								ir <= 0;
							end
						endcase
					end // end of GROUP - 0 (NOP, MOV, IN, OUT, PUSH, POP, RET, IRET, SWAP)
					
					// GROUP - 1 (JUMP)
					4'b0001: begin 
						case (ir[7:4]) 
							// J xx
							4'b0000: begin
								`ifdef DEBUG
								$display("%2x: J %x", ir[3:0], data);
								`endif
								pc <= data;
								addr <= data >>1;
								ir <= 0;
							end // end of J xx
							// JZ xx
							4'b0001: begin
								`ifdef DEBUG
								$display("%2x: JZ %x", ir[3:0], data);
								`endif
								if (f[ZERO] == 1) begin
									pc <= data;
									addr <= data >> 1;
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >>1;
								end
								ir <= 0;
							end // end of JZ xx
							// JNZ xx
							4'b0010: begin
								`ifdef DEBUG
								$display("%2x: JNZ %x", ir[3:0], data);
								`endif
								if (f[ZERO] == 0) begin
									pc <= data;
									addr <= data >> 1;
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >>1;
								end
								ir <= 0;
							end // end of JNZ xx
							// JC xx
							4'b0011: begin
								`ifdef DEBUG
								$display("%2x: JC %x", ir[3:0], data);
								`endif
								if (f[CARRY] == 1) begin
									pc <= data;
									addr <= data >> 1;
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >>1;
								end
								ir <= 0;
							end // end of JC xx
							// JNC xx
							4'b0100: begin
								`ifdef DEBUG
								$display("%2x: JNC %x", ir[3:0], data);
								`endif
								if (f[CARRY] == 0) begin
									pc <= data;
									addr <= data >> 1;
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >>1;
								end
								ir <= 0;
							end // end of JNC xx
							// JO xx
							4'b0101: begin
								`ifdef DEBUG
								$display("%2x: JO %x", ir[3:0], data);
								`endif
								if (f[OVERFLOW] == 1) begin
									pc <= data;
									addr <= data >> 1;
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >>1;
								end
								ir <= 0;
							end // end of JO xx
							// JNO xx
							4'b0110: begin
								`ifdef DEBUG
								$display("%2x: JNO %x", ir[3:0], data);
								`endif
								if (f[OVERFLOW] == 0) begin
									pc <= data;
									addr <= data >> 1;
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >>1;
								end
								ir <= 0;
							end // end of JNO xx
							// JP xx
							4'b0111: begin
								`ifdef DEBUG
								$display("%2x: JP %x", ir[3:0], data);
								`endif
								if (f[POSITIVE] == 1) begin
									pc <= data;
									addr <= data >> 1;
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >>1;
								end
								ir <= 0;
							end // end of JP xx
							// JNP xx, JS xx - jump no positive, also jump smaller
							4'b1000: begin
								`ifdef DEBUG
								$display("%2x: JNP (JS) %x", ir[3:0], data);
								`endif
								if (f[POSITIVE] == 0) begin
									pc <= data;
									addr <= data >> 1;
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >>1;
								end
								ir <= 0;
							end // end of JNP xx, JS xx
							// JG xx
							4'b1001: begin
								`ifdef DEBUG
								$display("%2x: JG %x", ir[3:0], data);
								`endif
								if (f[POSITIVE] == 1 && f[ZERO] == 0) begin
									pc <= data;
									addr <= data >> 1;
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >>1;
								end
								ir <= 0;
							end // end of JG xx
							// JSE xx - jump smaller or equal
							4'b1010: begin
								`ifdef DEBUG
								$display("%2x: JSE %x", ir[3:0], data);
								`endif
								if (f[POSITIVE] == 0 || f[ZERO] == 1) begin
									pc <= data;
									addr <= data >> 1;
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >>1;
								end
								ir <= 0;
							end // end of JSE xx
						endcase // end of case ir[7:4]
					end // end of GROUP - 1 (JUMP)
					
					// GROUP - 2 (CALL)
					4'b0010: begin
						case (ir[7:4])
							// CALL xx
							4'b0000: begin
								`ifdef DEBUG
								$display("%2x: CALL %x", ir[3:0], data);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to obtain  memory content from the next cell
										// xx is already on the data bus
										mbr <= data;
										// push to the stack the return value
										memwr <= 1'b1;
										memrd <= 1'b0;
										addr <= regs[SP] >> 1;
										regs[SP] <= regs[SP] + 2'd2;
										// the return value is in the pc + 2 
										data_to_write <= pc + 2'd2;
										// move to the next step
										mc_count <= 1;
									end
									1: begin
										// jump to the location in mbr
										memwr <= 1'b0;
										memrd <= 1'b1;
										pc <= mbr;
										addr <= mbr >> 1;
										ir <= 0;
									end
									default: begin
									end
								endcase
							end // end of CALL xx
							// CALLZ xx
							4'b0001: begin
								`ifdef DEBUG
								$display("%2x: CALLZ %x", ir[3:0], data);
								`endif
								if (f[ZERO] == 1) begin
									case (mc_count)
										0: begin
											// step 1: we try to obtain  memory content from the next cell
											// xx is already on the data bus
											mbr <= data;
											// push to the stack the return value
											memwr <= 1'b1;
											memrd <= 1'b0;
											addr <= regs[SP] >> 1;
											regs[SP] <= regs[SP] + 2'd2;
											// the return value is in the pc + 2 
											data_to_write <= pc + 2'd2;
											// move to the next step
											mc_count <= 1;
										end
										1: begin
											// jump to the location in mbr
											memwr <= 1'b0;
											memrd <= 1'b1;
											pc <= mbr;
											addr <= mbr >> 1;
											ir <= 0;
										end
										default: begin
										end
									endcase
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >> 1;
									ir <= 0;
								end
							end // end of CALLZ xx
							// CALLNZ xx
							4'b0010: begin
								`ifdef DEBUG
								$display("%2x: CALLNZ %x", ir[3:0], data);
								`endif
								if (f[ZERO] == 0) begin
									case (mc_count)
										0: begin
											// step 1: we try to obtain  memory content from the next cell
											// xx is already on the data bus
											mbr <= data;
											// push to the stack the return value
											memwr <= 1'b1;
											memrd <= 1'b0;
											addr <= regs[SP] >> 1;
											regs[SP] <= regs[SP] + 2'd2;
											// the return value is in the pc + 2 
											data_to_write <= pc + 2'd2;
											// move to the next step
											mc_count <= 1;
										end
										1: begin
											// jump to the location in mbr
											memwr <= 1'b0;
											memrd <= 1'b1;
											pc <= mbr;
											addr <= mbr >> 1;
											ir <= 0;
										end
										default: begin
										end
									endcase
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >> 1;
									ir <= 0;
								end
							end // end of CALLNZ xx
							// CALLC xx
							4'b0011: begin
								`ifdef DEBUG
								$display("%2x: CALLC %x", ir[3:0], data);
								`endif
								if (f[CARRY] == 1) begin
									case (mc_count)
										0: begin
											// step 1: we try to obtain  memory content from the next cell
											// xx is already on the data bus
											mbr <= data;
											// push to the stack the return value
											memwr <= 1'b1;
											memrd <= 1'b0;
											addr <= regs[SP] >> 1;
											regs[SP] <= regs[SP] + 2'd2;
											// the return value is in the pc + 2 
											data_to_write <= pc + 2'd2;
											// move to the next step
											mc_count <= 1;
										end
										1: begin
											// jump to the location in mbr
											memwr <= 1'b0;
											memrd <= 1'b1;
											pc <= mbr;
											addr <= mbr >> 1;
											ir <= 0;
										end
										default: begin
										end
									endcase
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >> 1;
									ir <= 0;
								end
							end // end of CALLC xx
							// CALLNC xx
							4'b0100: begin
								`ifdef DEBUG
								$display("%2x: CALLNC %x", ir[3:0], data);
								`endif
								if (f[CARRY] == 0) begin
									case (mc_count)
										0: begin
											// step 1: we try to obtain  memory content from the next cell
											// xx is already on the data bus
											mbr <= data;
											// push to the stack the return value
											memwr <= 1'b1;
											memrd <= 1'b0;
											addr <= regs[SP] >> 1;
											regs[SP] <= regs[SP] + 2'd2;
											// the return value is in the pc + 2 
											data_to_write <= pc + 2'd2;
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
										default: begin
										end
									endcase
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >> 1;
									ir <= 0;
								end
							end // end of CALLNC xx
							// CALLO xx
							4'b0101: begin
								`ifdef DEBUG
								$display("%2x: CALLO %x", ir[3:0], data);
								`endif
								if (f[OVERFLOW] == 1) begin
									case (mc_count)
										0: begin
											// step 1: we try to obtain  memory content from the next cell
											// xx is already on the data bus
											mbr <= data;
											// push to the stack the return value
											memwr <= 1'b1;
											memrd <= 1'b0;
											addr <= regs[SP] >> 1;
											regs[SP] <= regs[SP] + 2'd2;
											// the return value is in the pc + 2 
											data_to_write <= pc + 2'd2;
											// move to the next step
											mc_count <= 1;
										end
										1: begin
											// jump to the location in mbr
											memwr <= 1'b0;
											memrd <= 1'b1;
											pc <= mbr;
											addr <= mbr >> 1;
											ir <= 0;
										end
										default: begin
										end
									endcase
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >> 1;
									ir <= 0;
								end
							end // end of CALLO xx
							// CALLNO xx
							4'b0110: begin
								`ifdef DEBUG
								$display("%2x: CALLNO %x", ir[3:0], data);
								`endif
								if (f[OVERFLOW] == 0) begin
									case (mc_count)
										0: begin
											// step 1: we try to obtain  memory content from the next cell
											// xx is already on the data bus
											mbr <= data;
											// push to the stack the return value
											memwr <= 1'b1;
											memrd <= 1'b0;
											addr <= regs[SP] >> 1;
											regs[SP] <= regs[SP] + 2'd2;
											// the return value is in the pc + 2 
											data_to_write <= pc + 2'd2;
											// move to the next step
											mc_count <= 1;
										end
										1: begin
											// jump to the location in mbr
											memwr <= 1'b0;
											memrd <= 1'b1;
											pc <= mbr;
											addr <= mbr >> 1;
											ir <= 0;
										end
										default: begin
										end
									endcase
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >> 1;
									ir <= 0;
								end
							end // end of CALLNO xx
							// CALLP xx, CALLGE xx
							4'b0111: begin
								`ifdef DEBUG
								$display("%2x: CALLP %x", ir[3:0], data);
								`endif
								if (f[POSITIVE] == 1) begin
									case (mc_count)
										0: begin
											// step 1: we try to obtain  memory content from the next cell
											// xx is already on the data bus
											mbr <= data;
											// push to the stack the return value
											memwr <= 1'b1;
											memrd <= 1'b0;
											addr <= regs[SP] >> 1;
											regs[SP] <= regs[SP] + 2'd2;
											// the return value is in the pc + 2 
											data_to_write <= pc + 2'd2;
											// move to the next step
											mc_count <= 1;
										end
										1: begin
											// jump to the location in mbr
											memwr <= 1'b0;
											memrd <= 1'b1;
											pc <= mbr;
											addr <= mbr >> 1;
											ir <= 0;
										end
										default: begin
										end
									endcase
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >> 1;
									ir <= 0;
								end
							end // end of CALLP xx
							// CALLNP xx, CALLS xx - call smaller
							4'b1000: begin
								`ifdef DEBUG
								$display("%2x: CALLNP %x", ir[3:0], data);
								`endif
								if (f[POSITIVE] == 0) begin
									case (mc_count)
										0: begin
											// step 1: we try to obtain  memory content from the next cell
											// xx is already on the data bus
											mbr <= data;
											// push to the stack the return value
											memwr <= 1'b1;
											memrd <= 1'b0;
											addr <= regs[SP] >> 1;
											regs[SP] <= regs[SP] + 2'd2;
											// the return value is in the pc + 2 
											data_to_write <= pc + 2'd2;
											// move to the next step
											mc_count <= 1;
										end
										1: begin
											// jump to the location in mbr
											memwr <= 1'b0;
											memrd <= 1'b1;
											pc <= mbr;
											addr <= mbr >> 1;
											ir <= 0;
										end
										default: begin
										end
									endcase
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >> 1;
									ir <= 0;
								end
							end // end of CALLNP xx
							// CALLG xx
							4'b1001: begin
								`ifdef DEBUG
								$display("%2x: CALLG %x", ir[3:0], data);
								`endif
								if (f[POSITIVE] == 1 && f[ZERO] == 0) begin
									case (mc_count)
										0: begin
											// step 1: we try to obtain  memory content from the next cell
											// xx is already on the data bus
											mbr <= data;
											// push to the stack the return value
											memwr <= 1'b1;
											memrd <= 1'b0;
											addr <= regs[SP] >> 1;
											regs[SP] <= regs[SP] + 2'd2;
											// the return value is in the pc + 2 
											data_to_write <= pc + 2'd2;
											// move to the next step
											mc_count <= 1;
										end
										1: begin
											// jump to the location in mbr
											memwr <= 1'b0;
											memrd <= 1'b1;
											pc <= mbr;
											addr <= mbr >> 1;
											ir <= 0;
										end
										default: begin
										end
									endcase
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >> 1;
									ir <= 0;
								end
							end // end of CALLG xx
							// CALLSE xx - call smaller or equal
							4'b1010: begin
								`ifdef DEBUG
								$display("%2x: CALLSE %x", ir[3:0], data);
								`endif
								if (f[POSITIVE] == 0 || f[ZERO] == 1) begin
									case (mc_count)
										0: begin
											// step 1: we try to obtain  memory content from the next cell
											// xx is already on the data bus
											mbr <= data;
											// push to the stack the return value
											memwr <= 1'b1;
											memrd <= 1'b0;
											addr <= regs[SP] >> 1;
											regs[SP] <= regs[SP] + 2'd2;
											// the return value is in the pc + 2 
											data_to_write <= pc + 2'd2;
											// move to the next step
											mc_count <= 1;
										end
										1: begin
											// jump to the location in mbr
											memwr <= 1'b0;
											memrd <= 1'b1;
											pc <= mbr;
											addr <= mbr >> 1;
											ir <= 0;
										end
										default: begin
										end
									endcase
								end
								else begin
									pc <= pc + 2'd2;
									addr <= (pc + 2'd2) >> 1;
									ir <= 0;
								end
							end // end of CALLSE xx
					endcase
					end // end of GROUP - 2 (CALL)
					
					// GROUP - 3 (LOAD, STORE)
					4'b0011: begin
						case (ir[7:4]) 
							// LD regx, [regy]
							4'b0000: begin
								`ifdef DEBUG
								$display("%2x: LD r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
								`endif
								case (mc_count)
									0: begin
											// step 1: we try to read memory from the regy address
											addr <= regs[ir[15:12]] >> 1;
											mc_count <= 1;
									end
									1: begin
										// step 2: we get the memory content from the data bus and put it in the regx
										regs[ir[11:8]] <= data;
										ir <= 0;     // initiate fetch
										addr <= pc >> 1;
										// pc already points to the next instruction
									end
									default: begin
									end
								endcase // mc_count
							end	// end of LD regx, [regy]
							// LD regx, [xx]
							4'b0001: begin
								`ifdef DEBUG
								$display("%2x: LD r%-d, [%x]", ir[3:0], (ir[11:8]), data);
								`endif
								case (mc_count)
									0: begin
											// step 1: we try to read memory from the xx address
											addr <= data >> 1;
											mc_count <= 1;
											pc <= pc + 2'd2;  // move to the next instruction
									end
									1: begin
										// step 2: we get the memory content from the data bus and put it in the regx
										regs[ir[11:8]] <= data;
										ir <= 0;     // initiate fetch
										addr <= pc >> 1;  // put the pc to the data bus
										// pc points to the next instruction
									end
									default: begin
									end
								endcase // mc_count
							end	// end of LD regx, [xx]
							// LD regx, [regy + xx]
							4'b0010: begin
								`ifdef DEBUG
								$display("%2x: LD r%-d, [r%-d + %x]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
								`endif
								case (mc_count)
									0: begin
											// step 1: we try to read memory from the regy+xx address
											addr <= (data + regs[ir[15:12]]) >> 1;
											mc_count <= 1;
											pc <= pc + 2'd2;  // move to the next instruction
									end
									1: begin
										// step 2: we get the memory content from the data bus and put it in the regx
										regs[ir[11:8]] <= data;
										ir <= 0;     // initiate fetch
										addr <= pc >> 1;
										// pc already points to the next instruction
									end
									default: begin
									end
								endcase // mc_count
							end	 // end of LD regx, [regy + xx]
							// LD.B regx, [regy]
							4'b0011: begin
								`ifdef DEBUG
								$display("%2x: LD.B r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
								`endif
								case (mc_count)
									0: begin
											// step 1: we try to read memory from the regy address
											addr <= regs[ir[15:12]] >> 1;
											mbr <= regs[ir[15:12]];
											mc_count <= 1;
									end
									1: begin
										// step 2: we get the memory content from the data bus and put it in the regx
										if (mbr[0] == 1) begin
											// odd address
											regs[ir[11:8]] <= {8'd0, data[7:0]};
										end
										else begin
											// even address
											regs[ir[11:8]] <= {8'd0, data[15:8]};
										end
										ir <= 0;     // initiate fetch
										addr <= pc >> 1;
										// pc already points to the next instruction
									end
									default: begin
									end
								endcase // mc_count
							end	// end of LD.B regx, [regy]
							// LD.B regx, [xx]
							4'b0100: begin
								`ifdef DEBUG
								$display("%2x: LD.B r%-d, [%x]", ir[3:0], (ir[11:8]), data);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= data >> 1;
										mbr <= data;
										mc_count <= 1;
										pc <= pc + 2'd2;  // move to the next instruction
									end
									1: begin
										// step 2: we get the memory content from the data bus and put it in the regx
										if (mbr[0] == 1) begin
											// odd address
											regs[ir[11:8]] <= {8'd0, data[7:0]};
										end
										else begin
											// even address
											regs[ir[11:8]] <= {8'd0, data[15:8]};
										end
										ir <= 0;     // initiate fetch
										addr <= pc >> 1;  // put the pc to the data bus
										// pc points to the next instruction
									end
									default: begin
									end
								endcase // mc_count
							end	// end of LD.B regx, [xx]
							// LD.B regx, [regy + xx]
							4'b0101: begin
								`ifdef DEBUG
								$display("%2x: LD.B r%-d, [r%-d + %x]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
								`endif
								case (mc_count)
									0: begin
											// step 1: we try to read memory from the regy+xx address
											addr <= (data + regs[ir[15:12]]) >> 1;
											mbr <= data + regs[ir[15:12]];
											mc_count <= 1;
											pc <= pc + 2'd2;  // move to the next instruction
									end
									1: begin
										// step 2: we get the memory content from the data bus and put it in the regx
										if (mbr[0] == 1) begin
											// odd address
											regs[ir[11:8]] <= {8'd0, data[7:0]};
										end
										else begin
											// even address
											regs[ir[11:8]] <= {8'd0, data[15:8]};
										end
										ir <= 0;     // initiate fetch
										addr <= pc >> 1;
										// pc already points to the next instruction
									end
									default: begin
									end
								endcase // mc_count
							end	 // end of LD.B regx, [regy + xx]
							// ST [regy], regx
							4'b1000: begin
								`ifdef DEBUG
								$display("%2x: ST [r%-d], r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to write into the memory defined by the regy 
										memrd <= 0;
										memwr <= 1;
										// put regy to the addr
										addr <= regs[ir[11:8]] >> 1;
										data_to_write <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										// step 2: initiate fetch
										ir <= 0;    
										memrd <= 1;
										memwr <= 0;
										// pc already points to the next instruction
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase // mc_count
							end	 // end of ST [regy], regx
							// ST [xx], reg
							4'b1001: begin
								`ifdef DEBUG
								$display("%2x: ST [%x], r%-d", ir[3:0], data, (ir[11:8]));
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to obtain  memory content from the next cell
										// xx is already on the data bus
										mbr <= data;
										mc_count <= 1;
										// move pc to the next instruction
										pc <= pc + 2'd2;
									end
									1: begin
										// then we try to write to the xx
										addr <= mbr >> 1;
										memrd <= 1'b0;
										memwr <= 1'b1;
										data_to_write <= regs[ir[11:8]];
										mc_count <= 2;
									end
									2: begin
										// step 2: initiate fetch
										ir <= 0;     
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= pc >> 1;
										// pc already points to the next instruction
									end
									default: begin
									end
								endcase // mc_count
							end	 // end of ST [xx], reg
							// ST [regy + xx], regx
							4'b1010: begin
								`ifdef DEBUG
								$display("%2x: ST [r%-d + %x], r%d", ir[3:0], (ir[11:8]), data, (ir[15:12]));
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to obtain  memory content from the next cell
										// xx is already on the data bus
										mbr <= data;
										mc_count <= 1;
										// move pc to the next instruction
										pc <= pc + 2'd2;
									end
									1: begin
										// then we try to write to the reg + xx
										addr <= (regs[ir[11:8]] + mbr) >> 1;
										memrd <= 1'b0;
										memwr <= 1'b1;
										data_to_write <= regs[ir[15:12]];
										mc_count <= 2;
									end
									2: begin
										// step 2: initiate fetch
										ir <= 0;     
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= pc >> 1;
										// pc already points to the next instruction
									end
									default: begin
									end
								endcase // mc_count
							end	 // end of ST [regy + xx], regx
							// ST.B [regy], regx
							4'b1011: begin
								`ifdef DEBUG
								$display("%2x: ST.B [r%-d], r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								`endif
								case (mc_count)
									0: begin
										// step 1: we read the destination memory content
										memrd <= 1'b1;
										memwr <= 1'b0;
										// put regy to the addr
										addr <= regs[ir[11:8]] >> 1;
										mc_count <= 1;										
									end
									1: begin
										// step 2: we try to write into the memory defined by the regy 
										memrd <= 1'b0;
										memwr <= 1'b1;
										// put regy to the addr
										addr <= regs[ir[11:8]] >> 1;
										if (regs[ir[11:8]][0] == 1'b1) begin
											data_to_write <= {data[15:8], regs[ir[15:12]][7:0]};
										end
										else begin
											data_to_write <= {regs[ir[15:12]][7:0], data[7:0]};
										end
										mc_count <= 2;
									end
									2: begin
										// step 2: initiate fetch
										ir <= 0;    
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= pc >> 1;
										// pc already points to the next instruction
									end
									default: begin
									end
								endcase // mc_count
							end	// end of ST.B [regy], regx
							// ST.B [xx], reg
							4'b1100: begin
								`ifdef DEBUG
								$display("%2x: ST.B [%x], r%-d", ir[3:0], data, (ir[11:8]));
								`endif
								case (mc_count)
									0: begin
										// step 1: we read the destination memory content
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= data >> 1;
										mbr <= data;
										mc_count <= 1;										
										// move pc to the next instruction
										pc <= pc + 2'd2;
									end
									1: begin
										// then we try to write to the xx
										addr <= mbr >> 1;
										memrd <= 0;
										memwr <= 1;
										if (mbr[0] == 1'b1) begin
											data_to_write <= {data[15:8], regs[ir[15:12]][7:0]};
										end
										else begin
											data_to_write <= {regs[ir[15:12]][7:0], data[7:0]};
										end
										mc_count <= 2;
									end
									2: begin
										// step 2: initiate fetch
										ir <= 0;     
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= pc >> 1;
										// pc already points to the next instruction
									end
									default: begin
									end
								endcase // mc_count
							end	 // end of ST.B [xx], reg
							// ST.B [reg + xx], reg
							4'b1101: begin
								`ifdef DEBUG
								$display("%2x: ST.B [r%-d + %x], r%d", ir[3:0], (ir[11:8]), data, (ir[15:12]));
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to obtain  memory content from the given address
										// the constant part of the address is already on the data bus
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= (regs[ir[11:8]] + data) >> 1;
										mbr <= (regs[ir[11:8]] + data);
										mc_count <= 1;
										// move pc to the next instruction
										pc <= pc + 2'd2;
									end
									1: begin
										// then we try to write to the reg + xx
										addr <= mbr >> 1;
										memrd <= 1'b0;
										memwr <= 1'b1;
										if (mbr[0] == 1'b1) begin
											data_to_write <= {data[15:8], regs[ir[15:12]][7:0]};
										end
										else begin
											data_to_write <= {regs[ir[15:12]][7:0], data[7:0]};
										end
										mc_count <= 2;
									end
									2: begin
										// step 2: initiate fetch
										ir <= 0;     
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= pc >> 1;
										// pc already points to the next instruction
									end
									default: begin
									end
								endcase // mc_count
							end	 // end of ST.B [reg + xx], reg
						endcase // ir [7:4] 
					end // end of GROUP - 3 (LOAD, STORE)
					
					// GROUP - 4, 5, 6, 7, 8 (ADD, SUB, AND, OR, XOR, SHL, SHR, MUL, DIV)
					4'b0100, 4'b0101, 4'b0110, 4'b0111, 4'b1000: begin
						case (ir[7:4]) 
							// ADD/AND/XOR/SHL/MUL regx, regy
							4'b0000: begin
								`ifdef DEBUG
									case (ir[3:0])
									4: $display("%2x: ADD r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
									5: $display("%2x: AND r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
									6: $display("%2x: XOR r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
									7: $display("%2x: SHL r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
									8: $display("%2x: MUL r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
									endcase
								`endif
								case (mc_count)
									0: begin
										case (ir[3:0])
											4: alu_op <= ALU_ADD;
											5: alu_op <= ALU_AND;
											6: alu_op <= ALU_XOR;
											7: alu_op <= ALU_SHL;
											8: alu_op <= ALU_MUL;
											default: alu_op <= 0;
										endcase
										alu_a <= regs[ir[11:8]];
										alu_b <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										regs[H] <= alu_high;
										ir <= 0;     // initiate fetch
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of ADD/AND/XOR/SHL/MUL regx, regy
							// ADD/AND/XOR/SHL/MUL reg, xx
							4'b0001: begin
								`ifdef DEBUG
								case (ir[3:0])
									4: $display("%2x: ADD r%-d, %-d", ir[3:0], (ir[11:8]), data);
									5: $display("%2x: AND r%-d, %-d", ir[3:0], (ir[11:8]), data);
									6: $display("%2x: XOR r%-d, %-d", ir[3:0], (ir[11:8]), data);
									7: $display("%2x: SHL r%-d, %-d", ir[3:0], (ir[11:8]), data);
									8: $display("%2x: MUL r%-d, %-d", ir[3:0], (ir[11:8]), data);
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
								`endif
								case (mc_count)
									0: begin
										case (ir[3:0])
											4: alu_op <= ALU_ADD;
											5: alu_op <= ALU_AND;
											6: alu_op <= ALU_XOR;
											7: alu_op <= ALU_SHL;
											8: alu_op <= ALU_MUL;
											default: alu_op <= 0;
										endcase
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 1;
										pc <= pc + 2'd2;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										regs[H] <= alu_high;
										ir <= 0;     // initiate fetch
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of ADD/AND/XOR/SHL/MUL reg, xx
							// ADD/AND/XOR/SHL/MUL regx, [regy]
							4'b0010: begin
								`ifdef DEBUG
								case (ir[3:0])
									4: $display("%2x: ADD r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									5: $display("%2x: AND r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									6: $display("%2x: XOR r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									7: $display("%2x: SHL r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									8: $display("%2x: MUL r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy address
										addr <= regs[ir[15:12]] >> 1;
										mc_count <= 1;
									end
									1: begin
										case (ir[3:0])
											4: alu_op <= ALU_ADD;
											5: alu_op <= ALU_AND;
											6: alu_op <= ALU_XOR;
											7: alu_op <= ALU_SHL;
											8: alu_op <= ALU_MUL;
											default: alu_op <= 0;
										endcase
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										regs[H] <= alu_high;
										ir <= 0;     // initiate fetch
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of ADD/AND/XOR/SHL/MUL regx, [regy]
							// ADD/AND/XOR/SHL/MUL reg, [xx]
							4'b0011: begin
								`ifdef DEBUG
								case (ir[3:0])
									4: $display("%2x: ADD r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									5: $display("%2x: AND r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									6: $display("%2x: XOR r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									7: $display("%2x: SHL r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									8: $display("%2x: MUL r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= data >> 1;
										mc_count <= 1;
										pc <= pc + 2'd2;  // move to the next instruction
									end
									1: begin
										case (ir[3:0])
											4: alu_op <= ALU_ADD;
											5: alu_op <= ALU_AND;
											6: alu_op <= ALU_XOR;
											7: alu_op <= ALU_SHL;
											8: alu_op <= ALU_MUL;
											default: alu_op <= 0;
										endcase
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										regs[H] <= alu_high;
										ir <= 0;     // initiate fetch
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of ADD/AND/XOR/SHL/MUL reg, [xx]
							// ADD/AND/XOR/SHL/MUL regx, [regy + xx]
							4'b0100: begin
								`ifdef DEBUG
								case (ir[3:0])
									4: $display("%2x: ADD r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									5: $display("%2x: AND r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									6: $display("%2x: XOR r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									7: $display("%2x: SHL r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									8: $display("%2x: MUL r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= (regs[ir[15:12]] + data) >> 1;
										mc_count <= 1;
										pc <= pc + 2'd2;  // move to the next instruction
									end
									1: begin
										case (ir[3:0])
											4: alu_op <= ALU_ADD;
											5: alu_op <= ALU_AND;
											6: alu_op <= ALU_XOR;
											7: alu_op <= ALU_SHL;
											8: alu_op <= ALU_MUL;
											default: alu_op <= 0;
										endcase
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										regs[H] <= alu_high;
										ir <= 0;     // initiate fetch
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of ADD/AND/XOR/SHL/MUL regx, [regy + xx]
							// ADD/AND/XOR/SHL/MUL.B regx, [regy]
							4'b0101: begin
								`ifdef DEBUG
								case (ir[3:0])
									4: $display("%2x: ADD.B r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									5: $display("%2x: AND.B r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									6: $display("%2x: XOR.B r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									7: $display("%2x: SHL.B r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									8: $display("%2x: MUL.B r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy address
										addr <= regs[ir[15:12]] >> 1;
										mc_count <= 1;
									end
									1: begin
										case (ir[3:0])
											4: alu_op <= ALU_ADD;
											5: alu_op <= ALU_AND;
											6: alu_op <= ALU_XOR;
											7: alu_op <= ALU_SHL;
											8: alu_op <= ALU_MUL;
											default: alu_op <= 0;
										endcase
										alu_a <= regs[ir[11:8]];
										if (regs[ir[15:12]][0] == 1) begin
											// odd address
											alu_b <= {8'd0, data[7:0]};
										end
										else begin
											alu_b <= {8'd0, data[15:8]};
										end
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										regs[H] <= alu_high;
										ir <= 0;     // initiate fetch
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of ADD/AND/XOR/SHL/MUL.B regx, [regy]
							// ADD/AND/XOR/SHL/MUL.B reg, [xx]
							4'b0110: begin
								`ifdef DEBUG
								case (ir[3:0])
									4: $display("%2x: ADD.B r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									5: $display("%2x: AND.B r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									6: $display("%2x: XOR.B r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									7: $display("%2x: SHL.B r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									8: $display("%2x: MUL.B r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= data >> 1;
										mbr <= data;
										mc_count <= 1;
										pc <= pc + 2'd2;  // move to the next instruction
									end
									1: begin
										case (ir[3:0])
											4: alu_op <= ALU_ADD;
											5: alu_op <= ALU_AND;
											6: alu_op <= ALU_XOR;
											7: alu_op <= ALU_SHL;
											8: alu_op <= ALU_MUL;
											default: alu_op <= 0;
										endcase
										alu_a <= regs[ir[11:8]];
										if (mbr[0] == 1) begin
											// odd address
											alu_b <= {8'd0, data[7:0]};
										end
										else begin
											alu_b <= {8'd0, data[15:8]};
										end
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										regs[H] <= alu_high;
										ir <= 0;     // initiate fetch
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of ADD/AND/XOR/SHL/MUL.B reg, [xx]
							// ADD/AND/XOR/SHL/MUL.B regx, [regy + xx]
							4'b0111: begin
								`ifdef DEBUG
								case (ir[3:0])
									4: $display("%2x: ADD.B r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									5: $display("%2x: AND.B r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									6: $display("%2x: XOR.B r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									7: $display("%2x: SHL.B r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									8: $display("%2x: MUL.B r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= (regs[ir[15:12]] + data) >> 1;
										mbr <= regs[ir[15:12]] + data;
										mc_count <= 1;
										pc <= pc + 2'd2;  // move to the next instruction
									end
									1: begin
										case (ir[3:0])
											4: alu_op <= ALU_ADD;
											5: alu_op <= ALU_AND;
											6: alu_op <= ALU_XOR;
											7: alu_op <= ALU_SHL;
											8: alu_op <= ALU_MUL;
											default: alu_op <= 0;
										endcase
										alu_a <= regs[ir[11:8]];
										if (mbr[0] == 1) begin
											// odd address
											alu_b <= {8'd0, data[7:0]};
										end
										else begin
											alu_b <= {8'd0, data[15:8]};
										end
										mc_count <= 2;
									end
									2: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										regs[H] <= alu_high;
										ir <= 0;     // initiate fetch
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of ADD/AND/XOR/SHL/MUL.B regx, [regy + xx]
							// SUB/OR/SHR/DIV regx, regy
							4'b1000: begin
								`ifdef DEBUG
								case (ir[3:0])
									4: $display("%2x: SUB r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
									5: $display("%2x: OR r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
									7: $display("%2x: SHR r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
									8: $display("%2x: DIV r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
								`endif
								case (mc_count)
									0: begin
										case (ir[3:0])
											4: alu_op <= ALU_SUB;
											5: alu_op <= ALU_OR;
											7: alu_op <= ALU_SHR;
											8: alu_op <= ALU_DIV;
											default: alu_op <= 0;
										endcase
										alu_a <= regs[ir[11:8]];
										alu_b <= regs[ir[15:12]];
										if (ir[3:0] == 8'd8) begin
											// DIV only
											start_div <= 1'b1;
										end
										mc_count <= 1;
									end
									1: begin
										if (ir[3:0] == 8'd8) begin
											// DIV only
											start_div <= 1'b0;
											if (div_finished) begin
												regs[ir[11:8]] <= alu_res;
												f <= f_from_alu;
												regs[H] <= alu_high;
												ir <= 0;     // initiate fetch
												addr <= pc >> 1;
											end
										end
										else begin
											regs[ir[11:8]] <= alu_res;
											f <= f_from_alu;
											ir <= 0;     // initiate fetch
											addr <= pc >> 1;
										end
									end
									default: begin
									end
								endcase
							end // end of SUB/OR/SHR/DIV regx, regx
							// SUB/OR/SHR/DIV reg, xx
							4'b1001: begin
								`ifdef DEBUG
								case (ir[3:0])
									4: $display("%2x: SUB r%-d, %-d", ir[3:0], (ir[11:8]), data);
									5: $display("%2x: OR r%-d, %-d", ir[3:0], (ir[11:8]), data);
									7: $display("%2x: SHR r%-d, %-d", ir[3:0], (ir[11:8]), data);
									8: $display("%2x: DIV r%-d, %-d", ir[3:0], (ir[11:8]), data);
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
								`endif
								case (mc_count)
									0: begin
										case (ir[3:0])
											4: alu_op <= ALU_SUB;
											5: alu_op <= ALU_OR;
											7: alu_op <= ALU_SHR;
											8: alu_op <= ALU_DIV;
											default: alu_op <= 0;
										endcase
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										if (ir[3:0] == 8'd8) begin
											// DIV only
											start_div <= 1'b1;
										end
										mc_count <= 1;
										pc <= pc + 2'd2;
									end
									1: begin
										if (ir[3:0] == 8'd8) begin
											// DIV only
											start_div <= 1'b0;
											if (div_finished) begin
												regs[ir[11:8]] <= alu_res;
												f <= f_from_alu;
												regs[H] <= alu_high;
												ir <= 0;     // initiate fetch
												addr <= pc >> 1;
											end
										end
										else begin
											regs[ir[11:8]] <= alu_res;
											f <= f_from_alu;
											ir <= 0;     // initiate fetch
											addr <= pc >> 1;
										end
									end
									default: begin
									end
								endcase
							end // end of SUB/OR/SHR/DIV reg, xx
							// SUB/OR/SHR/DIV regx, [regy]
							4'b1010: begin
								`ifdef DEBUG
								case (ir[3:0])
									4: $display("%2x: SUB r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									5: $display("%2x: OR r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									7: $display("%2x: SHR r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									8: $display("%2x: DIV r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy address
										addr <= regs[ir[15:12]] >> 1;
										mc_count <= 1;
									end
									1: begin
										case (ir[3:0])
											4: alu_op <= ALU_SUB;
											5: alu_op <= ALU_OR;
											7: alu_op <= ALU_SHR;
											8: alu_op <= ALU_DIV;
											default: alu_op <= 0;
										endcase
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										if (ir[3:0] == 8'd8) begin
											// DIV only
											start_div <= 1'b1;
										end
										mc_count <= 2;
									end
									2: begin
										if (ir[3:0] == 8'd8) begin
											// DIV only
											start_div <= 1'b0;
											if (div_finished) begin
												regs[ir[11:8]] <= alu_res;
												f <= f_from_alu;
												regs[H] <= alu_high;
												ir <= 0;     // initiate fetch
												addr <= pc >> 1;
											end
										end
										else begin
											regs[ir[11:8]] <= alu_res;
											f <= f_from_alu;
											ir <= 0;     // initiate fetch
											addr <= pc >> 1;
										end
									end
									default: begin
									end
								endcase
							end // end of SUB/OR/SHR/DIV regx, [regy]
							// SUB/OR/SHR/DIV reg, [xx]
							4'b1011: begin
								`ifdef DEBUG
								case (ir[3:0])
									4: $display("%2x: SUB r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									5: $display("%2x: OR r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									7: $display("%2x: SHR r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									8: $display("%2x: DIV r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= data >> 1;
										mc_count <= 1;
										pc <= pc + 2'd2;  // move to the next instruction
									end
									1: begin
										case (ir[3:0])
											4: alu_op <= ALU_SUB;
											5: alu_op <= ALU_OR;
											7: alu_op <= ALU_SHR;
											8: alu_op <= ALU_DIV;
											default: alu_op <= 0;
										endcase
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										if (ir[3:0] == 8'd8) begin
											// DIV only
											start_div <= 1'b1;
										end
										mc_count <= 2;
									end
									2: begin
										if (ir[3:0] == 8'd8) begin
											// DIV only
											start_div <= 1'b0;
											if (div_finished) begin
												regs[ir[11:8]] <= alu_res;
												f <= f_from_alu;
												regs[H] <= alu_high;
												ir <= 0;     // initiate fetch
												addr <= pc >> 1;
											end
										end
										else begin
											regs[ir[11:8]] <= alu_res;
											f <= f_from_alu;
											ir <= 0;     // initiate fetch
											addr <= pc >> 1;
										end
									end
									default: begin
									end
								endcase
							end // end of SUB/OR/SHR/DIV reg, [xx]
							// SUB/OR/SHR/DIV regx, [regy + xx]
							4'b1100: begin
								`ifdef DEBUG
								case (ir[3:0])
									4: $display("%2x: SUB r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									5: $display("%2x: OR r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									7: $display("%2x: SHR r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									8: $display("%2x: DIV r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= (regs[ir[15:12]] + data) >> 1;
										mc_count <= 1;
										pc <= pc + 2'd2;  // move to the next instruction
									end
									1: begin
										case (ir[3:0])
											4: alu_op <= ALU_SUB;
											5: alu_op <= ALU_OR;
											7: alu_op <= ALU_SHR;
											8: alu_op <= ALU_DIV;
											default: alu_op <= 0;
										endcase
										alu_a <= regs[ir[11:8]];
										alu_b <= data;
										if (ir[3:0] == 8'd8) begin
											// DIV only
											start_div <= 1'b1;
										end
										mc_count <= 2;
									end
									2: begin
										if (ir[3:0] == 8'd8) begin
											// DIV only
											start_div <= 1'b0;
											if (div_finished) begin
												regs[ir[11:8]] <= alu_res;
												f <= f_from_alu;
												regs[H] <= alu_high;
												ir <= 0;     // initiate fetch
												addr <= pc >> 1;
											end
										end
										else begin
											regs[ir[11:8]] <= alu_res;
											f <= f_from_alu;
											ir <= 0;     // initiate fetch
											addr <= pc >> 1;
										end
									end
									default: begin
									end
								endcase
							end // end of SUB/OR/SHR/DIV regx, [regy + xx]
							// SUB/OR/SHR/DIV.B regx, [regy]
							4'b1101: begin
								`ifdef DEBUG
								case (ir[3:0])
									4: $display("%2x: SUB.B r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									5: $display("%2x: OR.B r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									7: $display("%2x: SHR.B r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									8: $display("%2x: DIV.B r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy address
										addr <= regs[ir[15:12]] >> 1;
										mc_count <= 1;
									end
									1: begin
										case (ir[3:0])
											4: alu_op <= ALU_SUB;
											5: alu_op <= ALU_OR;
											7: alu_op <= ALU_SHR;
											8: alu_op <= ALU_DIV;
											default: alu_op <= 0;
										endcase
										alu_a <= regs[ir[11:8]];
										if (regs[ir[15:12]][0] == 1) begin
											// odd address
											alu_b <= {8'd0, data[7:0]};
										end
										else begin
											alu_b <= {8'd0, data[15:8]};
										end
										if (ir[3:0] == 8'd8) begin
											// DIV only
											start_div <= 1'b1;
										end
										mc_count <= 2;
									end
									2: begin
										if (ir[3:0] == 8'd8) begin
											// DIV only
											start_div <= 1'b0;
											if (div_finished) begin
												regs[ir[11:8]] <= alu_res;
												f <= f_from_alu;
												regs[H] <= alu_high;
												ir <= 0;     // initiate fetch
												addr <= pc >> 1;
											end
										end
										else begin
											regs[ir[11:8]] <= alu_res;
											f <= f_from_alu;
											ir <= 0;     // initiate fetch
											addr <= pc >> 1;
										end
									end
									default: begin
									end
								endcase
							end // end of SUB/OR/SHR/DIV.B regx, [regy]
							// SUB/OR/SHR/DIV.B reg, [xx]
							4'b1110: begin
								`ifdef DEBUG
								case (ir[3:0])
									4: $display("%2x: SUB.B r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									5: $display("%2x: OR.B r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									7: $display("%2x: SHR.B r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									8: $display("%2x: DIV.B r%-d, [%-d]", ir[3:0], (ir[11:8]), data);
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= data >> 1;
										mbr <= data;
										mc_count <= 1;
										pc <= pc + 2'd2;  // move to the next instruction
									end
									1: begin
										case (ir[3:0])
											4: alu_op <= ALU_SUB;
											5: alu_op <= ALU_OR;
											7: alu_op <= ALU_SHR;
											8: alu_op <= ALU_DIV;
											default: alu_op <= 0;
										endcase
										alu_a <= regs[ir[11:8]];
										if (mbr[0] == 1) begin
											// odd address
											alu_b <= {8'd0, data[7:0]};
										end
										else begin
											alu_b <= {8'd0, data[15:8]};
										end
										if (ir[3:0] == 8'd8) begin
											// DIV only
											start_div <= 1'b1;
										end
										mc_count <= 2;
									end
									2: begin
										if (ir[3:0] == 8'd8) begin
											// DIV only
											start_div <= 1'b0;
											if (div_finished) begin
												regs[ir[11:8]] <= alu_res;
												f <= f_from_alu;
												regs[H] <= alu_high;
												ir <= 0;     // initiate fetch
												addr <= pc >> 1;
											end
										end
										else begin
											regs[ir[11:8]] <= alu_res;
											f <= f_from_alu;
											ir <= 0;     // initiate fetch
											addr <= pc >> 1;
										end
									end
									default: begin
									end
								endcase
							end // end of SUB/OR/SHR/DIV.B reg, [xx]
							// SUB/OR/SHR/DIV.B regx, [regy + xx]
							4'b1111: begin
								`ifdef DEBUG
								case (ir[3:0])
									4: $display("%2x: SUB.B r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									5: $display("%2x: SUB.B r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									7: $display("%2x: SUB.B r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									8: $display("%2x: DIV.B r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= (regs[ir[15:12]] + data) >> 1;
										mbr <= regs[ir[15:12]] + data;
										mc_count <= 1;
										pc <= pc + 2'd2;  // move to the next instruction
									end
									1: begin
										case (ir[3:0])
											4: alu_op <= ALU_SUB;
											5: alu_op <= ALU_OR;
											7: alu_op <= ALU_SHR;
											8: alu_op <= ALU_DIV;
											default: alu_op <= 0;
										endcase
										alu_a <= regs[ir[11:8]];
										if (mbr[0] == 1) begin
											// odd address
											alu_b <= {8'd0, data[7:0]};
										end
										else begin
											alu_b <= {8'd0, data[15:8]};
										end
										if (ir[3:0] == 8'd8) begin
											// DIV only
											start_div <= 1'b1;
										end
										mc_count <= 2;
									end
									2: begin
										if (ir[3:0] == 8'd8) begin
											// DIV only
											start_div <= 1'b0;
											if (div_finished) begin
												regs[ir[11:8]] <= alu_res;
												f <= f_from_alu;
												regs[H] <= alu_high;
												ir <= 0;     // initiate fetch
												addr <= pc >> 1;
											end
										end
										else begin
											regs[ir[11:8]] <= alu_res;
											f <= f_from_alu;
											ir <= 0;     // initiate fetch
											addr <= pc >> 1;
										end
									end
									default: begin
									end
								endcase
							end // end of SUB/OR/SHR/DIV.B regx, [regy + xx]
						endcase	
					end // end of GROUP - 4, 5, 6, 7, 8 (ADD, SUB, AND, OR, XOR, SHL, SHR, MUL, DIV)

					// GROUP - 9 (INC, DEC)
					4'b1001: begin 
						case (ir[7:4])
							// INC/DEC reg
							4'b0000, 4'b1000: begin
								`ifdef DEBUG
								if (ir[7] == 0)
									$display("%2x: INC r%-d", ir[3:0], (ir[11:8]));
								else
									$display("%2x: DEC r%-d", ir[3:0], (ir[11:8]));
								`endif
								case (mc_count)
									0: begin
										if (ir[7] == 0)
											alu_op <= ALU_ADD;
										else
											alu_op <= ALU_SUB;
										alu_a <= regs[ir[11:8]];
										alu_b <= 1;
										mc_count <= 1;
									end
									1: begin
										regs[ir[11:8]] <= alu_res;
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of INC/DEC reg
							// INC/DEC [reg]
							4'b0001, 4'b1001: begin
								`ifdef DEBUG
								if (ir[7] == 0)
									$display("%2x: INC [r%-d]", ir[3:0], (ir[11:8]));
								else
									$display("%2x: DEC [r%-d]", ir[3:0], (ir[11:8]));
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy address
										addr <= regs[ir[11:8]] >> 1;
										mc_count <= 1;
									end
									1: begin
										if (ir[7] == 0)
											alu_op <= ALU_ADD;
										else
											alu_op <= ALU_SUB;
										alu_a <= data;
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the reg
										memrd <= 1'b0;
										memwr <= 1'b1;
										// put regy to the addr
										addr <= regs[ir[11:8]] >> 1;
										data_to_write <= alu_res;
										mc_count <= 3;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of INC/DEC [reg]
							// INC/DEC [XX]
							4'b0010, 4'b1010: begin
								`ifdef DEBUG
								if (ir[7] == 0)
									$display("%2x: INC [%d]", ir[3:0], data);
								else
									$display("%2x: DEC [%d]", ir[3:0], data);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= data >> 1;
										mbr <= data;
										mc_count <= 1;
									end
									1: begin
										if (ir[7] == 0)
											alu_op <= ALU_ADD;
										else
											alu_op <= ALU_SUB;
										alu_a <= data;
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the XX
										memrd <= 1'b0;
										memwr <= 1'b1;
										// put regy to the addr
										addr <= mbr >> 1;
										data_to_write <= alu_res;
										mc_count <= 3;
										pc <= pc + 2'd2;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of INC/DEC [XX]
							// INC/DEC [reg + XX]
							4'b0011, 4'b1011: begin
								`ifdef DEBUG
								if (ir[7] == 0)
									$display("%2x: INC [r%-d + %d]", ir[3:0], ir[11:8], data);
								else
									$display("%2x: DEC [r%-d + %d]", ir[3:0], ir[11:8], data);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= (regs[ir[11:8]] + data) >> 1;
										mbr <= regs[ir[11:8]] + data;
										mc_count <= 1;
									end
									1: begin
										if (ir[7] == 0)
											alu_op <= ALU_ADD;
										else
											alu_op <= ALU_SUB;
										alu_a <= data;
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the XX
										memrd <= 1'b0;
										memwr <= 1'b1;
										// put regy + data to the addr
										addr <= mbr >> 1;
										data_to_write <= alu_res;
										mc_count <= 3;
										pc <= pc + 2'd2;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of INC/DEC [reg + xx]
							// INC.B/DEC.B [reg]
							4'b0100, 4'b1100: begin
								`ifdef DEBUG
								if (ir[7] == 0)
									$display("%2x: INC.B [r%-d]", ir[3:0], (ir[11:8]));
								else
									$display("%2x: DEC.B [r%-d]", ir[3:0], (ir[11:8]));
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy address
										addr <= regs[ir[11:8]] >> 1;
										mbr <= regs[ir[11:8]];
										mc_count <= 1;
									end
									1: begin
										if (ir[7] == 0)
											alu_op <= ALU_ADD;
										else
											alu_op <= ALU_SUB;
										
										if (mbr[0] == 1) begin
											// odd address
											alu_a <= {8'd0, data[7:0]};
										end
										else begin
											alu_a <= {8'd0, data[15:8]};
										end
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the reg
										memrd <= 1'b0;
										memwr <= 1'b1;
										// put regy to the addr
										addr <= regs[ir[11:8]] >> 1;
										if (mbr[0] == 1) begin
											// odd address
											data_to_write <= {data[15:8], alu_res[7:0]};
										end
										else begin
											data_to_write <= {alu_res[7:0], data[7:0]};
										end
										mc_count <= 3;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of INC/DEC [reg]
							// INC.B/DEC.B [XX]
							4'b0101, 4'b1101: begin
								`ifdef DEBUG
								if (ir[7] == 0)
									$display("%2x: INC.B [%d]", ir[3:0], data);
								else
									$display("%2x: DEC.B [%d]", ir[3:0], data);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= data >> 1;
										mbr <= data;
										mc_count <= 1;
									end
									1: begin
										if (ir[7] == 0)
											alu_op <= ALU_ADD;
										else
											alu_op <= ALU_SUB;
										if (mbr[0] == 1) begin
											// odd address
											alu_a <= {8'd0, data[7:0]};
										end
										else begin
											alu_a <= {8'd0, data[15:8]};
										end
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the XX
										memrd <= 1'b0;
										memwr <= 1'b1;
										// put regy to the addr
										addr <= mbr >> 1;
										if (mbr[0] == 1) begin
											// odd address
											data_to_write <= {data[15:8], alu_res[7:0]};
										end
										else begin
											data_to_write <= {alu_res[7:0], data[7:0]};
										end
										mc_count <= 3;
										pc <= pc + 2'd2;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of INC/DEC [XX]
							// INC.B/DEC.B [reg + XX]
							4'b0110, 4'b1110: begin
								`ifdef DEBUG
								if (ir[7] == 0)
									$display("%2x: INC.B [r%-d + %d]", ir[3:0], ir[11:8], data);
								else
									$display("%2x: DEC.B [r%-d + %d]", ir[3:0], ir[11:8], data);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= (regs[ir[11:8]] + data) >> 1;
										mbr <= regs[ir[11:8]] + data;
										mc_count <= 1;
									end
									1: begin
										if (ir[7] == 0)
											alu_op <= ALU_ADD;
										else
											alu_op <= ALU_SUB;
										if (mbr[0] == 1) begin
											// odd address
											alu_a <= {8'd0, data[7:0]};
										end
										else begin
											alu_a <= {8'd0, data[15:8]};
										end
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the XX
										memrd <= 1'b0;
										memwr <= 1'b1;
										// put regy + data to the addr
										addr <= mbr >> 1;
										if (mbr[0] == 1) begin
											// odd address
											data_to_write <= {data[15:8], alu_res[7:0]};
										end
										else begin
											data_to_write <= {alu_res[7:0], data[7:0]};
										end
										mc_count <= 3;
										pc <= pc + 2'd2;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of INC/DEC [reg + xx]
							default: begin
							end
						endcase // ir[7:4]
					end // end of GROUP - 9 (INC, DEC)
					
					// GROUP - 10 (CMP)
					4'b1010: begin 
						case (ir[7:4])
							// CMP regx, regy
							4'b0000: begin
								`ifdef DEBUG
								$display("%2x: CMP r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								`endif
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
										addr <= pc >> 1;
									end
								endcase
							end // end of CMP regx, regy
							// CMP reg, xx
							4'b0001: begin
								`ifdef DEBUG
								$display("%2x: CMP r%-d, %-d", ir[3:0], (ir[11:8]), data);
								`endif
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
										pc <= pc + 2'd2;
										addr <= (pc + 2'd2) >> 1;
									end
								endcase
							end // end of CMP regx, regy
							// CMP regx, [regy]
							4'b0010: begin
								`ifdef DEBUG
								$display("%2x: CMP r%-d, [r%-d]", ir[3:0], (ir[11:8]), ir[15:12]);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy address
										addr <= regs[ir[15:12]] >> 1;
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
										pc <= pc + 2'd2;
										addr <= (pc + 2'd2) >> 1;
									end
								endcase
							end // end of CMP regx, [regy]
							// CMP regx, [XX]
							4'b0011: begin
								`ifdef DEBUG
								$display("%2x: CMP r%-d, [%d]", ir[3:0], (ir[11:8]), data);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= data >> 1;
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
										pc <= pc + 2'd2;
										addr <= (pc + 2'd2) >> 1;
									end
								endcase
							end // end of CMP regx, [XX]
							// CMP regx, [regy + XX]
							4'b0100: begin
								`ifdef DEBUG
								$display("%2x: CMP r%-d, [r%-d + %d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= (regs[ir[15:12]] + data) >> 1;
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
										pc <= pc + 2'd2;
										addr <= (pc + 2'd2) >> 1;
									end
								endcase
							end // end of CMP regx, [regy + XX]
							// CMP.B regx, [regy]
							4'b0101: begin
								`ifdef DEBUG
								$display("%2x: CMP.B r%-d, [r%-d]", ir[3:0], (ir[11:8]), ir[15:12]);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy address
										addr <= regs[ir[15:12]] >> 1;
										mbr <= regs[ir[15:12]];
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_SUB;
										alu_a <= regs[ir[11:8]];
										if (mbr[0] == 1) begin
											// odd address
											alu_b <= {8'd0, data[7:0]};
										end
										else begin
											alu_b <= {8'd0, data[15:8]};
										end
										mc_count <= 2;
									end
									2: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										pc <= pc + 2'd2;
										addr <= (pc + 2'd2) >> 1;
									end
								endcase
							end // end of CMP.B regx, [regy]
							// CMP.B regx, [XX]
							4'b0110: begin
								`ifdef DEBUG
								$display("%2x: CMP.B r%-d, [%d]", ir[3:0], (ir[11:8]), data);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= data >> 1;
										mbr <= data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_SUB;
										alu_a <= regs[ir[11:8]];
										if (mbr[0] == 1) begin
											// odd address
											alu_b <= {8'd0, data[7:0]};
										end
										else begin
											alu_b <= {8'd0, data[15:8]};
										end
										mc_count <= 2;
									end
									2: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										pc <= pc + 2'd2;
										addr <= (pc + 2'd2) >> 1;
									end
								endcase
							end // end of CMP.B regx, [XX]
							// CMP.B regx, [regy + XX]
							4'b0111: begin
								`ifdef DEBUG
								$display("%2x: CMP.B r%-d, [r%-d + %d]", ir[3:0], (ir[11:8]), (ir[15:12]), data);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= (regs[ir[15:12]] + data) >> 1;
										mbr <= regs[ir[15:12]] + data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_SUB;
										alu_a <= regs[ir[11:8]];
										if (mbr[0] == 1) begin
											// odd address
											alu_b <= {8'd0, data[7:0]};
										end
										else begin
											alu_b <= {8'd0, data[15:8]};
										end
										mc_count <= 2;
									end
									2: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										pc <= pc + 2'd2;
										addr <= (pc + 2'd2) >> 1;
									end
								endcase
							end // end of CMP.B regx, [regy + XX]
							// NEG reg
							4'b1000: begin
								`ifdef DEBUG
								$display("%2x: NEG r%-d", ir[3:0], (ir[11:8]));
								`endif
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
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of NEG reg
							// NEG [reg]
							4'b1001: begin
								`ifdef DEBUG
								$display("%2x: NEG [r%-d]", ir[3:0], (ir[11:8]));
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy address
										addr <= regs[ir[11:8]] >> 1;
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
										memrd <= 1'b0;
										memwr <= 1'b1;
										// put regy to the addr
										addr <= regs[ir[11:8]];
										data_to_write <= alu_res;
										mc_count <= 3;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of NEG [reg]
							// NEG [XX]
							4'b1010: begin
								`ifdef DEBUG
								$display("%2x: NEG [%d]", ir[3:0], data);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= data >> 1;
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
										memrd <= 1'b0;
										memwr <= 1'b1;
										// put regy to the addr
										addr <= mbr >> 1;
										data_to_write <= alu_res;
										mc_count <= 3;
										pc <= pc + 2'd2;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1;
										memwr <= 0;
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of NEG [XX]
							// NEG [reg + XX]
							4'b1011: begin
								`ifdef DEBUG
								$display("%2x: NEG [r%-d + %d]", ir[3:0], ir[11:8], data);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= (regs[ir[11:8]] + data) >> 1;
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
										memrd <= 1'b0;
										memwr <= 1'b1;
										// put regy + data to the addr
										addr <= mbr >> 1;
										data_to_write <= alu_res;
										mc_count <= 3;
										pc <= pc + 2'd2;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of NEG [reg + xx]
							// NEG.B [reg]
							4'b1100: begin
								`ifdef DEBUG
								$display("%2x: NEG.B [r%-d]", ir[3:0], (ir[11:8]));
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the regy address
										addr <= regs[ir[11:8]] >> 1;
										mbr <= regs[ir[11:8]];
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_NEG;
										if (mbr[0] == 1) begin
											// odd address
											alu_a <= {8'd0, data[7:0]};
										end
										else begin
											alu_a <= {8'd0, data[15:8]};
										end
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the reg
										memrd <= 1'b0;
										memwr <= 1'b1;
										// put regy to the addr
										addr <= regs[ir[11:8]];
										if (mbr[0] == 1) begin
											// odd address
											data_to_write <= {data[15:8], alu_res[7:0]};
										end
										else begin
											data_to_write <= {alu_res[7:0], data[7:0]};
										end
										mc_count <= 3;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of NEG.B [reg]
							// NEG.B [XX]
							4'b1101: begin
								`ifdef DEBUG
								$display("%2x: NEG.B [%d]", ir[3:0], data);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= data >> 1;
										mbr <= data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_NEG;
										if (mbr[0] == 1) begin
											// odd address
											alu_a <= {8'd0, data[7:0]};
										end
										else begin
											alu_a <= {8'd0, data[15:8]};
										end
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the XX
										memrd <= 1'b0;
										memwr <= 1'b1;
										// put regy to the addr
										addr <= mbr >> 1;
										if (mbr[0] == 1) begin
											// odd address
											data_to_write <= {data[15:8], alu_res[7:0]};
										end
										else begin
											data_to_write <= {alu_res[7:0], data[7:0]};
										end
										mc_count <= 3;
										pc <= pc + 2'd2;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1;
										memwr <= 0;
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of NEG [XX]
							// NEG.B [reg + XX]
							4'b1110: begin
								`ifdef DEBUG
								$display("%2x: NEG.B [r%-d + %d]", ir[3:0], ir[11:8], data);
								`endif
								case (mc_count)
									0: begin
										// step 1: we try to read memory from the xx address
										addr <= (regs[ir[11:8]] + data) >> 1;
										mbr <= regs[ir[11:8]] + data;
										mc_count <= 1;
									end
									1: begin
										alu_op <= ALU_NEG;
										if (mbr[0] == 1) begin
											// odd address
											alu_a <= {8'd0, data[7:0]};
										end
										else begin
											alu_a <= {8'd0, data[15:8]};
										end
										alu_b <= 1;
										mc_count <= 2;
									end
									2: begin
										// step 2: we try to write ALU_RES into the memory defined by the XX
										memrd <= 1'b0;
										memwr <= 1'b1;
										// put regy + data to the addr
										addr <= mbr >> 1;
										if (mbr[0] == 1) begin
											// odd address
											data_to_write <= {data[15:8], alu_res[7:0]};
										end
										else begin
											data_to_write <= {alu_res[7:0], data[7:0]};
										end
										mc_count <= 3;
										pc <= pc + 2'd2;
									end
									3: begin
										f <= f_from_alu;
										ir <= 0;     // initiate fetch
										memrd <= 1'b1;
										memwr <= 1'b0;
										addr <= pc >> 1;
									end
									default: begin
									end
								endcase
							end // end of NEG [reg + xx]
						endcase // ir[7:4]
					end // end of CMP/NEG GROUP
					
				endcase	// {ir[3:0]}  - opcode
			end
		endcase // ir - fetch vs. opcode
	end
	$display("high=%4x\tpc=%4x", regs[H], pc);
	$display("f=%2x\t\tsp=%4x", f, regs[SP]);
	$display("r0=%4x\t\tir=%4x", regs[0], ir);
	$display("r1=%4x\t\tmc_count=%x", regs[1], mc_count);
	$display("r2=%4x", regs[2]);
	$display("r3=%4x\t\taddr=%4x", regs[3],addr);
	$display("r4=%4x\t\tdata=%4x/%4x", regs[4], data, data_to_write);
	$display("r5=%4x\t\tmr/mw=%x/%x", regs[5], memrd, memwr);
	$display("r6=%4x\t\tmbr=%x", regs[6], mbr);
	$display("r7=%4x\n", regs[7]);

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

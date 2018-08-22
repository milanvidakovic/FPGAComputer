
//=======================================================
//  This code is generated by Terasic System Builder
//=======================================================

module vga_320x240#(parameter N = 16)(
	input wire enable,
	//////////// CLOCK //////////
	input CLOCK_50,

	//////////// reset //////////
	input reset,

	//////////// GPIO //////////
	output wire rr, 
	output wire rg, 
	output wire rb, 
	output wire hs, 
	output wire vs,

	input [N-1:0] data,
	output [N-1:0] raddr,
	output rrd, 
	output rwr
);

//=======================================================
//  PARAMETER declarations
//=======================================================

localparam VIDEO_MEM_ADDR = 1200;

//=======================================================
//  PORT declarations
//=======================================================


//=======================================================
//  REG/WIRE declarations
//=======================================================

reg clk25; // 25MHz signal (clk divided by 2)
//reg newframe;
//reg newline;

reg [9:0] x;
reg [9:0] y;
wire valid;
reg [15:0] curr_char;

reg [9:0] xx;
reg [9:0] yy;

wire [15:0] pixels; // Pixels making up one row of the character 

//assign hs = x < (640 + 16) || x >= (640 + 16 + 96);
//assign vs = y < (480 + 10) || y >= (480 + 10 + 2);
assign valid = (x < 640) && (y < 480);
assign xx = x >> 1;
assign yy = y >> 1;

reg r, g, b;
reg [N-1:0] addr;
reg rd; 
reg wr;
assign raddr = enable ? addr : {N{1'bZ}};
assign rrd = enable ? rd : 1'bZ;
assign rwr = enable ? wr : 1'bZ;

assign hs = enable ? x < (640 + 16) || x >= (640 + 16 + 96) : 1'bZ;
assign vs = enable ? y < (480 + 10) || y >= (480 + 10 + 2)  : 1'bZ;
assign rr = enable ? r : 1'bZ;
assign rg = enable ? g : 1'bZ;
assign rb = enable ? b : 1'bZ;

reg [3:0]count_read;
reg mem_read;

always @(posedge CLOCK_50) begin
//	newframe <= 0;
//	newline <= 0;
	if (reset) begin
		x <= 10'b0;
		y <= 10'b0;
		clk25 <= 1'b0;
//		newframe <= 1;
//		newline <= 1;
	end 
	else begin
		clk25 <= ~clk25;
		if (clk25 == 1'b1) begin
			if (x < 10'd799) begin
				x <= x + 1'b1;
			end 
			else begin
				x <= 10'b0;
//				newline <= 1;
				if (y < 10'd524) begin
					y <= y + 1'b1;
				end 
				else begin
					y <= 10'b0;
//					newframe <= 1;
				end
			end
			if (mem_read) begin
				pixels <= data;
				rd <= 1'bz;
				wr <= 1'bz;
				mem_read <= 1'b0;
			end
		end 
		else begin
			// this is the other cycle when we divide 50MHz
			if (x >= 640) begin
				if (!mem_read) begin
					if ((x >= 640) && (y >= 480)) begin
						// when we start the vertical blanking, we need to fetch in advance the first word at (0, 0)
						rd <= 1'b1;
						wr <= 1'b0;
						mem_read <= 1'b1;
						addr <= VIDEO_MEM_ADDR + 0;
					end
					else if ((x >= 640) && (y < 480)) begin
						// when we start the horizontal blanking, and we need to go to the next line, 
						// we need to fetch in advance the first word in next line (0, y+1)
						rd <= 1'b1;
						wr <= 1'b0;
						mem_read <= 1'b1;
						if ((y & 1) == 1) begin
							addr <= VIDEO_MEM_ADDR + ((yy + 1) * 80);
						end
						else begin
							addr <= VIDEO_MEM_ADDR + ((yy) * 80);
						end
					end
				end
			end // if (!valid)
			// from this moment on, x and y are valid
			else if (x < 640 && !mem_read) begin
				if ((x & 7) == 7)  begin
					// when we are finishing current word, containing four pixels, we need to fetch in advance the next word (x+1, y)
					// at the last pixel of the current character, let's fetch next
					rd <= 1'b1;
					wr <= 1'b0;
					addr <= VIDEO_MEM_ADDR + ((xx >> 2) + (yy * 80) + 1);
					mem_read <= 1'b1;
				end
			end 
			 
		end
	end
	
	if (valid) begin
		r <= pixels[12 - ((xx & 3) << 2) + 0] == 1'b1;
		g <= pixels[12 - ((xx & 3) << 2) + 1] == 1'b1;
		b <= pixels[12 - ((xx & 3) << 2) + 2] == 1'b1;
	end
	else begin
		// blanking -> no pixels
		r <= 1'b0;
		g <= 1'b0;
		b <= 1'b0;
	end
end

initial begin
		x <= 10'b0;
		y <= 10'b0;
		clk25 <= 1'b0;
//		newframe <= 1;
//		newline <= 1;
end

endmodule
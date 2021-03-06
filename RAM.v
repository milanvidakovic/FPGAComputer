module RAM#(parameter N = 16)
(
	input  clk,
	output [N-1:0] data,
	input [N-1:0] addr,
	input rd, wr,
	input [N-1:0] indata,
	
	output [N-1:0] data2,
	input [N-1:0] addr2,
	input rd2
);
	// Declare the RAM variable
	reg [N-1:0] ram[32767:0];

	
	always @ (negedge clk)
	begin

		if (wr && !rd) 
		begin
			ram[addr] <= indata;
			data <= {N{1'bz}};
		end
		else if (!wr && rd) begin
			data <= ram[addr];
		end
		else begin
			data <= {N{1'bz}};
		end
		
		if (rd2) begin
			data2 <= ram[addr2];
		end
		else begin
			data2 <= {N{1'bz}};
		end
		
		$display("MEMORY addr:%-x > %-x, %-x, \t %-x, %-x", addr, ram[addr], ram[addr + 1], ram[addr + 2], ram[addr + 3]);
		
	end	
	
initial
begin
  $readmemh("ram.hex", ram);
/*
  ram[0] <= "W";
	ram[1] <= "O";
	ram[2] <= "R";
	ram[3] <= "D";
*/	
end

endmodule 
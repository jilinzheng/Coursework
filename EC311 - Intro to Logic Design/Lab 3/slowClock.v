`timescale 1ns / 1ps

// 4 Hz Clock
module slowClock(
        input in_clk,
        output reg out_clk
    );
    reg [25:0] count;
    
    initial begin
		// Initialize everything to zero
		count = 0;
		out_clk = 0;
	end
    
    always @ (posedge in_clk) begin
        count = count + 1;
        if (count == 12_500_000) begin // CHANGE TO LOW VALUE I.E. 2 FOR SIMULATIONS/TESTBENCHES
            out_clk = !out_clk;
            count <= 0;
        end
    end
endmodule
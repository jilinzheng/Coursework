`timescale 1ns / 1ps

// NOTE: TESTBENCH WILL NOT WORK UNLESS CLOCK SPEED IS RAISED BY MODIFYING slowClock.v
// I.E. CHANGE THE IF CONDITIONAL in slowClock.v TO (count == 2) (or something close to it)

module slowClock_test(

    );
    
    reg in_clk;
    wire out_clk;
    
    slowClock theSlowClock (
        .in_clk(in_clk),
        .out_clk(out_clk)
    );
    
    // Clock generator
    always #1 in_clk = ~in_clk;
    
    initial begin
        in_clk = 0;
    end
    initial #100 $finish;
    
endmodule
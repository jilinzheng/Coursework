`timescale 1ns / 1ps

// NOTE: TESTBENCH WILL NOT WORK UNLESS CLOCK SPEED IS RAISED BY MODIFYING slowClock.v
// I.E. CHANGE THE IF CONDITIONAL in slowClock.v TO (count == 2) (or something close to it)

module debouncingPushButton_test(

    );
    
    reg pb;
    reg in_clk;
    wire led;
    
    debouncingPushButton debounce(.pb(pb), .in_clk(in_clk), .led(led));
    
    // Clock generator
    always #1 in_clk = ~in_clk;
    
    initial begin
        in_clk = 0;
        pb = 0;
                
        #10 pb = 1; // Push the button
        #10 pb = 0; // Let go
        #10 pb = 1; // Push the button
        #20 pb = 0; // Let go
    end
        
endmodule

`timescale 1ns / 1ps

// Source Note:
// This code and its submodules are obtained from
// https://www.youtube.com/watch?v=LO8ONR1TceI
module debouncingPushButton(
        input pb,
        input in_clk,
        output led
    );
    
    wire out_clk;
    wire Q1, Q2, Q2bar;
    
    slowClock theSlowClock(in_clk,out_clk);
    dff d1(out_clk,pb,Q1);
    dff d2(out_clk,Q1,Q2);
    
    assign Q2bar = ~Q2;
    assign led = Q1 & Q2bar;
    
endmodule
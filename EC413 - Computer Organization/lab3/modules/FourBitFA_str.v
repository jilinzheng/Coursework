`timescale 1ns / 1ps

// 4-bit Full Adder/Ripple Carry Adder made from four 1-bit Full Adders.

//`define USE_DELAYED_GATES // comment/uncomment directive to use gates WITHOUT/WITH delay

module FourBitFA_str(
    input [3:0] a, b,
    input c_in,
    output [3:0] sum,
    output c_out
    );
    
    wire [2:0] c;
    
    `ifdef USE_DELAYED_GATES
        FA_str FA1 (
            .c_out(c[0]),
            .sum(sum[0]),
            .a(a[0]),
            .b(b[0]),
            .c_in(c_in)
        );
        
        FA_str FA2 (
            .c_out(c[1]),
            .sum(sum[1]),
            .a(a[1]),
            .b(b[1]),
            .c_in(c[0])
        );
        
        FA_str FA3 (
            .c_out(c[2]),
            .sum(sum[2]),
            .a(a[2]),
            .b(b[2]),
            .c_in(c[1])
        );
        
        FA_str FA4 (
            .c_out(c_out),
            .sum(sum[3]),
            .a(a[3]),
            .b(b[3]),
            .c_in(c[2])
        );
    `else
        FullAdder FA1 (
            .c_out(c[0]),
            .sum(sum[0]),
            .a(a[0]),
            .b(b[0]),
            .c_in(c_in)
        );
        
        FullAdder FA2 (
            .c_out(c[1]),
            .sum(sum[1]),
            .a(a[1]),
            .b(b[1]),
            .c_in(c[0])
        );
        
        FullAdder FA3 (
            .c_out(c[2]),
            .sum(sum[2]),
            .a(a[2]),
            .b(b[2]),
            .c_in(c[1])
        );
        
        FullAdder FA4 (
            .c_out(c_out),
            .sum(sum[3]),
            .a(a[3]),
            .b(b[3]),
            .c_in(c[2])
        );
    `endif

endmodule

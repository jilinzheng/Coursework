`timescale 1ns / 1ps

// 64-bit Ripple Carry Adder made from two 32-bit Ripple Carry Adders.

module SixtyFourBitRCA(
    input [63:0] a, b,
    input c_in,
    output [63:0] sum,
    output c_out
    );

    wire c;

    ThirtyTwoBitRCA ThirtyTwoBitRCA1 (
        .c_out(c),
        .sum(sum[31:0]),
        .a(a[31:0]),
        .b(b[31:0]),
        .c_in(c_in)
    );

    ThirtyTwoBitRCA ThirtyTwoBitRCA2 (
        .c_out(c_out),
        .sum(sum[63:32]),
        .a(a[63:32]),
        .b(b[63:32]),
        .c_in(c)
    );

endmodule

`timescale 1ns / 1ps

// 32-bit Ripple Carry Adder made from two 16-bit Ripple Carry Adders.

module ThirtyTwoBitRCA(
    input [31:0] a, b,
    input c_in,
    output [31:0] sum,
    output c_out
    );

    wire c;

    SixteenBitRCA SixteenBitRCA1 (
        .c_out(c),
        .sum(sum[15:0]),
        .a(a[15:0]),
        .b(b[15:0]),
        .c_in(c_in)
    );

    SixteenBitRCA SixteenBitRCA2 (
        .c_out(c_out),
        .sum(sum[31:16]),
        .a(a[31:16]),
        .b(b[31:16]),
        .c_in(c)
    );

endmodule

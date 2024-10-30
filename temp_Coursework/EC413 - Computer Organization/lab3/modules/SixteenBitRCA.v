`timescale 1ns / 1ps

// 16-bit Ripple Carry Adder made from two 8-bit Ripple Carry Adders.

module SixteenBitRCA(
    input [15:0] a, b,
    input c_in,
    output [15:0] sum,
    output c_out
    );

    wire c;

    EightBitRCA EightBitRCA1 (
        .c_out(c),
        .sum(sum[7:0]),
        .a(a[7:0]),
        .b(b[7:0]),
        .c_in(c_in)
    );

    EightBitRCA EightBitRCA2 (
        .c_out(c_out),
        .sum(sum[15:8]),
        .a(a[15:8]),
        .b(b[15:8]),
        .c_in(c)
    );

endmodule

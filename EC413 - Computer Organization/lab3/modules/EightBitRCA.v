`timescale 1ns / 1ps

// 8-bit Ripple Carry Adder made from two 4-bit Full Adders.

module EightBitRCA(
    input [7:0] a, b,
    input c_in,
    output [7:0] sum,
    output c_out
    );

    wire c;

    FourBitFA_str FourBitRCA1 (
        .c_out(c),
        .sum(sum[3:0]),
        .a(a[3:0]),
        .b(b[3:0]),
        .c_in(c_in)
    );

    FourBitFA_str FourBitRCA2 (
        .c_out(c_out),
        .sum(sum[7:4]),
        .a(a[7:4]),
        .b(b[7:4]),
        .c_in(c)
    );

endmodule

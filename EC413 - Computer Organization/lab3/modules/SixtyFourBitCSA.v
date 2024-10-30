`timescale 1ns / 1ps

// 64-bit Carry Select Adder, implemented with three 32-bit Ripple Carry Adders.

module SixtyFourBitCSA(
    input [63:0] a, b,
    input c_in,
    output [63:0] sum,
    output c_out
    );

    wire [2:0] c;                               // to hold c_out between RCAs
    wire [31:0] carry_zero_sum, carry_one_sum;  // to hold sums of the hard-wired RCAs

    // lower 32 bits
    ThirtyTwoBitRCA ThirtyTwoBitRCA1 (
        .c_out(c[0]),
        .sum(sum[31:0]),
        .a(a[31:0]),
        .b(b[31:0]),
        .c_in(c_in)
    );

    // hard-wired to 0 c_in
    ThirtyTwoBitRCA ThirtyTwoBitRCA2 (
        .c_out(c[1]),
        .sum(carry_zero_sum[31:0]),
        .a(a[63:32]),
        .b(b[63:32]),
        .c_in(1'b0)
    );

    // hard-wired to 1 c_in
    ThirtyTwoBitRCA ThirtyTwoBitRCA3 (
        .c_out(c[2]),
        .sum(carry_one_sum[31:0]),
        .a(a[63:32]),
        .b(b[63:32]),
        .c_in(1'b1)
    );

    // muxes for the higher order 32 bits of the sum and the carry
    assign sum[63:32] = c[0] ? carry_one_sum[31:0] : carry_zero_sum[31:0];
    assign c_out = c[0] ? c[2] : c[1];

endmodule

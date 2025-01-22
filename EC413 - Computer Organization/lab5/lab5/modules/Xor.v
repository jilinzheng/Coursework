`timescale 1ns / 1ps

// Helper XOR module (implemented using standard primitives) for SLT function.

module Xor(
    input a, b,
    output out
    );

    wire not_a, not_b;
    wire res1, res2;

    not not1 (not_a, a);
    not not2 (not_b, b);

    and and1 (res1, a, not_b);
    and and2 (res2, not_a, b);

    or or1 (out, res1, res2);

endmodule

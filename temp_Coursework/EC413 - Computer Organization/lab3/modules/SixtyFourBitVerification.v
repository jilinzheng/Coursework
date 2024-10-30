`timescale 1ns / 1ps

// 64-bit adder verification using behavioral Verilog.

module SixtyFourBitVerification(
    input [63:0] a, b,
    input c_in,
    output [63:0] sum,
    output c_out
    );

    assign {c_out, sum} = a + b + c_in;

endmodule

`timescale 1ns / 1ps

// Parameterized n-bit register module, using 1-bit register (D Flip FLop) bit slices.
// Note that DFF is clocked, so q (output) is delayed from d (input).

module NBitReg #(parameter n = 8) (
    input clk,
    input [n-1:0] in,
    output [n-1:0] out
    );

    genvar i;
    generate
        for (i = 0; i < n; i = i + 1) begin: Registers
            dff DFF (
                .q(out[i]),
                .d(in[i]),
                .clk(clk)
            );
        end
    endgenerate

endmodule

`timescale 1ns / 1ps

// Parameterized n-bit MOV module, using 1-bit OR bit slices.
// Note that one input of OR is fixed to 0, to preserve R2's values.

module NBitMov #(parameter n = 8) (
    input [n-1:0] R2,
    output [n-1:0] R1
    );

    genvar i;
    generate
    for (i = 0; i < n; i = i + 1) begin: Movs
        or Mov (R1[i], R2[i], 1'b0);
    end
    endgenerate

endmodule

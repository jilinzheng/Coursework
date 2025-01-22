`timescale 1ns / 1ps

// Parameterized n-bit AND module, using 1-bit AND bit slices.

module NBitAnd #(parameter n = 8) (
    input [n-1:0] R2, R3,
    output [n-1:0] R1
    );
    
    genvar i;
    generate
    for (i = 0; i < n; i = i + 1) begin: Ands
        and And (R1[i], R2[i], R3[i]);
    end
    endgenerate
    
endmodule

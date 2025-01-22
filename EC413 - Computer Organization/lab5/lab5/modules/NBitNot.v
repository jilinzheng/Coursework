`timescale 1ns / 1ps

// Parameterized n-bit NOT module, using 1-bit NOT bit slices.

module NBitNot #(parameter n = 8) (
    input [n-1:0] R2,
    output [n-1:0] R1
    );
    
    genvar i;
    generate
    for (i = 0; i < n; i = i + 1) begin: Nots
        not Not (R1[i], R2[i]);
    end
    endgenerate
    
endmodule

`timescale 1ns / 1ps

// Parameterized n-bit ADD module, using 1-bit ADD bit slices.

module NBitAdder #(parameter n = 8) (
    input [n-1:0] a, b,
    input c_in,
    output [n-1:0] sum,
    output c_out
    );

    wire [n:0] c;   // for the carries between FAs

    genvar i;
    generate
        for (i = 0; i < n; i = i + 1) begin: Adders
            FA_str FA (
                .c_out(c[i+1]),
                .sum(sum[i]),
                .a(a[i]),
                .b(b[i]),
                .c_in(c[i])
            );
        end
    endgenerate
    
    assign c[0] = c_in;
    assign c_out = c[n];

endmodule

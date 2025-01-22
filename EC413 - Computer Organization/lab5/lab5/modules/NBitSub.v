`timescale 1ns / 1ps

// Parameterized n-bit SUB module, using 1-bit SUB bit slices, implemented using 2's Complement addition.
// Note that c_in is hard-wired to 1, when instantiated in NBitALU.v

module NBitSub #(parameter n = 8) (
    input [n-1:0] a, b,
    input c_in,
    output [n-1:0] sum,
    output c_out
    );

    wire [n:0] c;       // for the carries between FAs
    wire [n-1:0] b_inv; // invert b for 2's complement subtraction 
                        // a + b_inv + 1

    genvar i;
    generate
        // invert second input (b) for 2's complement
        for (i = 0; i < n; i = i + 1) begin: Nots
            not Not (b_inv[i], b[i]);
        end

        // 2's Complement addition for subtraction
        for (i = 0; i < n; i = i + 1) begin: Adders
            FA_str FA (
                .c_out(c[i+1]),
                .sum(sum[i]),
                .a(a[i]),
                .b(b_inv[i]),
                .c_in(c[i])
            );
        end
    endgenerate

    assign c[0] = c_in;
    assign c_out = c[n];

endmodule

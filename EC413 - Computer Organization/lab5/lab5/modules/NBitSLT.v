`timescale 1ns / 1ps

// Parameterized n-bit SLT module, using 1-bit SLT bit slices.

module NBitSLT #(n = 4) (
    input [n-1:0] R2, R3,
    output [n-1:0] R1
    );

    wire [n-1:0] sub_res, xnor_res;
    wire [n-1:0] R3_inv;            // invert b for 2's complement subtraction 
                                    // a + b_inv + 1
    wire [n-1:0] c;                 // for the carries between FAs
    wire overflow;

    genvar i;

    // invert second input (b) for 2's complement addition
    generate
        for (i = 0; i < n; i = i + 1) begin: Nots
            not Not (R3_inv[i], R3[i]);
        end    
    endgenerate

    // hard-wire c_in = 1 for first FA for 2's complement addition
    FA_str FA0 (
        .c_out(c[0]),
        .sum(sub_res[0]),
        .a(R2[0]),
        .b(R3_inv[0]),
        .c_in(1)
    );

    // subtraction logic
    generate
        for (i = 1; i < n; i = i + 1) begin: Adders
            FA_str FA (
                .c_out(c[i]),
                .sum(sub_res[i]),
                .a(R2[i]),
                .b(R3_inv[i]),
                .c_in(c[i-1])
            );
        end

    endgenerate

    // detect overflow by XORing carry out and carry in of MSB of R2 - R3 subtraction result
    // overflow occurs when carry out and carry in of MSB differs
    // reference: https://uweb.engr.arizona.edu/~ece369/Resources/overflow.pdf
    Xor XOR (
        .a(c[n-1]),
        .b(c[n-2]),
        .out(overflow)
    );

    // MUX to drive R1 appropriately depending on overflow or sign of result
    assign R1 = overflow ? 
        R2[n-1] :
        sub_res[n-1] ? 1'b1 : 1'b0;

endmodule

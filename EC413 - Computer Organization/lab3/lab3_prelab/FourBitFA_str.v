`timescale 1ns / 1ps

module FourBitFA_str(
    input [3:0] a, b,
    input c_in,
    output [3:0] sum,
    output c_out
    );
    
    wire [2:0] c;
    
    FA_str FA1 (
        .c_out(c[0]),
        .sum(sum[0]),
        .a(a[0]),
        .b(b[0]),
        .c_in(c_in)
    );
    
    FA_str FA2 (
        .c_out(c[1]),
        .sum(sum[1]),
        .a(a[1]),
        .b(b[1]),
        .c_in(c[0])
    );
    
    FA_str FA3 (
        .c_out(c[2]),
        .sum(sum[2]),
        .a(a[2]),
        .b(b[2]),
        .c_in(c[1])
    );
    
    FA_str FA4 (
        .c_out(c_out),
        .sum(sum[3]),
        .a(a[3]),
        .b(b[3]),
        .c_in(c[2])
    );

endmodule

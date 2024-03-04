`timescale 1ns / 1ps

module RCA_verification #(parameter n = 1) (
        input [n-1:0] a, b,
        output [n:0] out
    );
    
    assign out = a + b;
endmodule

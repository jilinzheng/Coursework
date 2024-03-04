`timescale 1ns / 1ps

module RCA_Nbit #(parameter n = 1) (
        input [n-1:0] a,
        input [n-1:0] b,
        output [n:0] out
    );
    
    wire [n-1:0] sum;
    wire [n-1:0] c_out;
    
    genvar i;
    fullAdder FA(0, a[0], b[0], sum[0], c_out[0]);    // Hardwire c_in to 0
    generate
        for (i = 1; i < n; i = i + 1) begin
            fullAdder fa(c_out[i-1], a[i], b[i], sum[i], c_out[i]);
        end
    endgenerate
    
    assign out = {c_out[n-1], sum};
endmodule

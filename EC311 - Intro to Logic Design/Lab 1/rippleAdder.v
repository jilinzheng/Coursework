`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/18/2023 11:22:16 AM
// Design Name: 
// Module Name: rippleAdder
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module rippleAdder(
        input c_in,
        input [2:0] a,
        input [2:0] b,
        output [2:0] sum,
        output c_out
    );
    
    fullAdder firstFullAdder(.c_in(c_in),.a(a[0]),.b(b[0]),.sum(sum[0]),.c_out(w1));
    fullAdder secondFullAdder(.c_in(w1),.a(a[1]),.b(b[1]),.sum(sum[1]),.c_out(w2));
    fullAdder thirdFullAdder(.c_in(w2),.a(a[2]),.b(b[2]),.sum(sum[2]),.c_out(c_out));
    endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/12/2023 11:44:56 PM
// Design Name: 
// Module Name: carrySelectAdder
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


module carrySelectAdder( // 4-bit carry select adder
        input cIn,
        input [3:0] A,
        input [3:0] B,
        
        output finalCOut,
        output [3:0] finalSum
    );
    
    // Two 4-bit sums and cOuts (each) for the '0' and '1' lines of full adders
    wire [3:0] sums0, sums1, cOuts0, cOuts1;
    
    // 8 full adders for a 4-bit carry select adder
    // cIn = 0 line of full adders
    fullAdder FA0(.c_in(1'b0),.a(A[0]),.b(B[0]),.sum(sums0[0]),.c_out(cOuts0[0]));
    fullAdder FA1(.c_in(cOuts0[0]),.a(A[1]),.b(B[1]),.sum(sums0[1]),.c_out(cOuts0[1]));
    fullAdder FA2(.c_in(cOuts0[1]),.a(A[2]),.b(B[2]),.sum(sums0[2]),.c_out(cOuts0[2]));
    fullAdder FA3(.c_in(cOuts0[2]),.a(A[3]),.b(B[3]),.sum(sums0[3]),.c_out(cOuts0[3]));
    // cIn = 1 line of full adders
    fullAdder FA4(.c_in(1'b1),.a(A[0]),.b(B[0]),.sum(sums1[0]),.c_out(cOuts1[0]));
    fullAdder FA5(.c_in(cOuts1[0]),.a(A[1]),.b(B[1]),.sum(sums1[1]),.c_out(cOuts1[1]));
    fullAdder FA6(.c_in(cOuts1[1]),.a(A[2]),.b(B[2]),.sum(sums1[2]),.c_out(cOuts1[2]));
    fullAdder FA7(.c_in(cOuts1[2]),.a(A[3]),.b(B[3]),.sum(sums1[3]),.c_out(cOuts1[3]));
    
    // MUXes to select appropriate numbers for the finalSum
    assign finalSum[0] = cIn ? sums1[0] : sums0[0];
    assign finalSum[1] = cIn ? sums1[1] : sums0[1];
    assign finalSum[2] = cIn ? sums1[2] : sums0[2];
    assign finalSum[3] = cIn ? sums1[3] : sums0[3];
    
    // MUX to select appropriate number for finalCOut
    assign finalCOut = cIn ? cOuts1[3] : cOuts0[3];
    
endmodule

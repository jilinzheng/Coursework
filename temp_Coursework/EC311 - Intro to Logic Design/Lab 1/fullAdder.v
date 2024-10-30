`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/18/2023 10:39:52 AM
// Design Name: 
// Module Name: fullAdder
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


module fullAdder(
        input c_in,
        input a,
        input b,
        output sum,
        output c_out    
    );
    
    wire w1, w2, w3;
    
    halfAdder firstHalfAdder(.a(a), .b(b), .sum(w1), .c_out(w2));
    halfAdder secondHalfAdder(.a(c_in), .b(w1), .sum(sum),.c_out(w3));
    
    assign c_out = w3 || w2;
    
endmodule

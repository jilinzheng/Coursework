`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/02/2023 11:24:20 AM
// Design Name: 
// Module Name: fullALU
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


module fullALU(
        input [2:0] a,
        input [2:0] b,
        input [1:0] mode,
        input c_in,
        output c_out,
        output [2:0] finalResult
    );
    
    wire [2:0] xorOrAnd;
    assign xorOrAnd = mode[0] ? (a & b) : (a ^ b);
    
    wire [2:0] addOrSubtract;
    assign addOrSubtract = mode[0] ? (~b) : b;
    
    wire [2:0] sumAdd;
    wire w1;
    rippleAdder addRippleAdder(.c_in(c_in),.a(a),.b(addOrSubtract),.sum(sumAdd),.c_out(w1));
    wire [2:0] sumSubtract;
    wire w2;
    rippleAdder subtractRippleAdder(.c_in(mode[0]),.a(a),.b(addOrSubtract),.sum(sumSubtract),.c_out(w2));
    assign c_out = mode[0] ? w2 : w1;
    
    wire [2:0] sum;
    assign sum = mode[0] ? sumSubtract : sumAdd;
   
    wire [2:0] result;
    assign result = mode[1] ? xorOrAnd : sum;
    
    assign finalResult = result;
   
    //wire[2:0] bPrime;
    //assign bPrime = mode ? (~b) : b;
    
    // Sum needs to be intermediate now and we choose between sum or xorOrAnd
    //rippleAdder theRippleAdder(.c_in(c_in),.a(a),.b(b),.sum(result),.c_out(c_out));
    
endmodule

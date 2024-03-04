`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/02/2023 11:25:47 AM
// Design Name: 
// Module Name: fullALU_testbench
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


module fullALU_testbench(

    );
    
    reg c_in;
    reg[1:0] mode;
    reg[2:0] a, b;
    
    wire c_out;
    wire[2:0] finalResult;
    
    fullALU full(.c_in(c_in),.a(a),.b(b),.c_out(c_out),.mode(mode),.finalResult(finalResult));
    
    initial
    begin
        
    // Addition tests
    mode = 0;
    a = 4;
    b = 1;
    c_in = 0;
    #10 c_in = 1;
    #10 b = 4;
    
    // Subtraction tests
    #10 mode = 1;
    c_in = 0;
    a = 7;
    b = 1;
    #10 b = 3;
    #10 a = 1;
    
    // XOR tests
    #10 mode = 2;
    a = 3'b101;
    b = 3'b010;
    #10 a = 3'b001;
    b = 3'b101;
    
    // AND tests
    #10 mode = 3;
    a = 3'b101;
    b = 3'b010;
    #10 a = 3'b001;
    b = 3'b101;
    
    #10 $finish;
    end
    
endmodule

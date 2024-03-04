`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/18/2023 11:38:51 AM
// Design Name: 
// Module Name: rippleAdder_testbench
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


module rippleAdder_testbench(

    );
    reg c_in;
    reg[2:0] a, b;
    wire c_out;
    wire[2:0] sum;
    
    rippleAdder ripple(c_in,a,b,sum,c_out);
        
    initial
    begin
    c_in = 0;
    a = 0;
    b = 0;
    
    #10 a = 1;
    #10 b = 1;
    #10 a = 2;
    #10 b = 2;
    #10 a = 3;
    #10 b = 3;
    #10 a = 4;
    #10 b = 4;
    #10 a = 5;
    #10 b = 5;
    #10 a = 6;
    #10 b = 6;
    #10 a = 7;
    #10 b = 7;
    
    end
endmodule

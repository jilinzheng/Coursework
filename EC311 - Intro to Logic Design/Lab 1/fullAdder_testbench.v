`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/18/2023 10:58:05 AM
// Design Name: 
// Module Name: fullAdder_testbench
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


module fullAdder_testbench(

    );
    
    reg c_in, a, b;
    wire sum, c_out;
    
    fullAdder full(c_in,a,b,sum,c_out);
    
    initial
    begin
    c_in = 0;
    a = 0;
    b = 0; //000
    
    #10 c_in = 1; //100
    #10 a = 1; //110
    #10 b = 1; //111
    #10 c_in = 0; //011
    #10 a = 0; //001
    #10 c_in = 1; //101
    #10 a = 1;
    c_in = 0;
    b = 0; //010
    
    end
endmodule

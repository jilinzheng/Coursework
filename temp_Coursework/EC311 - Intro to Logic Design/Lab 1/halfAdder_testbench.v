`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/18/2023 10:28:37 AM
// Design Name: 
// Module Name: halfAdder_testbench
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


module halfAdder_testbench(

    );
    
    reg a,b;
    wire sum, c_out;
    
    halfAdder half(a,b,sum,c_out);
    
    initial
    begin
    a = 0;
    b = 0;
    
    #10 a = 1;
    #10 b = 1;
    #10 a = 0;
    #10 $finish;
    end
    
endmodule

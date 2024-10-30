`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/13/2023 12:20:48 AM
// Design Name: 
// Module Name: carrySelectAdder_testbench
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


module carrySelectAdder_testbench(

    );
    
    reg cIn;
    reg [3:0] A, B;
    
    wire finalCOut;
    wire [3:0] finalSum;
    
    carrySelectAdder theAdder(.cIn(cIn),.A(A),.B(B),.finalCOut(finalCOut),.finalSum(finalSum));
    
    // To make creating test cases convenient
    integer ii;
    initial
    begin
    
    cIn = 0;
    A = 0;
    B = 0;
    
    for (ii = 0; ii < 16; ii = ii + 1) begin
        #10 A = A + 1;
        #10 B = B + 1;
    end
    
    
    #10 cIn = 1;
    A = 15;
    B = 15;
    
    end
endmodule

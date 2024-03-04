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
    
    reg[3:0] A, B, opCode;
    reg cIn;
    
    wire[3:0] finalResult;
    wire cOut;
    
    fullALU full(.A(A),.B(B),.opCode(opCode),.cIn(cIn),.finalResult(finalResult),.cOut(cOut));
    
    // To easily loop through all sixteen operations
    integer ii;
    initial
    begin
    
    // Addition
    opCode = 4'b1010;
    A = 1;
    B = 3; // 1 + 3 = 4
    cIn = 0;
    #10;
    cIn = 1;
    A = 7;
    B = 7; // 7 + 7 + 1 = 15
    
    // Substraction
    #10;
    opCode = 4'b1011;
    B = 4; // 8 - 4 = 4
    #10;
    A = 2; // 2 - 4 = -2
    #10;
    
    // Loop through all operations
    for (ii = 0; ii < 16; ii = ii + 1) begin
        opCode = ii;
        #10;
    end
    
    end
    
endmodule

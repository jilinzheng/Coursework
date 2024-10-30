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
        input [3:0] opCode, // 4-bit operation code: MSB is the modeSelect, so remaining 3 bits = 8 operations
        input [3:0] A, // 4-bit input A
        input [3:0] B, // 4-bit input B
        input cIn, // 1-bit carry-in
        
        output [3:0] finalResult, // 4-bit output
        output cOut // 1-bit carry-out
    );
    
    wire modeSelect;
    assign modeSelect = opCode[3];
    
    wire [2:0] operation;
    assign operation = opCode[2:0];
    
    // Perform arithmetic mode operations
    reg [4:0] arithmeticResult; // One extra bit for carry out
    always @ (modeSelect, operation, A, B) begin
        case (operation)
            0: arithmeticResult = ~A + 1; // 2's Complement; flip bits and add 1
            1: arithmeticResult = ~B + 1;
            2: arithmeticResult = A + B + cIn;
            3: arithmeticResult = A - B;
            4: arithmeticResult = A * B;
            5: arithmeticResult = B - A;
            6: arithmeticResult = A + 1;
            7: arithmeticResult = A - 1;
        endcase
    end
    
    // Perform logical (bitwise) mode operations
    reg [3:0] logicalResult;
    always @ (modeSelect, operation, A, B) begin
        case (operation)
            0: logicalResult = A & B;
            1: logicalResult = A | B;
            2: logicalResult = ~(A & B);
            3: logicalResult = ~(A | B);
            4: logicalResult = A ^ B;
            5: logicalResult = A ~^ B;
            6: logicalResult = ~A;
            7: logicalResult = ~B;
        endcase
    end
    
    // MUX to choose between logical and arithmetic result
    assign finalResult = modeSelect ? arithmeticResult : logicalResult;
    
    // Set cOut to the MSB of arithmetic result
    assign cOut = arithmeticResult[4];
    
endmodule

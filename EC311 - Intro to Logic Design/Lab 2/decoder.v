`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/11/2023 11:25:37 PM
// Design Name: 
// Module Name: decoder
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


module decoder(
        input [11:0] instruction, // 12-bit instruction: 4-bit opCode, 4-bit A, 4-bit B
  
        output [3:0] opCode, // 4-bit operation code for ALU
        output [3:0] A, // 4-bit output A for ALU
        output [3:0] B // 4-bit output B for ALU
    );
    
    // Decode the instruction into opCode, A, and B for the ALU to use
    assign opCode = instruction [11:8];
    assign A = instruction [7:4];
    assign B = instruction [3:0];
endmodule

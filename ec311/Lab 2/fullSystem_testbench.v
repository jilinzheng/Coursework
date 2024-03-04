`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/12/2023 06:11:43 PM
// Design Name: 
// Module Name: fullSystem_testbench
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


module fullSystem_testbench(

    );
    
    reg cIn, sevenSegControl;
    reg [11:0] instruction;
    
    wire cOut;
    wire [7:0] anodeControl;
    wire [6:0] cathodeControl;
    
    fullSystem system(.cIn(cIn),.sevenSegControl(sevenSegControl),.instruction(instruction),.cOut(cOut),.anodeControl(anodeControl),.cathodeControl(cathodeControl));
    
    initial begin
    cIn = 0;
    sevenSegControl = 0; // Show opCode
    instruction = 12'b101000010011; // A (opCode); (full instruction is 1 + 3)
    
    // Addition
    #10 sevenSegControl = 1; // Show aluResult A = 1, B = 3, A + B = 4
    #10 cIn = 1; // A + B + cIn = 5
    
    //Subtraction
    #10 instruction = 12'b101100100100; // 2 - 4 = -2
    
    // Other opCodes
    #10 instruction = 12'b001000010011; // A = 0001, B = 0011, A NAND B = 1000 
    #10 instruction = 12'b000000111001; // Show aluResult A = 3, B = 9, A & B = 0001 (or 1)
    #10 instruction = 12'b010000010011; // A = 0001, B = 0011, A XOR B = 0010
    
    end
endmodule

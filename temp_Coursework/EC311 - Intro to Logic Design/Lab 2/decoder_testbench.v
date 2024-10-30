`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/12/2023 04:27:29 PM
// Design Name: 
// Module Name: decoder_testbench
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


module decoder_testbench(

    );
    
    reg [11:0] instruction;
    
    wire [3:0] opCode, A, B;
    
    decoder decode(.instruction(instruction),.opCode(opCode),.A(A),.B(B));
    
    initial
    begin
    
    instruction = 12'b101001010011; 
    #10 instruction = 12'b000000111001;
    
    end
    
endmodule

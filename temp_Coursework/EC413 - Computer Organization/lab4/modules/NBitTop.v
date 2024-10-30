`timescale 1ns / 1ps

// Parametrized n-bit top module, connecting NBitALU and NBitReg.
// Refer to the Lab 4 assignment diagram for the interpretation of ALUOp and R0-3.

module NBitTop #(n = 8) (
    input clk,
    input [n-1:0] R2, R3,
    input [2:0] ALUOp,
    output [n-1:0] R0
    );
    
    wire [n-1:0] R1;    // to connect ALU and Register
    wire c_out, c_out2;
    
    NBitALU #(.n(n)) ALU (
        .R2(R2),
        .R3(R3),
        .ALUOp(ALUOp),
        .R1(R1),
        .c_out(c_out),
        .c_out2(c_out2)
    );
    
    NBitReg #(.n(n)) Reg (
        .clk(clk),
        .in(R1),
        .out(R0)
    );
    
endmodule

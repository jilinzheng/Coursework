`timescale 1ns / 1ps

// Testbench for parameterized n-bit register.

module NBitReg_tb();
    parameter n = 8;
    reg clk;
    reg [n-1:0] in;
    wire [n-1:0] out;
    
    NBitReg #(.n(n)) Reg (
        .clk(clk),
        .in(in),
        .out(out)
    );

    always #1 clk = ~clk;

    initial begin
        clk = 0;
        in = 0;
    end

    always #2 in = in + 1;
endmodule

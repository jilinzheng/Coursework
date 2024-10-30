`timescale 1ns / 1ps

// Testbench for parameterized n-bit adder.

module NBitAdder_tb();
    parameter n = 4;
    reg [n-1:0] a, b;
    reg c_in;
    wire [n-1:0] sum;
    wire c_out;
    
    NBitAdder #(.n(n)) Adder (
        .a(a),
        .b(b),
        .c_in(c_in),
        .sum(sum),
        .c_out(c_out)
    );

    initial begin
        a = 0;
        b = 0;
        c_in = 0;
    end

    always #2 {a, b, c_in} = {a, b, c_in} + 1;

endmodule

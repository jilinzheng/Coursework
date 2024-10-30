`timescale 1ns / 1ps

// Testbench for parameterized n-bit SLT.

module NBitSLT_tb();
    parameter n = 4;
    reg clk;
    reg [n-1:0] R2, R3;
    wire [n-1:0] R1, verify;
    wire error_flag;
    
    NBitSLT #(.n(n)) SLT (
        .R2(R2),
        .R3(R3),
        .R1(R1)
    );
    
    ALU_behavioral #(.WIDTH(n)) verify_ALU (
        .R2(R2),
        .R3(R3),
        .ALUOp(3'b110),
        .R1(verify)
    );
    
    assign error_flag = (R1 != verify);
    
    always @ (posedge clk) begin
        if (error_flag)
            $display("ERROR @ R2 = %h, R3 = %h; R1 = %h, verify = %h", R2, R3, R1, verify);
    end
    
    always #1 clk = ~clk;

    initial begin
        clk = 0;
        R2 = 0;
        R3 = 0;
    end
    
    always #2 {R2, R3} = {R2, R3} + 1;
    
endmodule

`timescale 1ns / 1ps

// Testbench for parameterized n-bit top module.

module NBitTop_tb();
    parameter n = 4;
    reg clk;
    reg [n-1:0] R2, R3;
    reg [2:0] ALUOp;
    wire [n-1:0] R0, verify;
    wire error_flag;
    
    NBitTop #(.n(n)) Top (
        .clk(clk),
        .R2(R2),
        .R3(R3),
        .ALUOp(ALUOp),
        .R0(R0)
    );
    
    ALU_behavioral #(.WIDTH(n)) verify_ALU (
        .R2(R2),
        .R3(R3),
        .ALUOp(ALUOp),
        .R1(verify)
    );
    
    assign error_flag = (R0 != verify);

    // note that error_flag will be set for 1 time unit for tests (with the current 2 time unit clock cycles)
    // due to register being a sequential element; only on positive edge of clock will register be set;
    // recommended to use a zommed-in waveform to compare R0 values with verify;
    // R0 will be delayed compared to verify
    always @ (posedge clk) begin
        if (error_flag)
            $display("ERROR @ R2 = %h, R3 = %h; R0 = %h, verify = %h", R2, R3, R0, verify);
    end

    always #1 clk = ~clk;

    initial begin
        clk = 0;
        R2 = 0;
        R3 = 0;

        // test all ALUOps
        ALUOp = 3'b000;
        #100 ALUOp = 3'b001;
        #100 ALUOp = 3'b010;
        #100 ALUOp = 3'b011;
        #100 ALUOp = 3'b100;
        #100 ALUOp = 3'b101;
        #100 ALUOp = 3'b110;
        #100;
        $finish;
    end
    
    always #2 {R2, R3} = {R2, R3} + 1;

endmodule

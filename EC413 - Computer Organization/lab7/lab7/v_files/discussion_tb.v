`timescale 1ns / 1ps

module discussion_tb;
    // Inputs
    reg clk;
    reg Reset;
    reg LoadInstructions;
    reg [31:0] Instruction;

    // Outputs
    wire [31:0] out;

    // Instantiate the Unit Under Test (UUT)
    CPU uut (.out(out), 
            .clk(clk), 
            .Reset(Reset), 
            .LoadInstructions(LoadInstructions), 
            .Instruction(Instruction)
            );

    initial begin
        // Initialize Inputs
        clk = 0;
        Reset = 1;
        LoadInstructions = 0;
        Instruction = 0;
        #10;

        Reset = 0;
        LoadInstructions = 1;
        // #10;
        // 0
        Instruction = 32'b001000_00000_00001_0000000110100111;      // addi $R1, $0, 423
        #10; // 1                                                   // -> 423
        Instruction = 32'b001000_00000_00010_0000000001011100;      // addi $R2, $0, 92
        #10; // 2                                                   // -> 92
        Instruction = 32'b001000_00000_00011_0000000000001101;      // addi $R3, $0, 13
        #10; // 3                                                   // -> 13
        Instruction = 32'b001000_00000_00100_0000000010010010;      // addi $R4, $0, 146
        #10; // 4                                                   // -> 146
        Instruction = 32'b001000_00000_00101_0000000000000101;      // addi $R5, $0, 5
        #10; // 5                                                   // -> 5
        Instruction = 32'b000000_00001_00100_00101_00000_100000;    // add $R5, $R1, $R4
        #10; // 6                                                   // -> 569 (423 if wrong, 2-ahead)
        Instruction = 32'b000000_00011_00101_00110_00000_101010;    // slt $R6, $R3, $R5
        #10; // 7                                                   // -> 1 (0 if wrong, 1-ahead)
        Instruction = 32'b100011_00000_00100_0000000000000100;      // lw $R4, 4($0)
        #10; // 8                                                   // -> 4
        Instruction = 32'b000000_00100_00110_00111_00000_100010;    // sub $R7, $R4, $R6
        #10; // 9                                                   // -> 3 (146 or 145 if wrong, 1-ahead & 2-ahead)
        Instruction = 32'b001000_00011_00011_0000000000001101;      // addi $R3, $R3, 13
        #10; // 10                                                  // -> 26
        Instruction = 32'b001000_00100_00100_0000000010010010;      // addi $R4, $R4, 146
        #10; // 11                                                  // -> 150 (292 if wrong, testing register bypass/3-ahead)
        Instruction = 32'b000000_00011_00111_00001_00000_100010;    // sub $R1, $R3, $R7
        #10; // 12                                                  // -> 23 (26 if wrong, testing register bypass/3-ahead)

        LoadInstructions = 0;
        Reset = 1;
        #10;

        Reset = 0;
        #100;
    end

    always begin
        #5;
        clk = ~clk;
    end
endmodule

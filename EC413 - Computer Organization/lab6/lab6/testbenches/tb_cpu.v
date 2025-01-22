`timescale 1ns / 1ns

module tb_cpu;
    // Inputs
    reg rst;
    reg clk;
    reg initialize;
    reg [31:0] instruction_initialize_data;
    reg [31:0] instruction_initialize_address;

    // Instantiate the Unit Under Test (UUT)
    cpu uut (
        .rst(rst), 
        .clk(clk), 
        .initialize(initialize), 
        .instruction_initialize_data(instruction_initialize_data), 
        .instruction_initialize_address(instruction_initialize_address)
    );

    initial begin
        // Initialize Inputs
        rst = 1;
        clk = 0;
        initialize = 1;
        instruction_initialize_data = 0;
        instruction_initialize_address = 0;

        // task 1: provided testcases
        #100 instruction_initialize_address = 0;
        instruction_initialize_data = 32'b000000_00000_00010_00001_00000_10_0000;       // ADD R1, R0, R2
        #10	instruction_initialize_address = 4;
        instruction_initialize_data = 32'b000000_00100_00100_01000_00000_10_0010;       // SUB R8, R4, R4
        #10	instruction_initialize_address = 8;
        instruction_initialize_data = 32'b000000_00101_00110_00111_00000_10_0101;       // OR R7, R5, R6
        #10 instruction_initialize_address = 12;
        instruction_initialize_data = 32'b101011_00000_01001_00000_00000_00_1100;       // SW R9, 12(R0)
        #10 instruction_initialize_address = 16;
        instruction_initialize_data = 32'b100011_00000_01101_00000_00000_00_1100;       // LW R13, 12(R0)
        #10	instruction_initialize_address = 20;
        //instruction_initialize_data = 32'b000100_00000_00000_11111_11111_11_1111;       // BEQ R0, R0, -1
        instruction_initialize_data = 32'b000100_00000_00000_00000_00000_00_0000;       // BEQ R0, R0, 0 (just move to next instruction, but with an explicit branch)

        // task 2: slt testcases
        #10 instruction_initialize_address = 24;
        instruction_initialize_data = 32'b000000_00000_00001_00010_00000_10_1010;       // SLT R2, R0, R1
        #10 instruction_initialize_address = 28;
        instruction_initialize_data = 32'b000000_00100_00011_00010_00000_10_1010;       // SLT R2, R4, R3
        #10 instruction_initialize_address = 32;
        instruction_initialize_data = 32'b000000_00101_00110_00010_00000_10_1010;       // SLT R2, R5, R6
        #10 instruction_initialize_address = 36;
        instruction_initialize_data = 32'b000000_01000_00111_00010_00000_10_1010;       // SLT R2, R8, R7
        #10 instruction_initialize_address = 40;
        instruction_initialize_data = 32'b000000_01010_01001_00010_00000_10_1010;       // SLT R2, R10, R9

        // task 3: addi testcases
        #10 instruction_initialize_address = 44;
        instruction_initialize_data = 32'b001000_00010_00011_0000_0000_0000_0101;       // ADDI R3, R2, 5
        #10 instruction_initialize_address = 48;
        instruction_initialize_data = 32'b001000_00100_00011_1111_1111_1111_1000;       // ADDI R3, R4, -8
        #10 instruction_initialize_address = 52;
        instruction_initialize_data = 32'b001000_00110_00011_0111_1111_1111_1111;       // ADDI R3, R6, 32767
        #10 instruction_initialize_address = 56;
        instruction_initialize_data = 32'b001000_01000_00011_1000_0000_0000_0000;       // ADDI R3, R8, -32768
        #10 instruction_initialize_address = 60;
        instruction_initialize_data = 32'b001000_01010_00011_0000_0000_0000_0000;       // ADDI R3, R10, 0
        #10 instruction_initialize_address = 64;
        instruction_initialize_data = 32'b001000_00010_00011_0100_0000_0000_0000;       // ADDI R3, R2, 16384
        #10 instruction_initialize_address = 68;
        instruction_initialize_data = 32'b001000_00100_00011_1111_1100_0000_0000;       // ADDI R3, R4, -1024
        #10 instruction_initialize_address = 72;
        instruction_initialize_data = 32'b001000_00110_00011_0000_0000_1111_1111;       // ADDI R3, R6, 255
        #10 instruction_initialize_address = 76;
        instruction_initialize_data = 32'b001000_00110_00011_0000_0000_0100_0000;       // ADDI R3, R6, 64
        #10 instruction_initialize_address = 80;
        instruction_initialize_data = 32'b001000_00100_00011_0000_0000_0000_1010;       // ADDI R3, R4, 10

        // task 4: jump testcases
        #10 instruction_initialize_address = 84;
        instruction_initialize_data = 32'b000010_0000_0000_0000_0000_0000_0000_00;      // J 0 (back to beginning of program)
        //instruction_initialize_data = 32'b000010_0000_0000_0000_0000_0000_0101_10;      // J 22 (explicitly move on to next instruction (22*4 = 88))

        // task 5: bne testcases
        #10 instruction_initialize_address = 88;
        //instruction_initialize_data = 32'b000101_00000_11111_1111_1111_1111_0100;       // BNE R0, R31, -12 (move to instruction 44 if R0 != R31, 92-(12*4) = 44;
                                                                                        // 92 from PC_plus_4, -12 in 2's complement
        instruction_initialize_data = 32'b000101_00000_11111_0000_0000_0000_0000;       // BNE R0, R31, 1 (explicitly branch to next instruction; 92+0 = 92 from PC_plus_4)

        // task 6: lui testcases
        #10 instruction_initialize_address = 92;
        instruction_initialize_data = 32'h3c_00_ff_ff;                                  // LUI R0, 0xffff
        #10 instruction_initialize_address = 96;
        instruction_initialize_data = 32'h3c_00_ab_cd;                                  // LUI R0, 0xabcd
        #10 instruction_initialize_address = 100;
        instruction_initialize_data = 32'h3c_00_de_ad;                                  // LUI R0, 0xdead
        #10 instruction_initialize_address = 104;
        instruction_initialize_data = 32'h3c_00_be_ef;                                  // LUI R0, 0xbeef
        #10 instruction_initialize_address = 108;
        instruction_initialize_data = 32'h3c_00_c0_01;                                  // LUI R0, 0xc001

        // start program execution
        #10 initialize = 0; rst = 0;

        #300 $finish;
    end

    always #5 clk = ~clk;
endmodule
`timescale 1ns / 1ns

module tb_cpu_ec;
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

        // addi
        #100 instruction_initialize_address = 0;
        instruction_initialize_data = 32'b001000_00010_00011_0000_0000_0000_0101;       // ADDI R3, R2, 5
        #10 instruction_initialize_address = 4;
        instruction_initialize_data = 32'b001000_00001_00011_1111_1111_1111_0110;       // ADDI R3, R1, -10
        #10 instruction_initialize_address = 8;
        instruction_initialize_data = 32'b001000_00001_00011_1000_0000_0000_0000;       // ADDI R3, R1, -32768
        #10 instruction_initialize_address = 12;
        instruction_initialize_data = 32'b001000_00010_00011_0100_0000_0000_0000;       // ADDI R3, R2, 16384
        #10 instruction_initialize_address = 16;
        instruction_initialize_data = 32'b001000_00000_00011_0000_0000_0000_0000;       // ADDI R3, R0, 0

        // addiu
        #10 instruction_initialize_address = 20;
        instruction_initialize_data = 32'b001001_00010_00011_0000_0000_0000_0101;       // ADDIU R3, R2, 5
        #10 instruction_initialize_address = 24;
        instruction_initialize_data = 32'b001001_00001_00011_1111_1111_1111_0110;       // ADDIU R3, R1, -10
        #10 instruction_initialize_address = 28;
        instruction_initialize_data = 32'b001001_00001_00011_1000_0000_0000_0000;       // ADDIU R3, R1, -32768
        #10 instruction_initialize_address = 32;
        instruction_initialize_data = 32'b001001_00010_00011_0100_0000_0000_0000;       // ADDIU R3, R2, 16384
        #10 instruction_initialize_address = 36;
        instruction_initialize_data = 32'b001001_00000_00011_0000_0000_0000_0000;       // ADDIU R3, R0, 0
        
        // slti
        #10 instruction_initialize_address = 40;
        instruction_initialize_data = 32'b001010_00000_00001_1111_1111_1111_1111;       // SLTI R1, R0, -1
        #10 instruction_initialize_address = 44;
        instruction_initialize_data = 32'b001010_00000_00001_1111_1111_1111_1100;       // SLTI R1, R0, -4
        #10 instruction_initialize_address = 48;
        instruction_initialize_data = 32'b001010_00000_00001_1111_1111_1111_1000;       // SLTI R1, R0, -8
        #10 instruction_initialize_address = 52;
        instruction_initialize_data = 32'b001010_00000_00001_1111_1111_1111_0111;       // SLTI R1, R0, -9
        #10 instruction_initialize_address = 56;
        instruction_initialize_data = 32'b001010_00000_00001_1111_1111_1110_1010;       // SLTI R1, R0, -22

        // sltiu
        #10 instruction_initialize_address = 60;
        instruction_initialize_data = 32'b001011_00000_00001_1111_1111_1111_1111;       // SLTIU R1, R0, -1
        #10 instruction_initialize_address = 64;
        instruction_initialize_data = 32'b001011_00000_00001_1111_1111_1111_1100;       // SLTIU R1, R0, -4
        #10 instruction_initialize_address = 68;
        instruction_initialize_data = 32'b001011_00000_00001_1111_1111_1111_1000;       // SLTIU R1, R0, -8
        #10 instruction_initialize_address = 72;
        instruction_initialize_data = 32'b001011_00000_00001_1111_1111_1111_0111;       // SLTIU R1, R0, -9
        #10 instruction_initialize_address = 76;
        instruction_initialize_data = 32'b001011_00000_00001_1111_1111_1110_1010;       // SLTIU R1, R0, -22

        /*
        // andi
        #10 instruction_initialize_address = 84;
        instruction_initialize_data = 32'b001100_00001_00010_1111_1111_0000_0000;       // ANDI R2, R1, 0xFF00
        #10 instruction_initialize_address = 88;
        instruction_initialize_data = 32'b001100_00001_00010_0000_1111_1111_1111;       // ANDI R2, R1, 0x0FFF
        #10 instruction_initialize_address = 92;
        instruction_initialize_data = 32'b001100_00001_00010_1010_1010_1010_1010;       // ANDI R2, R1, 0xAAAA
        #10 instruction_initialize_address = 96;
        instruction_initialize_data = 32'b001100_00001_00010_1100_1100_1100_1100;       // ANDI R2, R1, 0xCCCC
        #10 instruction_initialize_address = 100;
        instruction_initialize_data = 32'b001100_00001_00010_0101_0101_0101_0101;       // ANDI R2, R1, 0x5555

        // ori
        #10 instruction_initialize_address = 104;
        instruction_initialize_data = 32'b001101_00001_00010_1111_1111_0000_0000;       // ORI R2, R1, 0xFF00
        #10 instruction_initialize_address = 108;
        instruction_initialize_data = 32'b001101_00001_00010_0000_1111_1111_1111;       // ORI R2, R1, 0x0FFF
        #10 instruction_initialize_address = 112;
        instruction_initialize_data = 32'b001101_00001_00010_1010_1010_1010_1010;       // ORI R2, R1, 0xAAAA
        #10 instruction_initialize_address = 116;
        instruction_initialize_data = 32'b001101_00001_00010_1100_1100_1100_1100;       // ORI R2, R1, 0xCCCC
        #10 instruction_initialize_address = 120;
        instruction_initialize_data = 32'b001101_00001_00010_0101_0101_0101_0101;       // ORI R2, R1, 0x5555

        // xori
        #10 instruction_initialize_address = 124;
        instruction_initialize_data = 32'b001110_00001_00010_1111_1111_0000_0000;       // XORI R2, R1, 0xFF00
        #10 instruction_initialize_address = 128;
        instruction_initialize_data = 32'b001110_00001_00010_0000_1111_1111_1111;       // XORI R2, R1, 0x0FFF
        #10 instruction_initialize_address = 132;
        instruction_initialize_data = 32'b001110_00001_00010_1010_1010_1010_1010;       // XORI R2, R1, 0xAAAA
        #10 instruction_initialize_address = 136;
        instruction_initialize_data = 32'b001110_00001_00010_1100_1100_1100_1100;       // XORI R2, R1, 0xCCCC
        #10 instruction_initialize_address = 140;
        instruction_initialize_data = 32'b001110_00001_00010_0101_0101_0101_0101;       // XORI R2, R1, 0x5555

        // lui
        #10 instruction_initialize_address = 144;
        instruction_initialize_data = 32'h3c_00_ff_ff;                                  // LUI R0, 0xffff
        #10 instruction_initialize_address = 148;
        instruction_initialize_data = 32'h3c_00_ab_cd;                                  // LUI R0, 0xabcd
        #10 instruction_initialize_address = 152;
        instruction_initialize_data = 32'h3c_00_de_ad;                                  // LUI R0, 0xdead
        #10 instruction_initialize_address = 156;
        instruction_initialize_data = 32'h3c_00_be_ef;                                  // LUI R0, 0xbeef
        #10 instruction_initialize_address = 160;
        instruction_initialize_data = 32'h3c_00_c0_01;                                  // LUI R0, 0xc001
        */

        // start program execution
        #10 initialize = 0; rst = 0;

        #200 $finish;
    end

    always #5 clk = ~clk;
endmodule
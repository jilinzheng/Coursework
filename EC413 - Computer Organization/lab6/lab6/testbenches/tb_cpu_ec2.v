`timescale 1ns / 1ns

module tb_cpu_ec2;
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

        // andi
        #10 instruction_initialize_address = 0;
        instruction_initialize_data = 32'b001100_00001_00010_1111_1111_0000_0000;       // ANDI R2, R1, 0xFF00
        #10 instruction_initialize_address = 4;
        instruction_initialize_data = 32'b001100_00001_00010_0000_1111_1111_1111;       // ANDI R2, R1, 0x0FFF
        #10 instruction_initialize_address = 8;
        instruction_initialize_data = 32'b001100_00001_00010_1010_1010_1010_1010;       // ANDI R2, R1, 0xAAAA
        #10 instruction_initialize_address = 12;
        instruction_initialize_data = 32'b001100_00001_00010_1100_1100_1100_1100;       // ANDI R2, R1, 0xCCCC
        #10 instruction_initialize_address = 16;
        instruction_initialize_data = 32'b001100_00001_00010_0101_0101_0101_0101;       // ANDI R2, R1, 0x5555

        // ori
        #10 instruction_initialize_address = 20;
        instruction_initialize_data = 32'b001101_00001_00010_1111_1111_0000_0000;       // ORI R2, R1, 0xFF00
        #10 instruction_initialize_address = 24;
        instruction_initialize_data = 32'b001101_00001_00010_0000_1111_1111_1111;       // ORI R2, R1, 0x0FFF
        #10 instruction_initialize_address = 28;
        instruction_initialize_data = 32'b001101_00001_00010_1010_1010_1010_1010;       // ORI R2, R1, 0xAAAA
        #10 instruction_initialize_address = 32;
        instruction_initialize_data = 32'b001101_00001_00010_1100_1100_1100_1100;       // ORI R2, R1, 0xCCCC
        #10 instruction_initialize_address = 36;
        instruction_initialize_data = 32'b001101_00001_00010_0101_0101_0101_0101;       // ORI R2, R1, 0x5555

        // xori
        #10 instruction_initialize_address = 40;
        instruction_initialize_data = 32'b001110_00001_00010_1111_1111_0000_0000;       // XORI R2, R1, 0xFF00
        #10 instruction_initialize_address = 44;
        instruction_initialize_data = 32'b001110_00001_00010_0000_1111_1111_1111;       // XORI R2, R1, 0x0FFF
        #10 instruction_initialize_address = 48;
        instruction_initialize_data = 32'b001110_00001_00010_1010_1010_1010_1010;       // XORI R2, R1, 0xAAAA
        #10 instruction_initialize_address = 52;
        instruction_initialize_data = 32'b001110_00001_00010_1100_1100_1100_1100;       // XORI R2, R1, 0xCCCC
        #10 instruction_initialize_address = 56;
        instruction_initialize_data = 32'b001110_00001_00010_0101_0101_0101_0101;       // XORI R2, R1, 0x5555

        // lui
        #10 instruction_initialize_address = 60;
        instruction_initialize_data = 32'h3c_00_ff_ff;                                  // LUI R0, 0xffff
        #10 instruction_initialize_address = 64;
        instruction_initialize_data = 32'h3c_00_ab_cd;                                  // LUI R0, 0xabcd
        #10 instruction_initialize_address = 68;
        instruction_initialize_data = 32'h3c_00_de_ad;                                  // LUI R0, 0xdead
        #10 instruction_initialize_address = 72;
        instruction_initialize_data = 32'h3c_00_be_ef;                                  // LUI R0, 0xbeef
        #10 instruction_initialize_address = 76;
        instruction_initialize_data = 32'h3c_00_c0_01;                                  // LUI R0, 0xc001

        // start program execution
        #10 initialize = 0; rst = 0;

        #200 $finish;
    end

    always #5 clk = ~clk;
endmodule
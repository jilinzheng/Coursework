`timescale 1ns / 1ps

// Testbench for Pipeline module, modified from sample testbench provided with assignment.

module Pipeline_tb;
    // inputs
    reg [31:0] InstrIn;
    reg Reset;
    reg Clk;

    // outputs
    wire [31:0] ALUOut;

    // instantiate the Unit Under Test (UUT)
    Pipeline uut (
        .InstrIn(InstrIn),
        .Reset(Reset),
        .Clk(Clk),
        .ALUOut(ALUOut)
    );

    always #5 Clk = ~Clk;

    initial begin
        // initialize Inputs
        InstrIn = 0;
        Reset = 1;
        Clk = 1;

        // wait 100 ns for global reset to finish
        #100 Reset = 0;

        // reminder that each instruction takes three cycles to complete (30 time units)

        // assignment-given instructions
        // note that these instructions do run into data hazards with reading stale data
        // since data forwarding has not been covered, I have continued to use these cases
        // knowingly, adjusting my paper test case accordingly
        #10 InstrIn = 32'b011010_00001_00001_0000000000001010;  // I, ADD R1, R1, 10    => R1 = 10
        #10 InstrIn = 32'b011100_00010_00010_0000000000000010;  // I, OR R2, R2, 2      => R2 = 2
        #20; // STALL to avoid data hazard/reading stale data
        #10 InstrIn = 32'b010010_00011_00001_00010_00000000000; // R, ADD R3, R1, R2    => R3 = 0 (STALE DATA!)
        #10 InstrIn = 32'b010011_00100_00001_00010_00000000000; // R, SUB R4, R1, R2    => R4 = 10 (STALE DATA!)
        
        // reset; note that I have also made the register file reset with this input
        #30 Reset = 1; #30 Reset = 0;
        
        // instructions modified from discussion slides
        InstrIn = 32'b010101_00110_00000_00001_00000000000;     // R, AND R6, R0, R1    => R6 = 0
        #10 InstrIn = 32'b011010_00001_00010_0000000000001111;  // I, ADDI R1, R2, 15   => R1 = 15
        #10 InstrIn = 32'b010110_00011_00100_00101_00000000000; // R, SLT R3, R4, R5    => R3 = 0
        #10 InstrIn = 32'b010100_00000_00110_00000_00000000000; // R, OR R0, R6, R0     => R0 = 0
        #10 InstrIn = 32'b010011_00010_00001_00010_00000000000; // R, SUB R2, R1, R2    => R2 = 15

        // reset
        #30 Reset = 1; #30 Reset = 0;

        // I-type instructions provided with sample testbench
        InstrIn = 32'b011010_00000_00000_0000000000000101;       // I, add r0 with 00000005  => r0 = 00000005
        #10 InstrIn = 32'b011010_00001_00001_0000000000001010;   // I, add r1 with 0000000A  => r1 = 0000000A
        #10 InstrIn = 32'b011010_00010_00010_1111111111111000;   // I, add r2 with 0000FFF8  => r2 = 0000FFF8
        #10 InstrIn = 32'b011001_00011_00011_1111111111111000;   // I, not r3                => r3 = FFFFFFFF
        #10 InstrIn = 32'b011100_00100_00100_1010101010101010;   // I, or r4 with 0000AAAA   => r4 = 00000AAAA
        #10 InstrIn = 32'b011101_00101_00101_1111111111111111;   // I, and r5 with 0000FFFF  => r5 = 00000000
        #10 InstrIn = 32'b011110_00110_00110_1111111111111000;   // I, slt r6 with 0000FFFF8 => r6 = 00000001

        // R-type instructions provided with sample testbench
        #10 InstrIn = 32'b010001_00111_00001_00000_00000000000;  // R, not r1(0000000A)                   => r7 = FFFFFFF5
        #10 InstrIn = 32'b010010_01000_00001_00010_00000000000;  // R, add r1(0000000A) with r2(0000FFF8) => r8 = 00010002
        #10 InstrIn = 32'b010010_01001_00001_00011_00000000000;  // R, add r1(0000000A) with r3(FFFFFFFF) => r9 = 00000009
        #10 InstrIn = 32'b010010_01010_00001_00100_00000000000;  // R, add r1(0000000A) with r4(0000AAAA) => r10 = 0000AAB4
        #10 InstrIn = 32'b010010_01011_00001_00101_00000000000;  // R, add r1(0000000A) with r5(00000000) => r11 = 0000000A
        #10 InstrIn = 32'b010010_01100_00001_00110_00000000000;  // R, add r1(0000000A) wtih r6(00000001) => r12 = 0000000B
        
        #50 $finish;
    end
endmodule


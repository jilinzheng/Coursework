`timescale 1ns / 1ps

// 3-stage pipeline: refer to lab 5 assignment for a diagram of how things are connected.

module Pipeline(
    input Clk,
    input Reset,
    input [31:0] InstrIn,

    output [31:0] ALUOut
    );

    // wires for stage 1
    wire [4:0] S1_ReadSelect1, S1_ReadSelect2, S1_WriteSelect;
    wire [15:0] S1_Imm;
    wire S1_DataSrc, S1_WriteEnable;
    wire [2:0] S1_ALUOp;

    // wires for regfile
    wire [31:0] ReadData1, ReadData2;

    // wires for stage 2
    wire [31:0] S2_ReadData1, S2_ReadData2;
    wire [15:0] S2_Imm;
    wire S2_DataSrc, S2_WriteEnable;
    wire [2:0] S2_ALUOp;
    wire [4:0] S2_WriteSelect;

    // wires for ALU and MUX; ImmMuxIn used to pad 16-bit immediate
    wire [31:0] R3, R1, ImmMuxIn;

    // wires for stage 3
    wire [4:0] S3_WriteSelect;
    wire S3_WriteEnable;

    S1Register S1 (
        .Clk(Clk),
        .Reset(Reset),
        .InstrIn(InstrIn),

        .S1_ReadSelect1(S1_ReadSelect1),
        .S1_ReadSelect2(S1_ReadSelect2),
        .S1_Imm(S1_Imm),
        .S1_DataSrc(S1_DataSrc),
        .S1_ALUOp(S1_ALUOp),
        .S1_WriteSelect(S1_WriteSelect),
        .S1_WriteEnable(S1_WriteEnable)
    );

    NBitRegisterFile RegFile (
        .ReadSelect1(S1_ReadSelect1),
        .ReadSelect2(S1_ReadSelect2),
        .WriteSelect(S3_WriteSelect),
        .WriteData(ALUOut),
        .WriteEnable(S2_WriteEnable),
        .Clk(Clk),
        .Reset(Reset),
        .ReadData1(ReadData1),
        .ReadData2(ReadData2)
    );

    S2Register S2 (
        .Clk(Clk),
        .Reset(Reset),
        .Reg_ReadData1(ReadData1),
        .Reg_ReadData2(ReadData2),
        .S1_Imm(S1_Imm),
        .S1_DataSrc(S1_DataSrc),
        .S1_ALUOp(S1_ALUOp),
        .S1_WriteSelect(S1_WriteSelect),
        .S1_WriteEnable(S1_WriteEnable),

        .S2_ReadData1(S2_ReadData1),
        .S2_ReadData2(S2_ReadData2),
        .S2_Imm(S2_Imm),
        .S2_DataSrc(S2_DataSrc),
        .S2_ALUOp(S2_ALUOp),
        .S2_WriteSelect(S2_WriteSelect),
        .S2_WriteEnable(S2_WriteEnable)
    );

    // pad 16-bit immediate with 0's, to 32 bits
    assign ImmMuxIn = {16'h0000, S2_Imm};

    NBit2x1Mux Mux (
        .MuxIn1(S2_ReadData2),
        .MuxIn2(ImmMuxIn),
        .MuxSel(S2_DataSrc),
        .MuxOut(R3)
    );

    NBitALU ALU (
        .R2(S2_ReadData1),
        .R3(R3),
        .ALUOp(S2_ALUOp),
        .R1(R1)
    );

    S3Register S3 (
        .Clk(Clk),
        .Reset(Reset),
        .R1(R1),
        .S2_WriteSelect(S2_WriteSelect),
        .S2_WriteEnable(S2_WriteEnable),

        .ALUOut(ALUOut),
        .S3_WriteSelect(S3_WriteSelect),
        .S3_WriteEnable(S3_WriteEnable)
    );
endmodule

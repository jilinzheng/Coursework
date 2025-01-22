`timescale 1ns / 1ps

// Stage 1 register (module) of datapath.

module S1Register(
    input Clk,                          // clock for all sequential logic
    input Reset,                        // reset all sequential logic to zero
    input [31:0] InstrIn,               // instruction input

    output reg [4:0] S1_ReadSelect1,    // register select for first read register
    output reg [4:0] S1_ReadSelect2,    // register select for second read register
    output reg [15:0] S1_Imm,           // immediate data
    output reg S1_DataSrc,              // data source selection for second operand
    output reg [2:0] S1_ALUOp,          // ALU operation
    output reg [4:0] S1_WriteSelect,    // register select for input register
    output reg S1_WriteEnable           // register file write enable
   );

    always @ (posedge Clk) begin
        if (Reset) begin
            S1_ReadSelect1 <= 5'd0;
            S1_ReadSelect2 <= 5'd0;
            S1_Imm         <= 16'd0;
            S1_DataSrc     <= 1'b0;
            S1_ALUOp       <= 3'b000;
            S1_WriteSelect <= 5'd0;
            S1_WriteEnable <= 1'b0;
        end else begin
            S1_ReadSelect1 <= InstrIn[20:16];
            S1_ReadSelect2 <= InstrIn[15:11];
            S1_Imm         <= InstrIn[15:0];
            S1_DataSrc     <= InstrIn[29];
            S1_ALUOp       <= InstrIn[28:26];
            S1_WriteSelect <= InstrIn[25:21];
            S1_WriteEnable <= 1'b1;
        end
    end
endmodule

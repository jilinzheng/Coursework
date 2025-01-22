`timescale 1ns / 1ps

// Stage 2 register (module) of datapath.

module S2Register(
    input Clk,                          // clock for all sequential logic
    input Reset,                        // reset all sequential logic to zero
    input [31:0] Reg_ReadData1,         // register output 1 from register file
    input [31:0] Reg_ReadData2,         // register output 2 from register file
    input [15:0] S1_Imm,                // immediate data from stage 1
    input S1_DataSrc,                   // data source selection for second operand from stage 1
    input [2:0] S1_ALUOp,               // ALU operation from stage 1
    input [4:0] S1_WriteSelect,         // register select for input register from stage 1
    input S1_WriteEnable,               // register file write enable from stage 1

    output reg [31:0] S2_ReadData1,     // register output 1
    output reg [31:0] S2_ReadData2,     // register output 2
    output reg [15:0] S2_Imm,           // immediate data
    output reg S2_DataSrc,              // data source selection for second operand
    output reg [2:0] S2_ALUOp,          // ALU operation
    output reg [4:0] S2_WriteSelect,    // register select for input register
    output reg S2_WriteEnable           // register file write enable
    );

    always @ (posedge Clk) begin
        if (Reset) begin
            S2_ReadData1   <= 32'd0;
            S2_ReadData2   <= 32'd0;
            S2_Imm         <= 16'd0;
            S2_DataSrc     <= 1'b0;
            S2_ALUOp       <= 3'b000;
            S2_WriteSelect <= 5'd0;
            S2_WriteEnable <= 1'b0;
        end else begin
            S2_ReadData1   <= Reg_ReadData1;
            S2_ReadData2   <= Reg_ReadData2;
            S2_Imm         <= S1_Imm;
            S2_DataSrc     <= S1_DataSrc;
            S2_ALUOp       <= S1_ALUOp;
            S2_WriteSelect <= S1_WriteSelect;
            S2_WriteEnable <= S1_WriteEnable;
        end
    end
endmodule

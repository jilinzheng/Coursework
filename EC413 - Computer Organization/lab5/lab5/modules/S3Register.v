`timescale 1ns / 1ps

// Stage 3 register (module) of datapath.

module S3Register(
    input Clk,                          // clock for all sequential logic
    input Reset,                        // reset all sequential logic to zero
    input [31:0] R1,                    // ALU output
    input [4:0] S2_WriteSelect,         // register select for input register from stage 2
    input S2_WriteEnable,               // register file write enable from stage 2

    output reg [31:0] ALUOut,           // ALU output
    output reg [4:0] S3_WriteSelect,    // register select for input register
    output reg S3_WriteEnable           // register file write enable
    );

    always @ (posedge Clk) begin
        if (Reset) begin
            ALUOut         <= 32'd0;
            S3_WriteSelect <= 5'd0;
            S3_WriteEnable <= 1'b0;
        end else begin
            ALUOut         <= R1;
            S3_WriteSelect <= S2_WriteSelect;
            S3_WriteEnable <= S2_WriteEnable;
        end
    end
endmodule

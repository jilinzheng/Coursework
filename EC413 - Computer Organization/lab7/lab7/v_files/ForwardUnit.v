`timescale 1ns / 1ps

// Forwarding Unit module.

module ForwardUnit(
    input EXMEM_RegWrite, MEMWB_RegWrite,
    input [4:0] EXMEM_RegDst, MEMWB_RegDst, IDEX_Rs, IDEX_Rt,
    output reg [1:0] ForwardA, ForwardB
    );

    // set ForwardA select signal
    always @ (EXMEM_RegWrite, MEMWB_RegWrite, EXMEM_RegDst, MEMWB_RegDst, IDEX_Rs) begin
        // check later instruction
        if (EXMEM_RegWrite                      // logic for no write
            && EXMEM_RegDst != 0                // logic for $0 write
            && EXMEM_RegDst == IDEX_Rs) begin   // logic for 1-ahead
            ForwardA = 2'b10;
        // check earlier instruction
        end else if (MEMWB_RegWrite             // logic for no write
            && MEMWB_RegDst != 0                // logic for $0 write
            && EXMEM_RegDst != IDEX_Rs          // logic for arbitration
            && MEMWB_RegDst == IDEX_Rs) begin   // logic for 2-ahead
            ForwardA = 2'b01;
        // default
        end else begin
            ForwardA = 2'b00;
        end
    end

    // set ForwardB select signal
    always @ (EXMEM_RegWrite, MEMWB_RegWrite, EXMEM_RegDst, MEMWB_RegDst, IDEX_Rt) begin
        // check later instruction
        if (EXMEM_RegWrite                      // logic for no write
            && EXMEM_RegDst != 0                // logic for $0 write
            && EXMEM_RegDst == IDEX_Rt) begin   // logic for 1-ahead
            ForwardB = 2'b10;
        // check earlier instruction
        end else if (MEMWB_RegWrite             // logic for no write
            && MEMWB_RegDst != 0                // logic for $0 write
            && EXMEM_RegDst != IDEX_Rt          // logic for arbitration
            && MEMWB_RegDst == IDEX_Rt) begin   // logic for 2-ahead
            ForwardB = 2'b01;
        // default
        end else begin
            ForwardB = 2'b00;
        end
    end
endmodule

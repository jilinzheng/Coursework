`timescale 1ns / 1ns

module control(
    input [5:0] instruction,
    output reg [3:0] ALUOp,
    output reg MemRead,
    output reg MemtoReg,
    output reg RegDst,
    output reg Branch,
    output reg BranchNE,
    output reg ALUSrc,
    output reg MemWrite,
    output reg RegWrite,
    output reg Jump,
    output reg LUI,
    output reg ZeroExtend
    );

    always @(*) begin
        if (instruction == 6'b00_0000) begin            // R-type
            ALUOp = 4'b0000;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            RegDst = 1'b1;
            Branch = 1'b0;
            BranchNE = 1'b0;
            ALUSrc = 1'b0;
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            Jump = 1'b0;
            LUI = 1'b0;
            ZeroExtend = 1'b0;
         end else if (instruction == 6'b00_0100) begin   // branch (beq)
            ALUOp = 4'b0001;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            RegDst = 1'b0;
            Branch = 1'b1;
            BranchNE = 1'b0;
            ALUSrc = 1'b0;
            MemWrite = 1'b0;
            RegWrite = 1'b0;
            Jump = 1'b0;
            LUI = 1'b0;
        end else if (instruction == 6'b00_0101) begin   // bne, opcode from MIPS green sheet
            ALUOp = 4'b0001;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            RegDst = 1'b0;
            Branch = 1'b0;
            BranchNE = 1'b1;
            ALUSrc = 1'b0;
            MemWrite = 1'b0;
            RegWrite = 1'b0;
            Jump = 1'b0;
            LUI = 1'b0;
            ZeroExtend = 1'b0;
        end else if (instruction == 6'b10_1011) begin   // sw
            ALUOp = 4'b0010;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            RegDst = 1'b0;
            Branch = 1'b0;
            BranchNE = 1'b0;
            ALUSrc = 1'b1;
            MemWrite = 1'b1;
            RegWrite = 1'b0;
            Jump = 1'b0;
            LUI = 1'b0;
            ZeroExtend = 1'b0;
        end else if (instruction == 6'b10_0011) begin   // lw
            ALUOp = 4'b0010;
            MemRead = 1'b1;
            MemtoReg = 1'b1;
            RegDst = 1'b0;
            Branch = 1'b0;
            BranchNE = 1'b0;
            ALUSrc = 1'b1;
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            Jump = 1'b0;
            LUI = 1'b0;
            ZeroExtend = 1'b0;
        end else if (instruction == 6'b00_1000) begin   // addi, opcode from MIPS green sheet
            ALUOp = 4'b0010;                            // the add-always case as used in sw/lw
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            RegDst = 1'b0;                              // select instr bits [20:16]
            Branch = 1'b0;
            BranchNE = 1'b0;
            ALUSrc = 1'b1;                              // select immediate
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            Jump = 1'b0;
            LUI = 1'b0;
            ZeroExtend = 1'b0;
        end else if (instruction == 6'b00_0010) begin   // jump, opcode from MIPS green sheet
            ALUOp = 4'b0010;                            // don't care;
            MemRead = 1'b0;                             // in fact, don't care about any control
            MemtoReg = 1'b0;                            // other than jump
            RegDst = 1'b0;
            Branch = 1'b0;
            BranchNE = 1'b0;
            ALUSrc = 1'b0;
            MemWrite = 1'b0;
            RegWrite = 1'b0;
            Jump = 1'b1;
            LUI = 1'b0;
            ZeroExtend = 1'b0;
        end else if (instruction == 6'b00_1111) begin   // lui, opcode from MIPS green sheet
            ALUOp = 4'b0011;                            // pass-thru
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            RegDst = 1'b0;                              // use instr [20:16]
            Branch = 1'b0;
            BranchNE = 1'b0;
            ALUSrc = 1'b1;                              // use immediate
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            Jump = 1'b0;
            LUI = 1'b1;
            ZeroExtend = 1'b0;

        // EXTRA CREDIT INSTRUCTIONS
        end else if (instruction == 6'b00_1001) begin   // addiu, opcode from MIPS green sheet
            ALUOp = 4'b0100;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            RegDst = 1'b0;                              // use instr [20:16]
            Branch = 1'b0;
            BranchNE = 1'b0;
            ALUSrc = 1'b1;                              // use immediate
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            Jump = 1'b0;
            LUI = 1'b0;
            ZeroExtend = 1'b0;
        end else if (instruction == 6'b00_1010) begin   // slti, opcode from MIPS green sheet
            ALUOp = 4'b0101;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            RegDst = 1'b0;                              // use instr [20:16]
            Branch = 1'b0;
            BranchNE = 1'b0;
            ALUSrc = 1'b1;                              // use immediate
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            Jump = 1'b0;
            LUI = 1'b0;
            ZeroExtend = 1'b0;
        end else if (instruction == 6'b00_1011) begin   // sltiu, opcode from MIPS green sheet
            ALUOp = 4'b0110;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            RegDst = 1'b0;                              // use instr [20:16]
            Branch = 1'b0;
            BranchNE = 1'b0;
            ALUSrc = 1'b1;                              // use immediate
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            Jump = 1'b0;
            LUI = 1'b0;
            ZeroExtend = 1'b0;
        end else if (instruction == 6'b00_1100) begin   // andi, opcode from MIPS green sheet
            ALUOp = 4'b0111;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            RegDst = 1'b0;                              // use instr [20:16]
            Branch = 1'b0;
            BranchNE = 1'b0;
            ALUSrc = 1'b1;                              // use immediate
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            Jump = 1'b0;
            LUI = 1'b0;
            ZeroExtend = 1'b1;                          // zero-extend instead of sign-extend
        end else if (instruction == 6'b00_1101) begin   // ori, opcode from MIPS green sheet
            ALUOp = 4'b1000;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            RegDst = 1'b0;                              // use instr [20:16]
            Branch = 1'b0;
            BranchNE = 1'b0;
            ALUSrc = 1'b1;                              // use immediate
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            Jump = 1'b0;
            LUI = 1'b0;
            ZeroExtend = 1'b1;                          // zero-extend instead of sign-extend
        end else if (instruction == 6'b00_1110) begin   // xori
            ALUOp = 4'b1001;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            RegDst = 1'b0;                              // use instr [20:16]
            Branch = 1'b0;
            BranchNE = 1'b0;
            ALUSrc = 1'b1;                              // use immediate
            MemWrite = 1'b0;
            RegWrite = 1'b1;
            Jump = 1'b0;
            LUI = 1'b0;
            ZeroExtend = 1'b1;                          // zero-extend instead of sign-extend
        end else begin
            ALUOp = 4'b0000;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            RegDst = 1'b0;
            Branch = 1'b0;
            BranchNE = 1'b0;
            ALUSrc = 1'b0;
            MemWrite = 1'b0;
            RegWrite = 1'b0;
            Jump = 1'b0;
            LUI = 1'b0;
            ZeroExtend = 1'b0;
        end
    end
endmodule
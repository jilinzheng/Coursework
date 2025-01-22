`timescale 1ns / 1ns

module ALU_control(
    input [5:0] instruction,
    input [3:0] ALUOp,
    output reg [3:0] func
    );

    always @ (*) begin
        if (ALUOp == 4'b0000) begin             // ALUOp
            if (instruction == 6'h20)           // add
                func = 4'd0;
            else if (instruction == 6'h22)      // sub
                func = 4'd1;
            else if (instruction == 6'h24)      // and
                func = 4'd2;
            else if (instruction == 6'h25)      // or
                func = 4'd3;
            else if (instruction == 6'h27)      // nor
                func = 4'd4;
            else if (instruction == 6'h2a)      // slt
                func = 4'd5;
            else
                func = 4'd15;
        end else if (ALUOp == 4'b0001) begin    // always sub
            func = 4'd1;
        end else if (ALUOp == 4'b0010) begin    // always add
            func = 4'd0;
        end else if (ALUOp == 4'b0011) begin    // pass-thru for lui
            func = 4'd6;

        // EXTRA CREDIT OPERATIONS
        end else if (ALUOp == 4'b0100) begin    // addiu
            func = 4'd7;
        end else if (ALUOp == 4'b0101) begin    // slti
            func = 4'd5;                        // reuse signed slt, just with imm control line
        end else if (ALUOp == 4'b0110) begin    // sltiu
            func = 4'd8;
        end else if (ALUOp == 4'b0111) begin    // andi
            func = 4'd2;                        // reuse and 
        end else if (ALUOp == 4'b1000) begin    // ori
            func = 4'd3;                        // reuse or
        end else if (ALUOp == 4'b1001) begin    // xori
            func = 4'd9;

        end else begin
            func = 4'd15;
        end
    end
endmodule
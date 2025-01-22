`timescale 1ns / 1ps

// Parameterized n-bit ALU, using parameterized n-bit function modules.
// Refer to the Lab 4 assignment diagram for the interpretation of ALUOp and R1-3.

module NBitALU #(parameter n = 32) (
    input [n-1:0] R2, R3,
    input [2:0] ALUOp,
    output [n-1:0] R1,
    output c_out,   // for the ADD module
    output c_out2   // for the SUB module
    );
    
    wire [n-1:0] mov_res, not_res, add_res, sub_res, or_res, and_res, slt_res;

    NBitMov #(.n(n)) Mov (
        .R2(R2),
        .R1(mov_res)
    );

    NBitNot #(.n(n)) Not (
        .R2(R2),
        .R1(not_res)
    );

    NBitAdder #(.n(n)) Add (
        .a(R2),
        .b(R3),
        .c_in(0),           // hard-wire c_in to zero for addition
        .c_out(c_out),
        .sum(add_res)
    );

    NBitSub #(.n(n)) Sub (
        .a(R2),
        .b(R3),
        .c_in(1),           // hard-wire c_in to one for subtraction ('2's complement addition')
        .c_out(c_out2),
        .sum(sub_res)
    );
    
    NBitOr #(.n(n)) Or (
        .R2(R2),
        .R3(R3),
        .R1(or_res)
    );
    
    NBitAnd #(.n(n)) And (
        .R2(R2),
        .R3(R3),
        .R1(and_res)
    );
    
    NBitSLT #(.n(n)) SetLessThan (
        .R2(R2),
        .R3(R3),
        .R1(slt_res)
    );

    // 3-8 MUX to drive R1 appropriately according to ALUOp
    assign R1 = (ALUOp[2]) ?
        (ALUOp[1] ?                             // MSB is 1
            slt_res :                           // ALUOp = 110
            (ALUOp[0] ? and_res : or_res)) :    // ALUOp = 101 : 100
        (ALUOp[1] ?                             // MSB is 0
            (ALUOp[0] ? sub_res : add_res) :    // ALUOp = 011 : 010
            (ALUOp[0] ? not_res : mov_res));    // ALUOp = 001 : 000

endmodule

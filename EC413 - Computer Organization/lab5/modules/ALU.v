`timescale 1ns / 1ps

// Behavioral ALU, modified from Ideal_ALU module provided with assignment.

module BehavioralALU #(word_size = 32) (      // word_size default value = 32
    input [word_size-1:0] R2, R3,
    input [2:0] ALUOp,

    output reg [word_size-1:0] R1   // note: R1 here is not a D-flip-flop; it just declares a variable here
    );                              // a "reg" without the "always @ (posedge clk)" is not a D-flip-flop

    always @ (R2, R3, ALUOp)        // when any of R2, R3, ALUOp changes, R1 will change
    begin
        case (ALUOp)
            3'h0: R1 = R2;
            3'h1: R1 = ~R2;
            3'h2: R1 = (R2 + R3);
            3'h3: R1 = (R2 - R3);
            3'h4: R1 = (R2 | R3);
            3'h5: R1 = (R2 & R3);
            3'h6: R1 = (($signed(R2) < $signed(R3))? 1:0);
        endcase
    end
endmodule

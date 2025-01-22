`timescale 1ns / 1ps

// Parameterized N-bit 2x1 MUX: inputs can be N bits in width, but MUX is 2x1 only.

module NBit2x1Mux #(N = 32) (
    input [N-1:0] MuxIn1, MuxIn2,
    input MuxSel,

    output [N-1:0] MuxOut
    );

    assign MuxOut = MuxSel ? MuxIn2 : MuxIn1;
endmodule

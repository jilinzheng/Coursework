`timescale 1ns / 1ps

// Mux select circuitry; in between carry select stages. Logic obtained from lecture notes.

`define DELAY 1 // delay 1 time unit for AND and OR gates

module MuxSelect(
    input prev_mux_select,
    input prev_RCA1_c_out,
    input prev_RCA0_c_out,
    output select
    );

    wire and_out;

    // mux select circuitry: select 1 if prev mux = 1 AND c_out = 1 from prev RCA with c_in = 1,
    //                       OR c_out = 1 from prev RCA with c_in = 0
    and #`DELAY and1 (and_out, prev_mux_select, prev_RCA1_c_out);
    or #`DELAY or1 (select, and_out, prev_RCA0_c_out);

endmodule

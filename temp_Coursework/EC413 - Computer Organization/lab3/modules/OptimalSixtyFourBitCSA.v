`timescale 1ns / 1ps

// OPTIMAL 64-bit Carry Select Adder using fifteen 8-bit Ripple Carry Adders.

module OptimalSixtyFourBitCSA(
    input [63:0] a, b,
    input c_in,
    output [63:0] sum,
    output c_out
    );

    wire [6:0] mux_select;                          // for the mux select circuitry
    wire [6:0] carry_zero_c_out, carry_one_c_out;   // to track the c_outs of hard-wired RCAs
    wire [55:0] carry_zero_sum, carry_one_sum;      // only need 56 bits, as the first eight bits only use one RCA

    // 1 8-bit RCA for the lowest 8 bits, no mux select circuitry for the first stage
    EightBitRCA RCA (
        .c_out(mux_select[0]),
        .sum(sum[7:0]),
        .a(a[7:0]),
        .b(b[7:0]),
        .c_in(c_in)
    );

    genvar i;
    generate

    // generate 7 8-bit RCAs hardwired to c_in = 0
    // skip the first 8 bits in inputs a and b, as the first RCA took care of the lowest 8 bits
    for (i = 0; i < 7; i = i + 1) begin : RCAs
        EightBitRCA RCA0 (
            .c_out(carry_zero_c_out[i]),
            .sum(carry_zero_sum[7+(8*i):(8*i)]),
            .a(a[15+(8*i):8+(8*i)]),
            .b(b[15+(8*i):8+(8*i)]),
            .c_in(1'b0)
        );

    // generate 7 8-bit RCAs hardwired to c_in = 1
    // skip the first 8 bits in inputs a and b, as the first RCA took care of the lowest 8 bits
        EightBitRCA RCA1 (
            .c_out(carry_one_c_out[i]),
            .sum(carry_one_sum[7+(8*i):(8*i)]),
            .a(a[15+(8*i):8+(8*i)]),
            .b(b[15+(8*i):8+(8*i)]),
            .c_in(1'b1)
        );
    end

    // mux select circuitry used to select subsequent RCAs
    for (i = 1; i < 7; i = i + 1) begin : MuxSelectCircuitry
        MuxSelect MS (
            .prev_mux_select(mux_select[i-1]),
            .prev_RCA1_c_out(carry_one_c_out[i-1]),
            .prev_RCA0_c_out(carry_zero_c_out[i-1]),
            .select(mux_select[i])
        );

    end

    // select RCA based on mux select circuitry
    // start at [15:8] to skip the first 8 bits that have already been assigned by the first RCA
    for (i = 0; i < 7; i = i + 1) begin : RCASelect
        assign sum[15+(8*i):8+(8*i)] = mux_select[i]
                                    ? carry_one_sum[7+(8*i):(8*i)]
                                    : carry_zero_sum[7+(8*i):(8*i)];
    end

    endgenerate

    // for the final c_cout
    assign c_out = mux_select[6] ? carry_one_c_out[6] : carry_zero_c_out[6];

endmodule

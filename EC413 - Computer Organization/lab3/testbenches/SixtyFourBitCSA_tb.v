`timescale 1ns / 1ps

// Testbench for 64-bit Carry Select Adder.

`define TOTAL_GATE_DELAY 64;    // total time of adder in gate delays
`define VERIFY_TIME_UNITS 10;   // time units to wait before switching inputs,
                                // to verify results visually using waveform

module SixtyFourBitCSA_tb;
    reg [63:0] a, b;
    reg c_in;
    wire [63:0] sum, sum_verify;
    wire c_out, c_out_verify;
    
    // instantiate UUT
    SixtyFourBitCSA CSA (
        .c_out(c_out),
        .sum(sum),
        .a(a),
        .b(b),
        .c_in(c_in)
    );
    
    // instantiate verification
    SixtyFourBitVerification verify (
        .c_out(c_out_verify),
        .sum(sum_verify),
        .a(a),
        .b(b),
        .c_in(c_in)
    );
    
    // assign error flag
    assign error_flag = (c_out != c_out_verify || sum[63:0] != sum_verify[63:0]);

    // for the final 'loop test'
    integer i;

    initial begin
        i = 0;
        a = 0; b = 0; c_in = 0;
        #`TOTAL_GATE_DELAY;
        #`VERIFY_TIME_UNITS;
        
        // test case 1: large numbers with c_out
        a = 64'hFFFFFFFFFFFFFFFF; b = 64'hF000000000000000; c_in = 0; #`TOTAL_GATE_DELAY; #`VERIFY_TIME_UNITS;
        $display("test 1: %h + %h + %b = %h, c_out = %b", a, b, c_in, sum, c_out);
        // expected: sum = 0xEFFFFFFFFFFFFFFF, c_out = 1
        if(error_flag) begin
            $display("Error occurs when a = %h, b = %h, c_in = %b, result should be %h\n", a, b, c_in, sum_verify);
        end else begin
            $display("Test passed.\n");
        end;
    
        // test case 2: large numbers without c_out
        a = 64'h7FFFFFFFFFFFFFFF; b = 64'h7FFFFFFFFFFFFFFE; c_in = 0; #`TOTAL_GATE_DELAY; #`VERIFY_TIME_UNITS;
        $display("test 2: %h + %h + %b = %h, c_out = %b", a, b, c_in, sum, c_out);
        // expected: sum = 0xFFFFFFFFFFFFFFFD, c_out = 0
        if(error_flag) begin
            $display("Error occurs when a = %h, b = %h, c_in = %b, result should be %h\n", a, b, c_in, sum_verify);
        end else begin
            $display("Test passed.\n");
        end;
    
        // test case 3: small numbers with c_out
        a = 64'h0000000000000001; b = 64'hFFFFFFFFFFFFFFFF; c_in = 0; #`TOTAL_GATE_DELAY; #`VERIFY_TIME_UNITS;
        $display("test 3: %h + %h + %b = %h, c_out = %b", a, b, c_in, sum, c_out);
        // expected: sum = 0x0000000000000000, c_out = 1
        if(error_flag) begin
            $display("Error occurs when a = %h, b = %h, c_in = %b, result should be %h\n", a, b, c_in, sum_verify);
        end else begin
            $display("Test passed.\n");
        end;
    
        // test case 4: small numbers without c_out
        a = 64'h000000000000002A; b = 64'h0000000000000049; c_in = 0; #`TOTAL_GATE_DELAY; #`VERIFY_TIME_UNITS;
        $display("test 4: %h + %h + %b = %h, c_out = %b", a, b, c_in, sum, c_out);
        // expected: sum = 0x0000000000000073, c_out = 0
        if(error_flag) begin
            $display("Error occurs when a = %h, b = %h, c_in = %b, result should be %h\n", a, b, c_in, sum_verify);
        end else begin
            $display("Test passed.\n");
        end;
    
        // test case 5: adding max value to 1 (overflow)
        a = 64'hFFFFFFFFFFFFFFFF; b = 64'h0000000000000001; c_in = 0; #`TOTAL_GATE_DELAY; #`VERIFY_TIME_UNITS;
        $display("test 5: %h + %h + %b = %h, c_out = %b", a, b, c_in, sum, c_out);
        // expected: sum = 0x0000000000000000, c_out = 1
        if(error_flag) begin
            $display("Error occurs when a = %h, b = %h, c_in = %b, result should be %h\n", a, b, c_in, sum_verify);
        end else begin
            $display("Test passed.\n");
        end;
    
        // test case 6: adding 0 to max value
        a = 64'h0000000000000000; b = 64'hFFFFFFFFFFFFFFFF; c_in = 0; #`TOTAL_GATE_DELAY; #`VERIFY_TIME_UNITS;
        $display("test 6: %h + %h + %b = %h, c_out = %b", a, b, c_in, sum, c_out);
        // expected: sum = 0xFFFFFFFFFFFFFFFF, c_out = 0
        if(error_flag) begin
            $display("Error occurs when a = %h, b = %h, c_in = %b, result should be %h\n", a, b, c_in, sum_verify);
        end else begin
            $display("Test passed.\n");
        end;
    
        // test case 7: random combination with c_out
        a = 64'hAB54A98CEB1F0AD2; b = 64'h892B007B95AF7C51; c_in = 0; #`TOTAL_GATE_DELAY; #`VERIFY_TIME_UNITS;
        $display("test 7: %h + %h + %b = %h, c_out = %b", a, b, c_in, sum, c_out);
        // expected: sum = 0x347FA90880CE8723, c_out = 1
        if(error_flag) begin
            $display("Error occurs when a = %h, b = %h, c_in = %b, result should be %h\n", a, b, c_in, sum_verify);
        end else begin
            $display("Test passed.\n");
        end;
    
        // test case 8: random combination without c_out
        a = 64'h4D0C3DCE2DE3E9B3; b = 64'h0F6B75E7C98E9273; c_in = 0; #`TOTAL_GATE_DELAY; #`VERIFY_TIME_UNITS;
        $display("test 8: %h + %h + %b = %h, c_out = %b", a, b, c_in, sum, c_out);
        // expected: sum = 0x5C77B3B607D27C26, c_out = 0
        if(error_flag) begin
            $display("Error occurs when a = %h, b = %h, c_in = %b, result should be %h\n", a, b, c_in, sum_verify);
        end else begin
            $display("Test passed.\n");
        end;
    
        // test case 9: adding two numbers that sum to max value
        a = 64'h7FFFFFFFFFFFFFFF; b = 64'h8000000000000000; c_in = 0; #`TOTAL_GATE_DELAY; #`VERIFY_TIME_UNITS;
        $display("test 9: %h + %h + %b = %h, c_out = %b", a, b, c_in, sum, c_out);
        // expected: sum = 0xFFFFFFFFFFFFFFFF, c_out = 0
        if(error_flag) begin
            $display("Error occurs when a = %h, b = %h, c_in = %b, result should be %h\n", a, b, c_in, sum_verify);
        end else begin
            $display("Test passed.\n");
        end;
    
        // test case 10: random combination with alternating bit patterns
        a = 64'h5555555555555555; b = 64'hAAAAAAAAAAAAAAAA; c_in = 0; #`TOTAL_GATE_DELAY; #`VERIFY_TIME_UNITS;
        $display("test 10: %h + %h + %b = %h, c_out = %b", a, b, c_in, sum, c_out);
        // expected: sum = 0xFFFFFFFFFFFFFFFF, c_out = 0
        if(error_flag) begin
            $display("Error occurs when a = %h, b = %h, c_in = %b, result should be %h\n", a, b, c_in, sum_verify);
        end else begin
            $display("Test passed.\n");
        end;
    
        // test case 11: adding a number to its two's complement
        a = 64'h112210F47DE98115; b = 64'hEEDDEF0B8216FEEB; c_in = 0; #`TOTAL_GATE_DELAY; #`VERIFY_TIME_UNITS;
        $display("test 11: %h + %h + %b = %h, c_out = %b", a, b, c_in, sum, c_out);
        // expected: sum = 0x0000000000000000, c_out = 1
        if(error_flag) begin
            $display("Error occurs when a = %h, b = %h, c_in = %b, result should be %h\n", a, b, c_in, sum_verify);
        end else begin
            $display("Test passed.\n");
        end;
    
        // test case 12: random combination with c_out in middle bits
        a = 64'h7FFFFFFFFFFFFFFF; b = 64'h7FFFFFFF00000000; c_in = 0; #`TOTAL_GATE_DELAY; #`VERIFY_TIME_UNITS;
        $display("test 12: %h + %h + %b = %h, c_out = %b", a, b, c_in, sum, c_out);
        // expected: sum = 0xFFFFFFFEFFFFFFFF, c_out = 0
        if(error_flag) begin
            $display("Error occurs when a = %h, b = %h, c_in = %b, result should be %h\n", a, b, c_in, sum_verify);
        end else begin
            $display("Test passed.\n");
        end;
    
        // test case 13: adding 1 with c_in = 1
        a = 64'h0000000000000001; b = 64'h0000000000000000; c_in = 1; #`TOTAL_GATE_DELAY; #`VERIFY_TIME_UNITS;
        $display("test 13: %h + %h + %b = %h, c_out = %b", a, b, c_in, sum, c_out);
        // expected: sum = 0x0000000000000002, c_out = 0
        if(error_flag) begin
            $display("Error occurs when a = %h, b = %h, c_in = %b, result should be %h\n", a, b, c_in, sum_verify);
        end else begin
            $display("Test passed.\n");
        end;
    
        // test case 14: all 1's with c_in = 1
        a = 64'hFFFFFFFFFFFFFFFF; b = 64'hFFFFFFFFFFFFFFFF; c_in = 1; #`TOTAL_GATE_DELAY; #`VERIFY_TIME_UNITS;
        $display("test 14: %h + %h + %b = %h, c_out = %b", a, b, c_in, sum, c_out);
        // expected: sum = 0xFFFFFFFFFFFFFFFF, c_out = 1
        if(error_flag) begin
            $display("Error occurs when a = %h, b = %h, c_in = %b, result should be %h\n", a, b, c_in, sum_verify);
        end else begin
            $display("Test passed.\n");
        end;
    
        // test case 15: random numbers with c_in = 1
        a = 64'h123456789ABCDEF0; b = 64'hFEDCBA9876543210; c_in = 1; #`TOTAL_GATE_DELAY; #`VERIFY_TIME_UNITS;
        $display("test 15: %h + %h + %b = %h, c_out = %b", a, b, c_in, sum, c_out);
        // expected: sum = 0x1111111111111101, c_out = 1
        if(error_flag) begin
            $display("Error occurs when a = %h, b = %h, c_in = %b, result should be %h\n", a, b, c_in, sum_verify);
        end else begin
            $display("Test passed.\n");
        end;
    
        // run another 'few' tests with random numbers
        $display("\nLoop test:");
        for (i = 1; i < 100000; i = i + 1) begin
            {a, b, c_in} = {a, b, c_in} * i + i; #`TOTAL_GATE_DELAY; #`VERIFY_TIME_UNITS;
            if(error_flag) begin
                $display("Error occurs when a = %h, b = %h, c_in = %b, result is %h but should be %h\n", a, b, c_in, sum, sum_verify);
            end else begin
                $display("Test passed: %h + %h + %b = %h, c_out = %b\n", a, b, c_in, sum, c_out);
            end;
        end
    
        $finish;
        
    end
    
endmodule

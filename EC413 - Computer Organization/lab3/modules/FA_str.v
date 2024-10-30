`timescale 1ns / 1ns

module FA_str(c_out, sum, a, b, c_in);
    input   a, b, c_in; //declare inputs a, b and c_in, 1 bit each
    output  c_out, sum; //declare outputs c_out and sum, 1 bit each
    
    //declare wires for values being passed between different gates
    wire    not_a, not_b, not_c;
    wire    and1_out, and2_out, and3_out, and4_out, and5_out, and6_out, and7_out;
    
    not    not1		(not_a, a);
    not	   not2		(not_b, b);
    not	   not3		(not_c, c_in);
    
    // get sum
    AND_T_3     and1       (and1_out, a, not_b, not_c);
    AND_T_3		and2       (and2_out, not_a, b, not_c);
    AND_T_3		and3       (and3_out, not_a, not_b, c_in);
    AND_T_3		and4       (and4_out, a, b, c_in);
    OR_T_4		or1        (sum, and1_out, and2_out, and3_out, and4_out);
    
    AND_T_2		and5       (and5_out, a, c_in);
    AND_T_2		and6       (and6_out, b, c_in);
    AND_T_2		and7       (and7_out, a, b);
    OR_T_3		or2        (c_out, and5_out, and6_out, and7_out);

endmodule

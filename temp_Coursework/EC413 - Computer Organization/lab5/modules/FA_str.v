`timescale 1ns / 1ns

// Structural Full Adder module provided in lab 4.

module FA_str(c_out, sum, a, b, c_in);

//parameter D = 0;		

input		a, b, c_in; //declare inputs a, b and c_in, 1 bit each
output	c_out, sum; //declare outputs c_out and sum, 1 bit each
//declare wires for values being passed between different gates
wire		not_a, not_b, not_c;
wire		and1_out, and2_out, and3_out, and4_out;
wire		and5_out, and6_out, and7_out, and8_out;

not	   not1		(not_a, a);
not	   not2		(not_b, b);
not		not3		(not_c, c_in);

// get sum
and			and1		(and1_out, a, not_b, not_c);
and			and2		(and2_out, not_a, b, not_c);
and			and3		(and3_out, not_a, not_b, c_in);
and			and4		(and4_out, a, b, c_in);
or				or1		(sum, and1_out, and2_out, and3_out, and4_out);

// get c_out							
and			and5		(and5_out, a, b);
and			and6		(and6_out, b, c_in);
and			and7		(and7_out, a, c_in);
and			and8		(and8_out, a, b, c_in);
or				or2		(c_out, and5_out, and6_out, and7_out, and8_out);


endmodule

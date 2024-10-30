`timescale 1ns / 1ps

module Verification_1bit(c_out, sum, a, b, c_in);

	input a, b, c_in; //declare inputs a, b, and c_in, one bit each
	output c_out, sum; //declare outputs c_out and sum, one bit each

	assign {c_out, sum} = a + b + c_in;

endmodule

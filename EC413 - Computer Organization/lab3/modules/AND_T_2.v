`timescale 1ns / 1ps

`define	D 1	// definition of the delay

// Delayed AND gate

module AND_T_2(out, in1, in2);
    input in1, in2;
    output out;
    
    and	#`D	and1(out, in1, in2);

endmodule

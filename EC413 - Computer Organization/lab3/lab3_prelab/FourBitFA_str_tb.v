`timescale 1ns / 1ps

module FourBitFA_str_tb;
    reg clk;
    reg [3:0] a, b;
    reg c_in;
    wire [3:0] sum, sum_verify;
    wire c_out, c_out_verify;
    
    // instantiate UUT
    FourBitFA_str FourBitFA (
        .c_out(c_out),
        .sum(sum),
        .a(a),
        .b(b),
        .c_in(c_in)
    );
    
    // instantiate verification
    FourBitVerification verify (
        .c_out(c_out_verify),
        .sum(sum_verify),
        .a(a),
        .b(b),
        .c_in(c_in)
    );
    
    // assign error flag
	assign error_flag = (c_out != c_out_verify || sum[3:0] != sum_verify[3:0]);
	
	// report errors if encountered
	always@(posedge clk)
		begin
		if(error_flag)
			$display("Error occurs when a = %d, b = %d, c_in = %d\n", a, b, c_in);
		end
		
	// 1 cycle every 10 ns
	always #5 clk = ~clk;

	initial begin
		clk = 0;
		
		a = 0;
		b = 0;
		c_in = 0;

	end
	
	always #10 {c_in, a, b} = {c_in, a, b} + 1;

endmodule

`timescale 1ns / 1ps

module FSM_debounced_tb();

    reg clk, reset, read;
    reg [7:0] in;
    
    wire debouncedRead;
    wire out;
    wire [1:0] state;
    
    debouncer readDebouncer(clk, read, reset, debouncedRead);
    moore_FSM theFSM(out, clk, reset, debouncedRead, in, state);
    
    always #5 clk = ~clk;
    
    initial begin
    clk = 0;
    reset = 0;
    #1 reset = 1;
    #0 reset = 0;
    read = 0;
    in = 8'b10110111;
    
    #10 read = 1;   // debouncedRead goes high at 45 ns and stays high for 160 ns
    
    end

endmodule

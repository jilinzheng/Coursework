`timescale 1ns / 1ps

module moore_FSM_tb();
    
    reg clk, reset, read;
    reg [7:0] in;
    wire out;
    wire [1:0] next_state_display;
    
    moore_FSM theFSM(out,clk,reset,read,in,next_state_display);
    
    always #5 clk = ~clk;   // Clock generator
    
    initial begin
        clk = 0;
        reset = 0;
        read = 0;
        in = 8'b10111101;
        
        // 5 ns after beginning of simulation, next state will be S0
        #10 reset = 1;    // Move back to S3 on the next clk posedge
        
        // 15 ns after beginning of simulation, next state will be S3
        #10 reset = 0;    // Move to S0 on the next clk posedge
        
        // 25 ns after beginning of simulation, next state will be S0
        #10 read = 1;     // Move to S1 on the next clk posedge
        
        // 35 ns after beginning of simulation, next state will be S1
        #10 read = 0;
        
        // 45 ns after beginning of simulation, next state will be S2
            // out should be in[p = 0] = 1 
            
        // 55 ns after beginning of simulation, next state will be S0
            // stay for rest of sim
            
        #45 $finish;     
    end   
    
endmodule

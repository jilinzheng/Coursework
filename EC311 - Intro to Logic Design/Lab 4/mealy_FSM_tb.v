`timescale 1ns / 1ps

module mealy_FSM_tb(

    );
    
    reg clk, in;
    wire out;
    wire [1:0] state;
    
    mealy_FSM theFSM(clk,in,out,state);
    
    always #5 clk = ~clk;
    
    initial begin
        clk = 0;
        in = 0;
        
        // 5 ns after beginning of simulation, out <= 0  and state <= 2
        #10 in = 1;
        
        // 15 ns after beginning of simulation, out <= 0 and state <= 1
                
        // 25 ns after beginning of simulation, out <= 1 and state <= 1 (self-loop)
        #20 in = 0;
        
        // 35 ns after beginning of simulation, out <= 1 and state <= 2
                 
        // 45 ns after beginning of simulation, out <= 0 and state <= 2 (self-loop)
        #20 in = 1;
        
        // 55 ns after beginning of simulation, out <= 0 and state <= 1
        
        #10 $finish;
    end
    
endmodule

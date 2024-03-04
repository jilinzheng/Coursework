`timescale 1ns / 1ps

/*
Note: I added a parameter 'state' to keep track of the upcoming state,
      primarily for the testbench/simulation.
*/
module moore_FSM(out, clk, reset, read, in, state);
    input clk, reset, read; 
    input [7:0] in;
    output reg out;
    output reg [1:0] state;
    
    reg [1:0] p;
    reg [1:0] present_state, next_state;    
    
    parameter S0 = 0, S1 = 1, S2 = 2, S3 = 3;
    
    initial present_state = S3;
    initial state = present_state;
    initial out = 0;
    
    /*
    always @(reset or read or present_state)
        case(present_state)
            S0: if(reset)
                    next_state = S3;
                else if(read)
                    next_state = S1;
            S1: begin
                out = in[p];
                next_state = S2;
                end
            S2: begin
                p <= p + 1;
                next_state = S0;
                end
            S3: begin
                p <= 0;
                out <= 0;
                next_state = S0;
                end    
        endcase
    */
    
    always @(posedge clk) begin
        case(present_state)
            S0: if(reset)
                    next_state = S3;
                else if(read)
                    next_state = S1;
            S1: begin
                out <= in[p];
                next_state = S2;
                end
            S2: begin
                p <= p + 1;
                next_state = S0;
                end
            S3: begin
                p <= 0;
                out <= 0;
                next_state = S0;
                end    
        endcase
        present_state = next_state;
        state = present_state;        
    end
endmodule
`timescale 1ns / 1ps

module mealy_FSM(
    input clk,
    input in,
    output reg out,
    output reg [1:0] state
    );

    initial state = 0;
    initial out = 0;
    
    always @ (posedge clk) begin
        case (state)
            0:  begin
                    if (in) begin
                        out <= 0;
                        state <= 1;
                    end
                    else begin
                        out <= 0;
                        state <= 2;
                    end
                end
                
            1:  begin
                    if (in) begin
                        out <= 1;
                        state <= 1;
                    end
                    else begin
                        out <= 1;
                        state <= 2;
                    end
                end
                
            2:  begin
                    if (in) begin
                        out <= 0;
                        state <= 1;
                    end
                    else begin
                        out <= 0;
                        state <= 2;
                    end
                end
        endcase
    end
endmodule

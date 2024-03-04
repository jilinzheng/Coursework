`timescale 1ns / 1ps
// Original debouncer.v
/*
module debouncer(
    input wire clk,     
    input wire rst,     
    input wire button,             // Button input
    output reg debounced_button    // Debounced button output
);
    reg [1:0] state, next_state;
    // Parameter for debounce delay (adjust this based on your clock frequency)
    parameter DELAY_COUNT = 50000;  // You may need to adjust this value
    reg [15:0] debounce_counter;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= 2'b00;
            debounce_counter <= 16'b0;
        end else begin
            state <= next_state;
            // Add code that increments the debouncer counter based on whether it is equal to DELAY_COUNT
            debounce_counter <= debounce_counter + 1;
            if (debounce_counter == DELAY_COUNT) begin
                
            end 
        end
    end
    
    // add state logic with IDLE being 2'b00, PRESSED being 2'b01, and RELEASED as 2'b10
    always @(posedge clk or posedge rst) begin
        case (state)
            2'b00: begin    // IDLE state
                if (button == 1'b0) begin
                    next_state <= 2'b01;    //fill in missing next state
                end else begin
                    next_state <= //fill in missing next state
                end
            end
    
            2'b01: begin    // PRESSED state
                if (button == 1'b0 && debounce_counter == DELAY_COUNT) begin
                    next_state <= //fill in next state
                end else if (button == 1'b1) begin
                    next_state <= //fill in next state
                end else begin
                    next_state <= //fill in next state
                end
            end
    
            2'b10: begin    // RELEASE_DETECTED state
                if (button == 1'b1 && debounce_counter == DELAY_COUNT) begin
                    next_state <= //fill in next state
                end else if (button == 1'b0) begin
                    next_state <= //fill in next state
                end else begin
                    next_state <= //fill in next state
                end
            end
    
            default: next_state <= 2'b00;
        endcase
    end
    
    // below is where debounced signal is outputted
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            debounced_button <= 1'b0;
        end else begin
            case (state)
                2'b10: debounced_button <= 1'b1;      // Output 1 when RELEASED
                default: debounced_button <= 1'b0;    // Output 0 otherwise
            endcase
        end
    end
endmodule
*/
// Alternative implementation from Fadi

module debouncer(
    input clk,     
    input button,
    input reset,        // Button input
    output reg clean    // Debounced button output
);
    reg [1:0] deb_count;
    reg [3:0] end_count;
    
    reg output_exist;
    reg [1:0] max = 2'b11;
    reg [3:0] endmax = 4'b1111;
    
    /*
    always @(posedge reset) begin
        deb_count <= 0;
        end_count <= 0;
        output_exist <=0;
    end
    */

    // Below is where debounced signal is outputted
    always @(posedge clk) begin
        if (reset == 1'b1) begin
            deb_count <= 0;
            end_count <= 0;
            output_exist <=0;
            clean <= 0;
        end
        
        if (button == 1'b1) begin
            if (output_exist == 1'b0) begin
                if (deb_count == max) begin
                    clean = 1;
                    if (end_count == endmax) begin
                        end_count = 0;
                        deb_count = 0;
                        output_exist = 1;
                    end
                    else begin
                        end_count = end_count + 4'b0001;
                    end
                end
                else begin
                    deb_count = deb_count + 4'b0001;
                end
            end
            else begin
                clean = 0;
            end
        end
        else begin
            deb_count =0;
            end_count = 0;
            output_exist = 0;
            clean = 0;
        end
    end
endmodule

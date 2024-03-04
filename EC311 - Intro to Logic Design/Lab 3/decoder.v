`timescale 1ns / 1ps

// The decoder
module decoder(
        input in_clk,           // Clock
        input incrementBtn,     // Button to increment input one at a time
        input saveBtn,          // Button to save value user inputted and continue to either next input or display output based on opCode
        input resetBtn,         // Button to reset user's inputs
               
        output reg [3:0] A,     // 4-bit output A for ALU
        output reg [3:0] B,     // 4-bit output B for ALU
        output btnPressLED,     // LED indicating if a button was pressed
        output reg state        // LED indicating whether A or B is currently selected
    );
       
    wire incrementBtnPress, saveBtnPress, resetBtnPress; // To track which button is pressed
    
    // Initial Conditions
    initial begin
        A = 0;              // Start user's input at zero
        B = 0;              // Start user's input at zero
        state = 0;
    end
    
    debouncingPushButton increment(incrementBtn,in_clk,incrementBtnPress);
    debouncingPushButton save(saveBtn,in_clk,saveBtnPress);
    debouncingPushButton reset(resetBtn,in_clk,resetBtnPress);

    wire slow_clk;
    slowClock newSlowClock(in_clk,slow_clk);
    always @ (posedge slow_clk) begin
        if (incrementBtnPress == 1) begin
            if (state == 0) begin
                A <= A + 1;
            end
            else if (state == 1) begin
                B <= B + 1;
            end
        end
        if (saveBtnPress == 1) begin
            state <= state + 1;
        end
        if (resetBtnPress == 1) begin
            A <= 0;
            B <= 0;
        end
    end
    
    // Light LED whenever a button is pressed
    assign btnPressLED = incrementBtnPress | saveBtnPress | resetBtnPress; 
    
endmodule

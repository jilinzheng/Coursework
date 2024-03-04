`timescale 1ns / 1ps

// NOTE: TESTBENCH WILL NOT WORK UNLESS CLOCK SPEED IS RAISED BY MODIFYING slowClock.v
// I.E. CHANGE THE IF CONDITIONAL in slowClock.v TO (count == 2) (or something close to it)

module decoder_test(

    );
    
    reg in_clk, incrementBtn, saveBtn, resetBtn;
    wire [3:0] A, B;
    wire btnPressLED, state;
    
    always #1 in_clk = ~in_clk;
    
    decoder DUT(in_clk, incrementBtn, saveBtn, resetBtn, A, B, btnPressLED, state);
    
    initial begin
        in_clk = 0;
        incrementBtn = 0;
        saveBtn = 0;
        resetBtn = 0;
        
        #10 incrementBtn = 1;
        #10 incrementBtn = 0;
        #10 incrementBtn = 1;
        #10 incrementBtn = 0;
        #10 incrementBtn = 1;
        #10 incrementBtn = 0;
        #10 incrementBtn = 1;
        #10 incrementBtn = 0;
        #10 incrementBtn = 1;
        #10 incrementBtn = 0; // A should be 5
        
        #10 saveBtn = 1;      // B is new target, state should be 1
        #10 saveBtn = 0;
        
        #10 incrementBtn = 1;
        #10 incrementBtn = 0;
        #10 incrementBtn = 1;
        #10 incrementBtn = 0;
        #10 incrementBtn = 1;
        #10 incrementBtn = 0;
        #10 incrementBtn = 1;
        #10 incrementBtn = 0; // B should be 4
        
        #10 resetBtn = 1;
        #10 resetBtn = 0;     // A and B should be reset to 0
        
    end
    
endmodule

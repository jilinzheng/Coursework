`timescale 1ns / 1ps

// NOTE: TESTBENCH WILL NOT WORK UNLESS CLOCK SPEED IS RAISED BY MODIFYING slowClock.v
// I.E. CHANGE THE IF CONDITIONAL in slowClock.v TO (count == 2) (or something close to it)

module fullSystem_test(

    );
    
    reg in_clk, incrementBtn, saveBtn, resetBtn, cIn, resetVGA;
    reg [3:0] opCode;
    wire btnPressLED, state, cOut, hsync, vsync;
    wire [3:0] A, B;
    wire [6:0] cathodes;
    wire [7:0] anodes;
    wire [11:0] rgb;
    
    fullSystem DUT (.in_clk(in_clk),.incrementBtn(incrementBtn),.saveBtn(saveBtn),.resetBtn(resetBtn),.A(A),.B(B),.btnPressLED(btnPressLED),.state(state),
                    .opCode(opCode),.cIn(cIn),.cOut(cOut),
                    .resetVGA(resetVGA),.hsync(hsync),.vsync(vsync),.rgb(rgb),
                    .anodes(anodes),.cathodes(cathodes)
                    );
                    
    always #1 in_clk = ~in_clk;
    
    initial begin
        in_clk = 0;
        incrementBtn = 0;
        saveBtn = 0;
        resetBtn = 0;
        cIn = 0;
        resetVGA = 0;
        opCode = 0;
        
        #10 opCode = 4'b1010;
        #10 incrementBtn = 1;
        #10 incrementBtn = 0;
        #10 incrementBtn = 1;
        #10 incrementBtn = 0;
        #10 incrementBtn = 1;
        #10 incrementBtn = 0;
        #10 incrementBtn = 1; 
        #10 incrementBtn = 0; // A = 4
        
        #10 saveBtn = 1;
        #10 saveBtn = 0;      // Switch target to B
        
        #10 incrementBtn = 1; // B = 1
        #10 incrementBtn = 0;
        
        #10 resetBtn = 1;     // Reset inputs A & B
        #10 resetBtn = 0;     
    end
    
endmodule

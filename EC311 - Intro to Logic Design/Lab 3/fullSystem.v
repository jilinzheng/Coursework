`timescale 1ns / 1ps

// The full system
module fullSystem(

        // For theDecoder
        input in_clk,               // On-board clock
        input incrementBtn,         // Button to increment inputs by one
        input saveBtn,              // Button to save the current input
        input resetBtn,             // Button to reset the inputs
        output [3:0] A,             // Input A
        output [3:0] B,             // Input B
        output btnPressLED,         // LED to indicate a button has been pressed
        output state,               // Determines whether A or B is being incremented/saved
        
        // For theFullALU
        input [3:0] opCode,         // Operation code (matches opCodes from Lab 2)
        input cIn,                  // Carry-in bit
        output cOut,                // Carry-out bit
        
       // For vgaControl
        input resetVGA,             // Reset VGA display
        output hsync,               // Horizontal synchronization
        output vsync,               // Vertical synchronization
        output [11:0] rgb,          // RGB output that sets color of every bit
        
        // Originally for debugging (uses 7-segment display)
        output [7:0] anodes,        // Control which digit of the 8 on the board to turn on
        output reg [6:0] cathodes   // Control which cathodes on the selected digit to turn on
        
    );
    
    decoder theDecoder(             // Instantiate theDecoder
        .in_clk(in_clk),
        .incrementBtn(incrementBtn),
        .saveBtn(saveBtn),
        .resetBtn(resetBtn),
        .A(A),
        .B(B),
        .btnPressLED(btnPressLED),
        .state(state)
    );
    
    wire [3:0] finalResult;         // To hold the finalResult of the operation performed by the ALU
    fullALU theFullALU(             // Instantiate theFullALU
        .opCode(opCode),
        .A(A),
        .B(B),
        .cIn(cIn),
        .finalResult(finalResult),
        .cOut(cOut)
    );
    
     assign anodes = 8'b11111110;    // Turn rightmost digit of 7-segment display on
    
    // Set the appropriate cathodes for a hex digit on (originally used for debugging)
    always @ (finalResult) begin
        case (finalResult)
            0: cathodes = 7'b0000001;
            1: cathodes = 7'b1001111;
            2: cathodes = 7'b0010010;
            3: cathodes = 7'b0000110;
            4: cathodes = 7'b1001100;
            5: cathodes = 7'b0100100;
            6: cathodes = 7'b0100000;
            7: cathodes = 7'b0001111;
            8: cathodes = 7'b0000000;
            9: cathodes = 7'b0000100;
            10: cathodes = 7'b0001000;      // Hex A
            11: cathodes = 7'b1100000;      // Hex B
            12: cathodes = 7'b0110001;      // Hex C
            13: cathodes = 7'b1000010;      // Hex D
            14: cathodes = 7'b0110000;      // Hex E
            15: cathodes = 7'b0111000;      // Hex F
            default: cathodes = 7'b1111111; // no display i.e. all cathodes off
        endcase
    end
    
    // Convert opCode, A, B, and finalReslt (of ALU) to hexadecimal ASCII codes for use in vgaControl
    wire [6:0] codedOpCode, codedA, codedB, codedFinalResult;
    toHexASCIICode convertOpCode(.fourBitInput(opCode), .sevenBitHexASCIICode(codedOpCode));
    toHexASCIICode convertA(.fourBitInput(A), .sevenBitHexASCIICode(codedA));
    toHexASCIICode convertB(.fourBitInput(B), .sevenBitHexASCIICode(codedB));
    toHexASCIICode convertFinalResult(.fourBitInput(finalResult), .sevenBitHexASCIICode(codedFinalResult));
    
    vga vgaControl(                 // Instantiate vgaControl
        .clk(in_clk),
        .reset(resetVGA),
        .A(codedA),
        .B(codedB),
        .opCode(codedOpCode),
        .finalResult(codedFinalResult),
        .hsync(hsync),
        .vsync(vsync),
        .rgb(rgb)
    );
    
endmodule

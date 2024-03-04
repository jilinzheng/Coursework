`timescale 1ns / 1ps

// Module that takes in a 4-bit input and outputs respective 7-bit ASCII hex codes
module toHexASCIICode(
        input [3:0] fourBitInput,
        output reg [6:0] sevenBitHexASCIICode
    );
    
    always @ (fourBitInput) begin
        case (fourBitInput)
            0: sevenBitHexASCIICode = 7'h30;
            1: sevenBitHexASCIICode = 7'h31;
            2: sevenBitHexASCIICode = 7'h32;
            3: sevenBitHexASCIICode = 7'h33;
            4: sevenBitHexASCIICode = 7'h34;
            5: sevenBitHexASCIICode = 7'h35;
            6: sevenBitHexASCIICode = 7'h36;
            7: sevenBitHexASCIICode = 7'h37;
            8: sevenBitHexASCIICode = 7'h38;
            9: sevenBitHexASCIICode = 7'h39;
            10: sevenBitHexASCIICode = 7'h41;
            11: sevenBitHexASCIICode = 7'h42;
            12: sevenBitHexASCIICode = 7'h43;
            13: sevenBitHexASCIICode = 7'h44;
            14: sevenBitHexASCIICode = 7'h45;
            15: sevenBitHexASCIICode = 7'h46;
        endcase
    end
        
endmodule

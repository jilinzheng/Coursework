`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/11/2023 11:36:16 PM
// Design Name: 
// Module Name: fullSystem
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module fullSystem(
        input cIn,
        input sevenSegControl, // 1-bit 7-segment display control: 0 shows opCode in hex, 1 shows ALU result in hex
        input [11:0] instruction, 
        
        output cOut,
        output [7:0] anodeControl, // Control which digit of the 8 on the board to turn on
        output [6:0] cathodeControl // Control which cathodes of the 7 on the digit to turn on
    );
    
    // Turn on left-most digit of the 7-segment display
    assign anodeControl = 8'b01111111;
    
    // Create wires for theDecoder to output to and for theFullAlu to use as inputs
    wire [3:0] opCode, A, B;
    decoder theDecoder(.instruction(instruction),.opCode(opCode),.A(A),.B(B));
    
    // Create wire for theFullAlu's output
    wire [3:0] aluResult;
    fullALU theFullALU(.opCode(opCode),.A(A),.B(B),.cIn(cIn),.cOut(cOut),.finalResult(aluResult));
    
    // Implement output for sevenSegDisplay
    reg [6:0] cathodes;
    always @ (sevenSegControl) begin // Run whenever the seven-segment control is changed
        if (sevenSegControl == 1) begin // Show ALU result in hex
            case (aluResult)
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
                10: cathodes = 7'b0001000; // hex A
                11: cathodes = 7'b1100000; // hex B
                12: cathodes = 7'b0110001; // hex C
                13: cathodes = 7'b1000010; // hex D
                14: cathodes = 7'b0110000; // hex E
                15: cathodes = 7'b0111000; // hex F
            endcase
        end
        else begin // Show opCode in hex
            case (opCode)
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
                10: cathodes = 7'b0001000; // hex A
                11: cathodes = 7'b1100000; // hex B
                12: cathodes = 7'b0110001; // hex C
                13: cathodes = 7'b1000010; // hex D
                14: cathodes = 7'b0110000; // hex E
                15: cathodes = 7'b0111000; // hex F
            endcase
        end
    end
       
    assign cathodeControl = cathodes; // Light it up baby! 
    
endmodule

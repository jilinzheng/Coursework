`timescale 1ns / 1ps

module toHexASCIICode_test(

    );
    
    reg [3:0] fourBitInput;
    wire [6:0] sevenBitHexASCIICode;
    
    toHexASCIICode convert(.fourBitInput(fourBitInput),.sevenBitHexASCIICode(sevenBitHexASCIICode));
    
    integer ii; // Iterator variable for loop
    initial begin
    
    // Loop through all possibilities of four-bit input
    for (ii = 0; ii < 16; ii = ii + 1) begin
        fourBitInput = ii;
        #10;
    end
    
    end
    
endmodule

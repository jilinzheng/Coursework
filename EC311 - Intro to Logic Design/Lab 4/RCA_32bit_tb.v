`timescale 1ns / 1ps

module RCA_32bit_tb();
    
    reg [31:0] a, b;                  // 32-bit inputs
    wire [32:0] actualOut, verOut;    // 33-bit outputs for the actual module and the verification module
    wire errorFlag;                   // Equals 1 if actualOut != verOut
    
    RCA_Nbit #(32) parametrizedRCA(a,b,actualOut);      // Instantiate actual module
    
    RCA_verification #(32) verification(a,b,verOut);    // Instantiate verification module
    
    assign errorFlag = (actualOut != verOut);           // Assign the error flag
    
    // Print errors to the console
    always @ (actualOut) begin
        if (errorFlag) begin
            $display ("Error occurs when a = %d and b = %d", a, b);
        end
    end
    
    // Initialize a = b = 0
    initial begin
        a = 0;
        b = 0;
        #500 a = 2147483647;
        #500 b = 2147483647;
    end
    
    // Exhaustively test values of a and b
    always begin
        #1 a = a + 1;
        #1 b = b + 1;
    end
    
endmodule

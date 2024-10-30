`timescale 1ns / 1ps

// Testbench for NBitRegisterFile module, modified from file provided with assignment.

module NBitRegisterFile_tb;
    // register select line size (number of registers is 2 to the power of this number)
    parameter REG_SELECT_WIDTH = 5;
    // register size in bits
    parameter DATA_WIDTH = 32;

    // inputs
    reg [DATA_WIDTH-1:0]       WriteData;
    reg [REG_SELECT_WIDTH-1:0] ReadSelect1;
    reg [REG_SELECT_WIDTH-1:0] ReadSelect2;
    reg [REG_SELECT_WIDTH-1:0] WriteSelect;
    reg WriteEnable;
    reg Reset;
    reg Clk;

    // outputs
    wire [DATA_WIDTH-1:0] ReadData1;
    wire [DATA_WIDTH-1:0] ReadData2;

    // instantiate the Unit Under Test (UUT)
    NBitRegisterFile #(DATA_WIDTH, REG_SELECT_WIDTH) uut (
        .WriteData(WriteData), 
        .ReadData1(ReadData1),
        .ReadData2(ReadData2), 
        .ReadSelect1(ReadSelect1), 
        .ReadSelect2(ReadSelect2), 
        .WriteSelect(WriteSelect),
        .WriteEnable(WriteEnable), 
        .Clk(Clk),
        .Reset(Reset)
    );

    initial begin
        // initial Reset, should only occur on rising edge
        WriteData= 32'h00000000; ReadSelect1= 0; ReadSelect2= 1; WriteSelect= 0; WriteEnable= 0; Reset= 1; Clk= 0;
        #20 WriteData= 32'h00000000; ReadSelect1= 0; ReadSelect2= 1; WriteSelect= 0; WriteEnable= 0; Reset= 1; Clk= 1;

        // write a value into register 0
        #20 WriteData= 32'hDEADBEEF; ReadSelect1= 0; ReadSelect2= 1; WriteSelect= 0; WriteEnable= 1; Reset= 0; Clk= 0;
        #20 WriteData= 32'hDEADBEEF; ReadSelect1= 0; ReadSelect2= 1; WriteSelect= 0; WriteEnable= 1; Reset= 0; Clk= 1;

        // attempt to write a value into register 0 but should not write because enable is not set
        #20 WriteData= 32'hBADF000D; ReadSelect1= 0; ReadSelect2= 1; WriteSelect= 0; WriteEnable= 0; Reset= 0; Clk= 0;
        #20 WriteData= 32'hBADF000D; ReadSelect1= 0; ReadSelect2= 1; WriteSelect= 0; WriteEnable= 0; Reset= 0; Clk= 1;

        // try a different read select and see that contents are 0
        #20 WriteData= 32'hBADF000D; ReadSelect1= 5; ReadSelect2= 1; WriteSelect= 0; WriteEnable= 0; Reset= 0; Clk= 0;
        #20 WriteData= 32'hBADF000D; ReadSelect1= 5; ReadSelect2= 1; WriteSelect= 0; WriteEnable= 0; Reset= 0; Clk= 1;

        // attempt again to write a value into register 0 but should not write because enable is not set
        #20 WriteData= 32'hBADF000D; ReadSelect1= 0; ReadSelect2= 1; WriteSelect= 0; WriteEnable= 0; Reset= 0; Clk= 0;
        #20 WriteData= 32'hBADF000D; ReadSelect1= 0; ReadSelect2= 1; WriteSelect= 0; WriteEnable= 0; Reset= 0; Clk= 1;
        
        // write a value into register 1 and read it back
        #20 WriteData = 32'h12345678; ReadSelect1 = 1; ReadSelect2 = 2; WriteSelect = 1; WriteEnable = 1; Reset = 0; Clk = 0;
        #20 WriteData = 32'h12345678; ReadSelect1 = 1; ReadSelect2 = 2; WriteSelect = 1; WriteEnable = 1; Reset = 0; Clk = 1;
        #20 WriteData = 32'h00000000; ReadSelect1 = 1; ReadSelect2 = 2; WriteSelect = 0; WriteEnable = 0; Reset = 0; Clk = 0;
        
        // write to register 31 (assuming 32 registers) and read it back
        #20 WriteData = 32'hFFFFFFFF; ReadSelect1 = 31; ReadSelect2 = 0; WriteSelect = 31; WriteEnable = 1; Reset = 0; Clk = 0;
        #20 WriteData = 32'hFFFFFFFF; ReadSelect1 = 31; ReadSelect2 = 0; WriteSelect = 31; WriteEnable = 1; Reset = 0; Clk = 1;
        #20 WriteData = 32'h00000000; ReadSelect1 = 31; ReadSelect2 = 0; WriteSelect = 0; WriteEnable = 0; Reset = 0; Clk = 0;
        
        // attempt again to write a value but should not write because enable is not set
        #20 WriteData = 32'hAAAAAAAA; ReadSelect1 = 2; ReadSelect2 = 3; WriteSelect = 2; WriteEnable = 0; Reset = 0; Clk = 0;
        #20 WriteData = 32'hAAAAAAAA; ReadSelect1 = 2; ReadSelect2 = 3; WriteSelect = 2; WriteEnable = 0; Reset = 0; Clk = 1;
        #20 WriteData = 32'h00000000; ReadSelect1 = 2; ReadSelect2 = 3; WriteSelect = 0; WriteEnable = 0; Reset = 0; Clk = 0;
        
        // test reset functionality
        #20 WriteData = 32'h00000000; ReadSelect1 = 1; ReadSelect2 = 31; WriteSelect = 0; WriteEnable = 0; Reset = 1; Clk = 0;
        #20 WriteData = 32'h00000000; ReadSelect1 = 1; ReadSelect2 = 31; WriteSelect = 0; WriteEnable = 0; Reset = 1; Clk = 1;
        #20 WriteData = 32'h00000000; ReadSelect1 = 1; ReadSelect2 = 31; WriteSelect = 0; WriteEnable = 0; Reset = 0; Clk = 0;
        
        // write to multiple registers in sequence
        #20 WriteData = 32'h11111111; ReadSelect1 = 3; ReadSelect2 = 4; WriteSelect = 3; WriteEnable = 1; Reset = 0; Clk = 0;
        #20 WriteData = 32'h11111111; ReadSelect1 = 3; ReadSelect2 = 4; WriteSelect = 3; WriteEnable = 1; Reset = 0; Clk = 1;
        #20 WriteData = 32'h22222222; ReadSelect1 = 4; ReadSelect2 = 5; WriteSelect = 4; WriteEnable = 1; Reset = 0; Clk = 0;
        #20 WriteData = 32'h22222222; ReadSelect1 = 4; ReadSelect2 = 5; WriteSelect = 4; WriteEnable = 1; Reset = 0; Clk = 1;
        #20 WriteData = 32'h33333333; ReadSelect1 = 5; ReadSelect2 = 6; WriteSelect = 5; WriteEnable = 1; Reset = 0; Clk = 0;
        #20 WriteData = 32'h33333333; ReadSelect1 = 5; ReadSelect2 = 6; WriteSelect = 5; WriteEnable = 1; Reset = 0; Clk = 1;
        
        // read from multiple registers in sequence
        #20 WriteData = 32'h00000000; ReadSelect1 = 3; ReadSelect2 = 4; WriteSelect = 0; WriteEnable = 0; Reset = 0; Clk = 0;
        #20 WriteData = 32'h00000000; ReadSelect1 = 4; ReadSelect2 = 5; WriteSelect = 0; WriteEnable = 0; Reset = 0; Clk = 0;
        #20 WriteData = 32'h00000000; ReadSelect1 = 5; ReadSelect2 = 3; WriteSelect = 0; WriteEnable = 0; Reset = 0; Clk = 0;

        #20 $finish;
    end
endmodule


`timescale 1ns / 1ps

// NBitRegisterFile module, modified from the n-bit register file module provided in prelab.
// First two comments below belong to original module.

// This can be used to replace the original nbit_register_file in lab5.
// In iSim you can actually expand this reg_file to view the contents in it.

module NBitRegisterFile #(DATA_WIDTH = 32, SELECT_WIDTH = 5) (
    input [SELECT_WIDTH-1:0] ReadSelect1, ReadSelect2, WriteSelect,
    input [DATA_WIDTH-1:0] WriteData,
    input WriteEnable, Clk, Reset,

    output [DATA_WIDTH-1:0] ReadData1, ReadData2
    );

    reg [DATA_WIDTH-1:0] register_file [0:DATA_WIDTH-1];

    // for loop initializes all registers to 0
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin 
            register_file[i] = 0;
        end
    end

    // drive read data continuously/combinational
    assign ReadData1 = register_file[ReadSelect1];
    assign ReadData2 = register_file[ReadSelect2];

    // sequential writes to registers
    always @ (posedge Clk) begin
        if (Reset) begin
            for (i = 0; i < 32; i = i + 1) begin 
                register_file[i] = 0;
            end
        end
        if (WriteEnable) 
            register_file[WriteSelect] <= WriteData;
    end
endmodule

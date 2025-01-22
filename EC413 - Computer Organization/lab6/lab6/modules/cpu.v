`timescale 1ns / 1ps

// 8 bit data
// 4 bit wide address for memories and reg file
// 32 bit wide instruction
// 4 bit immediate

module cpu(
    rst,
    clk,
    initialize,
    instruction_initialize_data,
    instruction_initialize_address
    );

    input rst;
    input clk;
    input initialize;
    input [31:0] instruction_initialize_data;
    input [31:0] instruction_initialize_address;

    wire [31:0] PC_out;
    wire [31:0] instruction;
    wire [31:0] instruction_mem_out;
    assign instruction = (initialize) ? 32'hFFFF_FFFF : instruction_mem_out;
    InstrMem InstructionMemory (instruction_mem_out, instruction_initialize_data,
                                (initialize) ? instruction_initialize_address : PC_out, initialize, clk);

    wire [3:0] ALUOp;
    wire MemRead;
    wire MemtoReg;
    wire RegDst;
    wire Branch;
    wire BranchNE;
    wire ALUSrc;
    wire MemWrite;
    wire RegWrite;
    wire Jump;
    wire LUI;
    wire ZeroExtend;
    control Control(instruction[31:26], ALUOp, MemRead, MemtoReg, RegDst, Branch, BranchNE, ALUSrc, MemWrite, RegWrite, Jump, LUI, ZeroExtend);

    wire [31:0] write_data;
    wire [4:0] write_register;
    wire [31:0] read_data_1, read_data_2;
    wire [31:0] ALUOut, MemOut;
    mux #(5) Write_Reg_MUX(RegDst, instruction[20:16], instruction[15:11], write_register);
    nbit_register_file Register_File(write_data, read_data_1, read_data_2, instruction[25:21], instruction[20:16], write_register, RegWrite, clk);

    wire [31:0] sign_extend_res; // hold result of sign-extending immediate
    sign_extend Sign_Extend(instruction[15:0], sign_extend_res);

    wire [31:0] immediate;
    // drive immediate with either shift left by 16 bits if LUI control is set or 
    // zero extend if ZeroExtend control is set, else sign extend
    assign immediate = LUI ? {instruction[15:0], 16'b0} : 
                            ZeroExtend ? {16'b0, instruction[15:0]} : sign_extend_res;

    wire [31:0] ALU_input_2;
    wire zero_flag;
    wire [3:0] ALU_function;
    mux #(32) ALU_Input_2_Mux(ALUSrc, read_data_2, immediate, ALU_input_2);
    ALU_control ALU_Control(instruction[5:0], ALUOp, ALU_function);
    ALU ALU(read_data_1, ALU_input_2, ALU_function, ALUOut, zero_flag);

    // negate zero flag for BNE
    wire not_zero_flag;
    assign not_zero_flag = ~zero_flag;

    Memory Data_Memory(ALUOut, read_data_2, MemOut, MemRead, MemWrite, clk);

    mux #(32) ALU_Mem_Select_MUX (MemtoReg, ALUOut, MemOut, write_data);

    wire [31:0] PC_in;
    PC Program_Counter(PC_out, PC_in, clk, rst);

    wire [31:0] PC_plus_4;
    Adder #(32) PC_Increment_Adder (PC_out, 32'd4, PC_plus_4);

    wire [31:0] Branch_target_address;
    wire [31:0] immediate_x_4;
    shift_left_2 #(32) Shift_Left_Two (immediate, immediate_x_4);
    Adder #(32) Branch_Target_Adder (PC_plus_4, immediate_x_4, Branch_target_address);

    wire branch_sel;
    wire [31:0] branch_mux_out; // to hold intermediate result for new jump mux, replaced PC_in
    // behavioral logic for driving branch_sel for BEQ or BNE
    assign branch_sel = (Branch && zero_flag) || (BranchNE  && not_zero_flag);
    mux #(32) BranchMux (branch_sel, PC_plus_4, Branch_target_address, branch_mux_out);
    
    // shift left by 2 module, new jump mux using corresponding Jump control
    wire [27:0] jump_addr;
    shift_left_2 #(.size(28)) Jump_Shift_Left_Two ({2'b00, instruction[25:0]}, jump_addr);
    mux #(32) JumpMux (Jump, branch_mux_out, {PC_plus_4[31:28], jump_addr}, PC_in);
endmodule
`timescale 1ns / 1ps

// Testbench for parametrized ALU. Verification done using behavioral ALU provided in lab4.

module NBitALU_tb();
    parameter n = 32;
    reg [n-1:0] R2, R3;
    reg [2:0] ALUOp;
    wire [n-1:0] R1, verify;
    wire error_flag;

    // structural
    NBitALU #(.n(n)) ALU (
        .R2(R2),
        .R3(R3),
        .ALUOp(ALUOp),
        .R1(R1)
    );

    // behavioral
    BehavioralALU #(.word_size(n)) verify_ALU (
        .R2(R2),
        .R3(R3),
        .ALUOp(ALUOp),
        .R1(verify)
    );

    assign error_flag = (R1 != verify);

    // task for testing cases without writing repetitive test logic
    // displays error if R1 != verify (error_flag is set)
    task test_case;
        input [3:0] op;
        input [n-1:0] in1, in2;
        begin
            #5;
            ALUOp = op;
            R2 = in1;
            R3 = in2;
            if (error_flag)
                $display("ERROR @ R2 = %h, R3 = %h; R1 = %h, verify = %h", R2, R3, R1, verify);
        end
    endtask
    
    initial begin
        R2 = 0;
        R3 = 0;
 
        // MOV tests (ALUOp = 000)
        $display("\nMOV Tests:");
        test_case(3'b000, 32'hA5A5A5A5, 32'h5A5A5A5A); // alternating bit pattern
        test_case(3'b000, 32'hCD3E3C81, 32'h8B7E5E63); // random values
        test_case(3'b000, 32'hFF4F2E06, 32'h753F0F35); // random values
        test_case(3'b000, 32'h00000000, 32'hFFFFFFFF); // all ones
        test_case(3'b000, 32'hFFFFFFFF, 32'h00000000); // all zeros

        // NOT tests (ALUOp = 001)
        $display("\nNOT Tests:");
        test_case(3'b001, 32'hFFFF0000, 32'h00000000); // half ones, half zeros
        test_case(3'b001, 32'hDF3E1C3C, 32'h00000000); // random value
        test_case(3'b001, 32'h3627F9BF, 32'h00000000); // random value
        test_case(3'b001, 32'h5BD1F1A2, 32'h00000000); // random value
        test_case(3'b001, 32'h80000000, 32'h00000000); // largest negative number

        // ADD tests (ALUOp = 010)
        $display("\nADD Tests:");
        test_case(3'b010, 32'h7FFFFFFF, 32'h00000001); // positive overflow
        test_case(3'b010, 32'hC7937E13, 32'h2DE7A58D); // random large values
        test_case(3'b010, 32'h4684F5C8, 32'hF7B2F0E3); // positive + negative
        test_case(3'b010, 32'h2FC7E8C8, 32'h5BE7F0C1); // two positive numbers
        test_case(3'b010, 32'h80000000, 32'h80000000); // two largest negative numbers

        // SUB tests (ALUOp = 011)
        $display("\nSUB Tests:");
        test_case(3'b011, 32'h00000010, 32'h00000010); // same values
        test_case(3'b011, 32'hD84A3E81, 32'h4F2A3C81); // larger from smaller
        test_case(3'b011, 32'h389B0ABA, 32'h208C1BFE); // positive result
        test_case(3'b011, 32'hAF2A3C94, 32'hFC5A3D8D); // negative from negative
        test_case(3'b011, 32'h80000000, 32'h00000001); // largest negative minus one

        // OR tests (ALUOp = 100)
        $display("\nOR Tests:");
        test_case(3'b100, 32'hAAAA5555, 32'h5555AAAA); // alternating bits
        test_case(3'b100, 32'h41D7D0C1, 32'h82621A15); // random values
        test_case(3'b100, 32'h00000000, 32'hFFFFFFFF); // all zeros with all ones
        test_case(3'b100, 32'hF0F0F0F0, 32'h0F0F0F0F); // alternating nibbles
        test_case(3'b100, 32'h00000000, 32'h00000000); // all zeros with all zeros

        // AND tests (ALUOp = 101)
        $display("\nAND Tests:");
        test_case(3'b101, 32'hF0F0F0F0, 32'h0F0F0F0F); // alternating nibbles
        test_case(3'b101, 32'h39F0B8E5, 32'h8FE9C0E1); // random values
        test_case(3'b101, 32'h5097EABA, 32'h8FA7C0A5); // random values
        test_case(3'b101, 32'h2287E0BE, 32'hA1E6F0E1); // random values
        test_case(3'b101, 32'h6720D5F7, 32'h2EF7A4C5); // random values

        // SLT tests (ALUOp = 110) - 2's Complement representation
        $display("\nSLT Tests:");
        test_case(3'b110, 32'h00000010, 32'h00000005); // R2 > R3 (both positive)
        test_case(3'b110, 32'hDCBF8545, 32'h84A0AEBE); // R2 > R3 (both negative)
    
        test_case(3'b110, 32'h0F26DCBC, 32'h13B2DBCF); // R2 < R3 (both positive)
        test_case(3'b110, 32'h84A0AEBE, 32'hDCBF8545); // R2 < R3 (both negative)
        
        test_case(3'b110, 32'h00000010, 32'h00000010); // R2 = R3 (both positive)
        test_case(3'b110, 32'h84A0AEBE, 32'h84A0AEBE); // R2 = R3 (both negative)
        test_case(3'b110, 32'h00000000, 32'h00000000); // R2 = R3 (both zero)
        
        test_case(3'b110, 32'hA47E1E75, 32'h2CDE2A57); // R2 < R3 (negative and positive)
        test_case(3'b110, 32'h210B1E0C, 32'hD2251E82); // R2 > R3 (positive and negative)
        
        test_case(3'b110, 32'h00000000, 32'h210B1E0C); // R2 < R3 (zero and positive)
        test_case(3'b110, 32'h00000000, 32'hD2251E82); // R2 > R3 (zero and negative)
        
        test_case(3'b110, 32'h210B1E0C, 32'h00000000); // R2 > R3 (positive and zero)
        test_case(3'b110, 32'hD2251E82, 32'h00000000); // R2 < R3 (negative and zero)
        
        test_case(3'b110, 32'h80000000, 32'h7FFFFFFF); // largest negative < largest positive
        
        $finish;
    end
endmodule

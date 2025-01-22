`timescale 1ns / 1ns

module ALU(
    a,
    b,
    func,
    out,
    zero_flag
    );
    parameter size = 32;

    input [size-1:0] a;
    input [size-1:0] b;
    input [3:0] func;
    output reg [size-1:0] out;
    output reg zero_flag;
    
    always @(*) begin
        case (out)
            0: zero_flag = 1'b1;
            default: zero_flag = 1'b0;
        endcase
    end

    always @(*) begin
        if (func == 4'd0)       // signed add
            out = $signed(a) + $signed(b);
        else if (func == 4'd1)  // signed sub
            out = $signed(a) - $signed(b);
        else if (func == 4'd2)  // and
            out = a & b;
        else if (func == 4'd3)  // or
            out = a | b;
        else if (func == 4'd4)  // nor
            out = ~(a | b);
        else if (func == 4'd5)  // signed slt
            out = ($signed(a) < $signed(b)) ? 1'b1 : 1'b0;
        else if (func == 4'd6)  // lui pass-thru
            out = b;

        // EXTRA CREDIT OPERATIONS
        else if (func == 4'd7)                                  // addiu
            if ($signed(a) < 32'd0 && $signed(b) < 32'd0)       // scale up both a and b then add
                out = ($signed(a) + 32'h80_00_00_00) + ($signed(b) + 32'h80_00_00_00);
            else if ($signed(a) < 32'd0)                        // only scale a
                out = ($signed(a) + 32'h80_00_00_00) + b;
            else if ($signed(b) < 32'd0)                        // only scale b
                out = a + ($signed(b) + 32'h80_00_00_00);
            else                                                // no scaling required
                out = a + b;
        else if (func == 4'd8)                                  // sltiu
            if (a < 32'd0 && b < 32'd0)                         // scale up both a and b then compare
                out = (a + 32'h80_00_00_00) < (b + 32'h80_00_00_00) ? 1'b1 : 1'b0;
            else if (a < 32'd0)                                 // only scale a
                out = (a + 32'h80_00_00_00) < b ? 1'b1 : 1'b0;
            else if (b < 32'd0)                                 // only scale b
                out = a < (b + 32'h80_00_00_00) ? 1'b1 : 1'b0;
            else                                                // no scaling required
                out = a < b ? 1'b1 : 1'b0;
        else if (func == 4'd9)                                  // xori
            out = a ^ b;

        else
            out = 0;
   end
endmodule
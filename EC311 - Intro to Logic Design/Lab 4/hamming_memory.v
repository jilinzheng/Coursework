`timescale 1ns / 1ps

module hamming_memory(
        Address,
   	    MemRead, 
		ReadData,
    	MemWrite,
		WriteData,
		err
    );
    
input MemRead, MemWrite; 
input [8:0] Address; // 9 bit address, largest is 511
input [15:0]   WriteData;
output reg [15:0]  ReadData;
output reg err;     // set this error flag only for double-errors (which are uncorrectable)

// If you need extra registers, you can instantiate them here.
// 
// YOUR CODE HERE 
// 
reg [4:0] parityBits;
reg [20:0] firstTwentyOneBits; 
reg [21:0] fullDataWithParity;
reg [4:0] checkBits;
reg P, C;

// 512 entries in RAM. Instantiate memory.
localparam MEM_DEPTH = 1 << 9;
reg [21:0]     ram[0:MEM_DEPTH-1]; // 16 bits + 5 parity bits

// bit position:  1   2     3   4    5  6  7    8   9 10 11 12 13 14 15  16   17 ... 21
// value:         p_1 p_2 data p_4  |< data >| p_8 |<------ data ---->| p_16  |< data >|

/* Initialize memory. Do not modify. */
integer i;
initial begin
	for (i=0;i<MEM_DEPTH;i=i+1) begin
		ram[i] = 0;
	end
end

always@(MemRead or MemWrite or Address or WriteData) begin
	if (MemRead) begin
		//ReadData = ram[Address];
		// Currently, the above line just reads the data from the memory at that address.
		// Comment out that line and add your error-correcting code according to the instructions
		// You should be outputting to "ReadData" at the end.
		// 
        // YOUR CODE HERE 
        // 
		// Calculate the 5 check bits

		checkBits[0] = ram[Address][21] ^ ram[Address][19] ^ ram[Address][17] ^ ram[Address][15] ^ ram[Address][13] ^ ram[Address][11] ^    // c_1
		               ram[Address][9] ^ ram[Address][7] ^ ram[Address][5] ^ ram[Address][3] ^ ram[Address][1];
	    checkBits[1] = ram[Address][20] ^ ram[Address][19] ^ ram[Address][16] ^ ram[Address][15] ^ ram[Address][12] ^ ram[Address][11] ^    // c_2
		               ram[Address][8] ^ ram[Address][7] ^ ram[Address][4] ^ ram[Address][3];
        checkBits[2] = ram[Address][18] ^ ram[Address][17] ^ ram[Address][16] ^ ram[Address][15] ^ ram[Address][10] ^ ram[Address][9] ^     // c_4
		               ram[Address][8] ^ ram[Address][7] ^ ram[Address][2] ^ ram[Address][1];
		checkBits[3] = ram[Address][14] ^ ram[Address][13] ^ ram[Address][12] ^ ram[Address][11] ^ ram[Address][10] ^ ram[Address][9] ^     // c_8
		               ram[Address][8] ^ ram[Address][7];
		checkBits[4] = ram[Address][6] ^ ram[Address][5] ^ ram[Address][4] ^ ram[Address][3] ^ ram[Address][2] ^ ram[Address][1];           // c_16
		
		/*
		checkBits[0] = ram[Address][1] ^ ram[Address][3] ^ ram[Address][5] ^ ram[Address][7] ^ ram[Address][9] ^ ram[Address][11] ^    // c_1
		               ram[Address][13] ^ ram[Address][15] ^ ram[Address][17] ^ ram[Address][19] ^ ram[Address][21];
	    checkBits[1] = ram[Address][2] ^ ram[Address][3] ^ ram[Address][6] ^ ram[Address][7] ^ ram[Address][10] ^ ram[Address][11] ^    // c_2
		               ram[Address][14] ^ ram[Address][15] ^ ram[Address][18] ^ ram[Address][19];
        checkBits[2] = ram[Address][4] ^ ram[Address][5] ^ ram[Address][6] ^ ram[Address][7] ^ ram[Address][12] ^ ram[Address][13] ^     // c_4
		               ram[Address][14] ^ ram[Address][15] ^ ram[Address][20] ^ ram[Address][21];
		checkBits[3] = ram[Address][8] ^ ram[Address][9] ^ ram[Address][10] ^ ram[Address][11] ^ ram[Address][12] ^ ram[Address][13] ^     // c_8
		               ram[Address][14] ^ ram[Address][15];
		checkBits[4] = ram[Address][16] ^ ram[Address][17] ^ ram[Address][18] ^ ram[Address][19] ^ ram[Address][20] ^ ram[Address][21];           // c_16
		*/
        
		//$display ("checkBits = %d",checkBits);
		// Check for error using checkBits
		if (checkBits != 0 &&                                                                               // checkBits is nonzero AND
		    //(checkBits != 21 && checkBits != 20 && checkBits != 18 && checkBits != 14 && checkBits != 6)    // checkBits is not equal to a parityBit
		    (checkBits != 1 && checkBits != 2 && checkBits != 4 && checkBits != 8 && checkBits != 16) &&   // checkBits is not equal to a parityBit
		    checkBits < 22
		    ) begin                                                // There is an error
		    checkBits = 22 - checkBits;                            // My implementation gives the checkBits in "reverse" order, so this "corrects" it
		    //$display ("new checkBits = %d",checkBits);
		    ram[Address][checkBits] = ~ram[Address][checkBits];    // Flip the erroneous bit
		    //$display ("the read ram[Address] = %b", ram[Address]);
		    ReadData = {ram[Address][19], ram[Address][17:15], ram[Address][13:7], ram[Address][5:1]};
		end
		else begin    // No errors or error is at a parity bit
		    //$display ("the read ram[Address] = %b", ram[Address]);
		    ReadData = {ram[Address][19], ram[Address][17:15], ram[Address][13:7], ram[Address][5:1]};
		end
		
		/*
		if (|checkBits) ram[Address][checkBits-1] = ~ram[Address][checkBits-1];
		
		ReadData[0] = ram[Address][2];
		ReadData[3:1] = ram[Address][6:4];
		ReadData[10:4] = ram[Address][14:8];
		ReadData[15:11] = ram[Address][20:16];
		*/
		// Detect double-error
		P = ^ram[Address];
		C = |checkBits;
		if (C && ~P && checkBits > 21) begin
		    $display ("error at checkBits = %b, or %d", checkBits, checkBits);
		    err = 1;
		end
		else err = 0;
	end
	
	if (MemWrite) begin
		//ram[Address] = WriteData;
		// Currently, the above line just writes the data to the memory at that address.
		// Comment out that line and add your parity-bits code according to the instructions.
		// You should be setting all 22 bits of ram[Address].
		// 
        // YOUR CODE HERE 
        // 
        // Calculate the 5 parity bits
        // Potentially inverted everything
        
		parityBits[0] = WriteData[15] ^ WriteData[14] ^ WriteData[12] ^ WriteData[11] ^ WriteData[9] ^    // p_1
		                WriteData[7] ^ WriteData[5] ^ WriteData[4] ^ WriteData[2] ^ WriteData[0];
		parityBits[1] = WriteData[15] ^ WriteData[13] ^ WriteData[12] ^ WriteData[10] ^ WriteData[9] ^    // p_2
		                WriteData[6] ^ WriteData[5] ^ WriteData[3] ^ WriteData[2];
		parityBits[2] = WriteData[14] ^ WriteData[13] ^ WriteData[12] ^ WriteData[8] ^ WriteData[7] ^     // p_4
		                WriteData[6] ^ WriteData[5] ^ WriteData[1] ^ WriteData[0];
		parityBits[3] = WriteData[11] ^ WriteData[10] ^ WriteData[9] ^ WriteData[8] ^ WriteData[7] ^      // p_8
		                WriteData[6] ^ WriteData[5];
	    parityBits[4] = WriteData[4] ^ WriteData[3] ^ WriteData[2] ^ WriteData[1] ^ WriteData[0];         // p_16
	    
	    /*
	    parityBits[0] = WriteData[0] ^ WriteData[1] ^ WriteData[3] ^ WriteData[4] ^ WriteData[6] ^    // p_1
		                WriteData[8] ^ WriteData[10] ^ WriteData[11] ^ WriteData[13] ^ WriteData[15];
		parityBits[1] = WriteData[0] ^ WriteData[2] ^ WriteData[3] ^ WriteData[5] ^ WriteData[6] ^    // p_2
		                WriteData[9] ^ WriteData[10] ^ WriteData[12] ^ WriteData[13];
		parityBits[2] = WriteData[1] ^ WriteData[2] ^ WriteData[3] ^ WriteData[7] ^ WriteData[8] ^     // p_4
		                WriteData[9] ^ WriteData[10] ^ WriteData[14] ^ WriteData[15];
		parityBits[3] = WriteData[4] ^ WriteData[5] ^ WriteData[6] ^ WriteData[7] ^ WriteData[8] ^      // p_8
		                WriteData[9] ^ WriteData[10];
	    parityBits[4] = WriteData[11] ^ WriteData[12] ^ WriteData[13] ^ WriteData[14] ^ WriteData[15];         // p_16
	    */
	    //$display("the parity bits: %b",parityBits);
	    
	    // Populate the first 21 bits
	    firstTwentyOneBits = {parityBits[0], parityBits[1], WriteData[15], parityBits[2], WriteData[14:12],
	                          parityBits[3], WriteData[11:5], parityBits[4], WriteData[4:0]};
	    
	    // Concatenate the final bit using XOR reduction
	    fullDataWithParity = {firstTwentyOneBits, ^firstTwentyOneBits};
	    
	    // Finally, write
	    ram[Address] = fullDataWithParity;
	    //$display ("the written ram[Address] = %b", ram[Address]);
	    
	    /*
	    ram[Address][2] = WriteData[0];
	    ram[Address][6:4] = WriteData[3:1];
	    ram[Address][14:8] = WriteData[10:4];
	    ram[Address][20:16] = WriteData[15:11];
	    
	    ram[Address][0] = parityBits[0];
	    ram[Address][1] = parityBits[1];
	    ram[Address][3] = parityBits[2];
	    ram[Address][7] = parityBits[3];
	    ram[Address][15] = parityBits[4];
	    
	    ram[Address][21] = ^(ram[Address][20:0]);
	    */
	end
end

endmodule

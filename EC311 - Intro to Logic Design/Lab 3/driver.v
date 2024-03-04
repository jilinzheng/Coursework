// Source: https://github.com/klam20/FPGAProjects/tree/main/VGATextGeneration
// Note: This and all submodules are sourced from the github repo above.
//       However, I have modified code to satisfy my requirements.
module vga
	(
	    input wire clk, reset,
        input wire [6:0] A, B, opCode, finalResult,
        output wire hsync, vsync,
        output wire [11:0] rgb
	);
	
	// VGA
	wire video_on;          // Video status output from vga_sync to tell when to route out RGB signal to DAC
    wire [9:0] x,y;         // Pixel location
    
    // Instantiate vga_sync for the monitor sync and x,y pixel tracing
    vga_sync vga_sync_unit(.clk(clk), .reset(reset), .hsync(hsync), .vsync(vsync),
                            .video_on(video_on), .x(x), .y(y));
                        
    wire [6:0] ascii;       // Signal is concatenated with X coordinate to get a value for the ROM address                 
    wire [6:0] a[12:0];     // Each index of this array holds a 7-bit ASCII value
    wire d[12:0];           // Each index of this array holds a signal that says whether the i-th item in array a above should display
    wire displayContents;   // Control signal to determine whether a character should be displayed on the screen
        
    // Display opCode, A, B, and finalResult in hexadecimal
    textGeneration c0 (.clk(clk),.reset(reset),.asciiData(a[0]), .ascii_In(opCode),
    .x(x),.y(y), .displayContents(d[0]), .x_desired(10'd296), .y_desired(10'd240)); 
    textGeneration c1 (.clk(clk),.reset(reset),.asciiData(a[1]), .ascii_In(A),
    .x(x),.y(y), .displayContents(d[1]), .x_desired(10'd312), .y_desired(10'd240));
    textGeneration c2 (.clk(clk),.reset(reset),.asciiData(a[2]), .ascii_In(B), 
    .x(x),.y(y), .displayContents(d[2]), .x_desired(10'd328), .y_desired(10'd240));
    textGeneration c3 (.clk(clk),.reset(reset),.asciiData(a[3]), .ascii_In(finalResult),
    .x(x),.y(y), .displayContents(d[3]), .x_desired(10'd344), .y_desired(10'd240));
    
    // Display my name
    textGeneration name0 (.clk(clk),.reset(reset),.asciiData(a[4]), .ascii_In(7'h4A),
    .x(x),.y(y), .displayContents(d[4]), .x_desired(10'd296), .y_desired(10'd160));
    textGeneration name1 (.clk(clk),.reset(reset),.asciiData(a[5]), .ascii_In(7'h49),
    .x(x),.y(y), .displayContents(d[5]), .x_desired(10'd304), .y_desired(10'd160));
    textGeneration name2 (.clk(clk),.reset(reset),.asciiData(a[6]), .ascii_In(7'h4C),
    .x(x),.y(y), .displayContents(d[6]), .x_desired(10'd312), .y_desired(10'd160));
    textGeneration name3 (.clk(clk),.reset(reset),.asciiData(a[7]), .ascii_In(7'h49),
    .x(x),.y(y), .displayContents(d[7]), .x_desired(10'd320), .y_desired(10'd160));
    textGeneration name4 (.clk(clk),.reset(reset),.asciiData(a[8]), .ascii_In(7'h4E),
    .x(x),.y(y), .displayContents(d[8]), .x_desired(10'd328), .y_desired(10'd160));
    
    // Display the title, 'Lab 3'
    textGeneration title0 (.clk(clk),.reset(reset),.asciiData(a[9]), .ascii_In(7'h4C),
    .x(x),.y(y), .displayContents(d[9]), .x_desired(10'd320), .y_desired(10'd192));
    textGeneration title1 (.clk(clk),.reset(reset),.asciiData(a[10]), .ascii_In(7'h41),
    .x(x),.y(y), .displayContents(d[10]), .x_desired(10'd328), .y_desired(10'd192));
    textGeneration title2 (.clk(clk),.reset(reset),.asciiData(a[11]), .ascii_In(7'h42),
    .x(x),.y(y), .displayContents(d[11]), .x_desired(10'd336), .y_desired(10'd192));
    textGeneration title3 (.clk(clk),.reset(reset),.asciiData(a[12]), .ascii_In(7'h33),
    .x(x),.y(y), .displayContents(d[12]), .x_desired(10'd344), .y_desired(10'd192));
    
    // Decoder to trigger displayContents signal high or low depending on which ASCII char is reached
    assign displayContents = d[0] ? d[0] :
                             d[1] ? d[1] :
                             d[2] ? d[2] :
                             d[3] ? d[3] :
                             d[4] ? d[4] :
                             d[5] ? d[5] :
                             d[6] ? d[6] :
                             d[7] ? d[7] : 
                             d[8] ? d[8] : 
                             d[9] ? d[9] : 
                             d[10] ? d[10] : 
                             d[11] ? d[11] : 
                             d[12] ? d[12] : 
                             0;
	
	// Decoder to assign correct ASCII value depending on which displayContents signal is used                     
    assign ascii = d[0] ? a[0] :
                   d[1] ? a[1] :
                   d[2] ? a[2] :
                   d[3] ? a[3] : 
                   d[4] ? a[4] : 
                   d[5] ? a[5] : 
                   d[6] ? a[6] : 
                   d[7] ? a[7] : 
                   d[8] ? a[8] :
                   d[9] ? a[9] :
                   d[10] ? a[10] :
                   d[11] ? a[11] :
                   d[12] ? a[12] :
                   7'h30;
    
    // ASCII_ROM    
    // Handle the row of the rom
    wire [3:0] rom_row;
    // Handle the column of the rom data
    wire [2:0] rom_col;
    // Connections to ascii_rom
    wire [10:0] rom_addr;
    // Wire to connect to rom_data of ascii_rom
    wire [7:0] rom_data;
    // Bit to signal display of data
    wire rom_bit;
    
    ascii_rom rom(.clk(clk), .rom_addr(rom_addr), .data(rom_data));
         
    // Concatenate to get 11 bit rom_addr
    assign rom_row = y[3:0];
    assign rom_col = x[2:0];
    assign rom_addr = {ascii, rom_row};
    assign rom_bit = rom_data[~rom_col]; // need to negate since it initially displays mirrored
       
    // If video on then check
        //If rom_bit is on
            // If x and y are in the origin/end range
                // Set RGB to display whatever is in the ROM within the origin/end range
            // Else we are out of range so we should not modify anything, RGB set to blue
        // rom_bit is off display blue
    // Video_off display black
    assign rgb = video_on ? (rom_bit ? ((displayContents) ? 12'hFFF: 12'h8): 12'h8) : 12'b0; // blue background white text
    
endmodule
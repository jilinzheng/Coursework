Name: Jilin Zheng
UID: U49258796

CONTRIBUTION STATEMENT:
	I worked solely on this lab/project and thus all contribution is by myself (with exception to online sources cited below).
		SIGNATURE (digitally signed with initials): J.Z.

Demo Video: 
	https://drive.google.com/file/d/1TzKz1YhjRr3mk6eilARP8L0Q9s3ik5IP/view?usp=sharing
	
Github Repository Link:
	https://github.com/jilinnn/ec311-lab3

Files Included in this Submission:
	Verilog Files:
		-ascii_rom.v
		-debouncingPushButton.v
		-decoder.v
		-dff.v
		-driver.v
		-fullALU.v
		-fullSystem.v
		-hexASCIICode.v
		-slowClock.v
		-textGeneration.v
		-vga_sync.v
	
	Testbench Files:
		-debouncingPushButton_test.v
		-decoder_test.v
		-fullALU_test.v
		-fullSystem_test.v
		-slowClock_test.v
		-toHexASCIICode_test.v
	
	Note: The source of the VGA controller I used included a working video of the module (and submodules); thus I did not use testbenches to test it out and instead implemented it right into my program (though I did modify it from the original version).

Note on proposal modification:
	I originally wanted to modify my ALU, but while I worked on the lab, it made a lot more sense to modify my Decoder with my new input mechanism and other sequential logic elements. The modified proposal is as follows:
		IO: Change input mechanism to combination of pushbuttons and switches
		Design: Modify decoder; Add sequential logic elements
		Surprise: VGA display
	Just wanted to note the small change! Thank you.

List of sources used in program and demo video:
-Nexys A7 Top View: https://digilent.com/reference/_media/reference/programmable-logic/nexys-a7/nexys-a7-top-600.png
-Debouncing Pushbutton: https://www.youtube.com/watch?v=LO8ONR1TceI
-Debouncing Pushbutton Diagram: https://www.fpga4student.com/2017/04/simple-debouncing-verilog-code-for.html
-ASCII Table: https://www.asciitable.com/
-VGA Text Generation: https://github.com/klam20/FPGAProjects/tree/main/VGATextGeneration	
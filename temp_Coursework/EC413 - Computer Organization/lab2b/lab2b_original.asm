############################################################################
# 
#                       EC413
#
#    		Assembly Language Lab -- Programming with Loops.
#
############################################################################
#  DATA
############################################################################
        .data           # Data segment
Hello:  .asciiz " \n Hello World! \n"  # declare a zero terminated string
AnInt:	.word	17      # a word initialized to 17
space:	.asciiz	" "	    # declare a zero terminate string
AnotherInt: .word 23	# another word, this time initialized to 23
WordAvg:   .word 0		#use this variable for part 4
ValidInt:  .word 0		#
ValidInt2: .word 0		#
lf:     .byte	10, 0	# string with carriage return and line feed
InLenW:	.word   4       # initialize to number of words in input1 and input2
InLenB:	.word   16      # initialize to number of bytes in input1 and input2
        .align  4       # pad to next 16 byte boundary (address % 16 == 0)
Input1_TAG: .ascii "Input1 starts on next line"
		.align	4
Input1:	.word	0x01020304,0x05060708
	    .word	0x090A0B0C,0x0D0E0F10
        .align  4
Input2_TAG: .ascii "Input2 starts on next line"
        .align  4
Input2: .word   0x01221117,0x090b1d1f   # input
        .word   0x0e1c2a08,0x06040210
        .align  4
Copy_TAG: .ascii "Copy starts on next line"
        .align  4
Copy:  	.space  128		# space to copy input word by word
        .align  4
Input3_TAG: .ascii "Input3 starts on next line"
        .align  4
Input3:	.space	400		# space for data to be transposed
Transpose_TAG: .ascii "Transpose starts on next line"
        .align  4
Transpose: .space 400	# space for transposed data
 
############################################################################
#  CODE
############################################################################
        .text                   # code segment
#
# print out greeting.  
# Task 2:  change the message so that it prints out your name.
main:
        la	$a0,Hello		# address of string to print
        li	$v0,4			# system call code for print_str
        syscall				# print the string
#
# Print the integer value of AnInt
# Task 3: modify the code so that it prints out two integers
# separated by a space.
		lw	$a0,AnInt		# load I/O register with value of AnInt
		li	$v0,1			# system call code for print_int
		syscall				# print the integer
#
# Print the integer values of each byte in the array Input1
# Task 4a: modify the code so that it prints spaces between the integers
		la	$s0,Input1		# load pointer to array Input1
		lw	$s1,InLenB		# get length of array Input1 in bytes
task4:	
		ble	$s1,$zero,done4	# test if done
		lb	$a0,($s0)		# load next byte into I/O register
		li	$v0,1			# specify print integer
		syscall				# print the integer
		add	$s0,$s0,1		# point to next byte
		sub	$s1,$s1,1		# decrement index variable
		j	task4			# do it again
done4:
#
# Task 4b: copy the Task 4 code and paste here.  Modify the code to print
# the array backwards.

task4a:

done4a:
#
# Code for Task 5 -- copy the contents of Input2 to Copy
		la	$s0,Input2		# load pointer to array Input1
		la	$s1,Copy		# load pointer to array Copy
		lw	$s2,InLenB		# get length of array Input1 in bytes
task5:
		ble	$s2,$zero,done5	# test if done
		lb	$t0,($s0)		# get the next byte
		sb	$t0,($s1)		# put the byte
		add	$s0,$s0,1		# increment input pointer
		add	$s1,$s1,1		# increment output point
		sub	$s2,$s2,1		# decrement index variable
		j	task5			# continue
done5:
#
# Task 5:  copy the Task 5 code and paste here.  Modify the code to copy
# the data in words rather than bytes.

task5b:

done5b:
#
# Code for Task 6 -- 
# Print the integer average of the contents of array Input2 (bytes)

task6:

done6:
#
# Code for Task 7 --  
# Print the first 25 integers that are divisible by 7 (with spaces)

task7:

done7:
#
# The following code initializes Input3 for Task9
		la	$s0,Input3		# load pointer to Input3
		li	$s1,100			# load size of array in bytes
		li	$t0,3			# start with 3
init8a:
		ble	$s1,$zero,done8a	# test if done
		sb	$t0,($s0)		# write out another byte
		add	$s0,$s0,1		# point to next byte
		sub	$s1,$s1,1		# decrement index variable
		add	$t0,$t0,1		# increase value by 1
		j 	init8a			# continue
done8a:
#
# Code for Task 9 --
# Transpose the 10x10 byte array in Input3 into Transpose


# ALL DONE!
Exit:
jr $ra



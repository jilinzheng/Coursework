############################################################################
# 
#                       EC413
#
#    Assembly Language Lab -- Programming with Loops.
#
############################################################################
#	DATA
############################################################################
		.data			# Data segment
Hello:  .asciiz " \n Hello Jilin! \n"  # declare a zero terminated string
AnInt:	.word	17		# a word initialized to 17
space:	.asciiz	" "		# declare a zero terminate string
AnotherInt: .word 23	# another word, this time initialized to 23
WordAvg:   .word 0		# use this variable for part 4
ValidInt:  .word 0		#
ValidInt2: .word 0		#
lf:     .byte	10, 0	# string with carriage return and line feed
InLenW:	.word   4		# initialize to number of words in input1 and input2
InLenB:	.word   16		# initialize to number of bytes in input1 and input2
		.align  4		# pad to next 16 byte boundary (address % 16 == 0)
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
newline: .asciiz "\n"	# newline
Quotient: .word 0		# for tasks 6 and 7
Rows:		.word 10	# for task 8, specify rows in matrix
Columns:	.word 10	# for task 8, specify columns in matrix

############################################################################
#	CODE
############################################################################
		.text			# code segment

# print out greeting.
# Task 2: change the message so that it prints out your name.
main:
		la	$a0,Hello		# address of string to print
		li	$v0,4			# system call code for print_str
		syscall				# print the string


# Print the integer value of AnInt
# Task 3: modify the code so that it prints out two integers
# separated by a space.
		lw	$a0,AnInt		# load I/O register with value of AnInt
		li	$v0,1			# system call code for print_int
		syscall				# print the integer
		la	$a0,space		# load space
		li	$v0,4			# load print_str syscall
		syscall				# print space
		lw	$a0,AnotherInt	# load second int
		li	$v0,1			# load print_int syscall
		syscall				# print int
		la	$a0,newline		# newline for readability
		li	$v0,4			# load print_str syscall
		syscall				# print newline


# Print the integer values of each byte in the array Input1
# Task 4a: modify the code so that it prints spaces between the integers
		la	$s0,Input1			# load pointer to array Input1
		lw	$s1,InLenB			# get length of array Input1 in bytes
task4a:	
		ble	$s1,$zero,done4a	# test if done
		lb	$a0,($s0)			# load next byte into I/O register
		li	$v0,1				# specify print integer
		syscall					# print the integer
		la	$a0,space			# load space
		li	$v0,4				# load print_str syscall
		syscall					# print space
		add	$s0,$s0,1			# point to next byte
		sub	$s1,$s1,1			# decrement index variable
		j	task4a				# do it again
done4a:
		la	$a0,newline			# newline for readability
		li	$v0,4				# load print_str syscall
		syscall					# print newline

# Task 4b: copy the Task 4 code and paste here. Modify the code to print
# the array backwards.
		lw	$s1,InLenB			# get length of array in bytes again
		sub	$s0,$s0,1			# point back to previous byte (end of array)
task4b:
		ble	$s1,$zero,done4b	# test if done
		lb	$a0,($s0)			# load next byte into I/O register
		li	$v0,1				# specify print integer
		syscall					# print the integer
		la	$a0,space			# load space
		li	$v0,4				# load print_str syscall
		syscall					# print space
		sub	$s0,$s0,1			# point to previous byte
		sub	$s1,$s1,1			# decrement index variable
		j	task4b				# do it again
done4b:
		la	$a0,newline			# newline for readability
		li	$v0,4				# load print_str syscall
		syscall					# print newline


# Code for Task 5 -- copy the contents of Input2 to Copy
		la	$s0,Input2		# load pointer to array Input2
		la	$s1,Copy		# load pointer to array Copy
		lw	$s2,InLenB		# get length of array Input2 in bytes
task5:
		ble	$s2,$zero,done5	# test if done
		lb	$t0,($s0)		# get the next byte
		sb	$t0,($s1)		# put the byte
		add	$s0,$s0,1		# increment input pointer
		add	$s1,$s1,1		# increment output point
		sub	$s2,$s2,1		# decrement index variable
		j	task5			# continue
done5:

# Task 5: copy the Task 5 code and paste here.  Modify the code to copy
# the data in words rather than bytes.
		la	$s0,Input2		# load pointer to array Input2
		la	$s1,Copy		# load pointer to array Copy
		lw	$s2,InLenW		# get length of array Input1 in words
task5b:
		ble	$s2,$zero,done5b# test if done
		lw	$t0,($s0)		# get the next word
		sw	$t0,($s1)		# put the word
		add	$s0,$s0,4		# increment input pointer
		add	$s1,$s1,4		# increment output point
		sub	$s2,$s2,1		# decrement index variable
		j	task5b			# continue
done5b:


# Code for Task 6
# Print the integer average of the contents of array Input2 (bytes)
		la	$s0,Input2		# load pointer to array Input2
		lw	$s1,InLenB		# get length of array Input2 in bytes
		li 	$s2,0			# running sum to use during loop
task6:
		ble $s1,$zero,done6	# test if done
		lb	$t0,($s0)		# get the next byte
		add	$s2,$s2,$t0		# add to running Sum
		add	$s0,$s0,1		# increment input pointer
		sub	$s1,$s1,1		# decrement index variable
		j	task6			# continue
done6:
		lw	$s1,InLenB		# get length of array Input2 in bytes again
		div $s2,$s1			# divide running sum by InLenB
		mflo $t1			# get quotient from LO
		sw	$t1,Quotient	# store quotient/integer average
		lw	$a0,Quotient	# load quotient into I/O register
		li	$v0,1			# specify print integer
		syscall				# print the integer
		la	$a0,newline		# newline for readability
		li	$v0,4			# load print_str syscall
		syscall				# print newline


# Code for Task 7
# Print the first 25 integers that are divisible by 7 (with spaces)
task7:
		li	$s0,25			# track number of integers to print
		li	$s1,7			# dividing by 7
		li	$t0,0			# number to check for divisibility
task7_loop:
		beqz $s0,done7		# branch if equal to zero
		add $t0,$t0,1		# increment $t1 by 1
		div $t0,$s1			# $t0/$s1
		mfhi $t1			# get remainder
		bnez $t1,task7_loop	# repeat increment if there exists a remainder (!= 0)
task7_print:
		sw	$t0,Quotient	# store the quotient (number divisible by 7)
		lw	$a0,Quotient	# load quotient into I/O register
		li	$v0,1			# specify print integer
		syscall				# print the integer
		la	$a0,space		# load space
		li	$v0,4			# load print_str syscall
		syscall				# print space
		sub	$s0,$s0,1		# decrement the numbers of integers to print
		bnez $s0,task7_loop
done7:
		la	$a0,newline		# newline for readability
		li	$v0,4			# load print_str syscall
		syscall				# print newline


# The following code initializes Input3 for Task8
		la	$s0,Input3		# load pointer to Input3
		lw	$t0,Rows		# load number of rows in matrix
		lw	$t1,Columns		# load number of columns in matrix
		mul $s1,$t0,$t1		# calculate number of bytes in Rows*Columns matrix
		li	$t0,3			# start with 3
init8:
		ble	$s1,$zero,task8	# test if done
		sb	$t0,($s0)		# write out another byte
		add	$s0,$s0,1		# point to next byte
		sub	$s1,$s1,1		# decrement index variable
		add	$t0,$t0,1		# increase value by 1
		j 	init8			# continue
# Code for Task 8
# Transpose the 10x10 byte array in Input3 into Transpose
task8:
		la	$s0,Input3			# load pointer to Input3 again
		la	$s1,Transpose		# load pointer to Transpose
		lw	$s2,Rows			# load from data for arithmetic operations to index
		lw	$s3,Columns			# load from data for arithmetic operations to index
		li	$t0,0				# outer loop index starting at 0, i == current row
		add	$t1,$t0,0			# inner loop index starting at $t0+1, j == current column
								# only need to swap things above/below diagonal
task8_innerloop:				# traverse columns
		beq	$t1,$s3,task8_outerloop	# branch if looped through one row

		mul	$t2,$t0,$s3			# calculate 'left' index to swap
		add $t2,$t2,$t1			# left = i * Columns + j
		add $t2,$t2,$s0			# add left index onto Input3 base pointer

		mul	$t3,$t1,$s2			# calculate 'right' index to swap
		add	$t3,$t3,$t0			# right = j * Rows + i
		add	$t3,$t3,$s0			# add right index onto Input3 base pointer

		lb	$t4,($t2)			# load first 'index' to swap
		lb 	$t5,($t3)			# load second 'index' to swap

		sub	$t2,$t2,$s0			# subtract by Input3 base pointer
		add	$t2,$t2,$s1			# add back in Transpose base pointer

		sub	$t3,$t3,$s0			# subtract by Input3 base pointer
		add	$t3,$t3,$s1			# add back in Transpose base pointer

		sb	$t4,($t3)			# store swapped value
		sb	$t5,($t2)			# store swapped value

		add	$t1,$t1,1			# increment inner loop/column index

		j	task8_innerloop		# unconditionally jump to inner loop for same row
task8_outerloop:				# traverse rows
		add	$t0,$t0,1			# increment outer loop/row index
		add	$t1,$t0,0			# inner loop index starting at $t0+1, j == current column

		bne	$t0,$s2,task8_innerloop	# branch if not all rows have been transposed

# Print task 8 for lab demo + verification of transpose
print_task8:
		li	$t0,0				# outer loop index starting at 0, i == current row
		li	$t1,0				# inner loop index starting at 0, j == current column
print_innerloop:				# traverse columns
		beq	$t1,$s3,print_outerloop	# branch if looped through one row

		mul	$t2,$t0,$s3			# multiply row index with Columns
		add	$t2,$t2,$t1			# add column index
		add $t2,$t2,$s1			# add Transpose base pointer

		lb	$a0,($t2)			# load content
		li	$v0,1				# load print_int syscall
		syscall					# print character

		la	$a0,space			# load space
		li	$v0,4				# load print_str syscall
		syscall					# print space

		add	$t1,$t1,1			# increment inner loop/column index

		j	print_innerloop		# unconditionally jump to inner loop for same row (traversing column indices)
print_outerloop:				# traverse rows
		la	$a0,newline			# newline for readability
		li	$v0,4				# load print_str syscall
		syscall					# print newline
		
		add	$t0,$t0,1			# increment outer loop/row index
		li	$t1,0				# reset column index to 0

		bne	$t0,$s2,print_innerloop	# branch if not all rows have been transposed


# ALL DONE!
Exit:
jr $ra

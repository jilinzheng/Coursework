# Jilin Zheng | EC413 Lab 4 Report

## 0. Note

This document is best read in a Markdown viewer, but feel free to continue at your discretion.

## 1. Module and Testbench Description

My .v files are split into [modules](./modules/) and [testbenches](./testbenches/). Although I have included the provided modules/testbenches for completeness, I will describe only the new files I have created, skipping past the provided files. They are as follows:

Modules:

- [NBitTop.v](./modules/NBitTop.v): Parametrized n-bit top module, connecting NBitALU and NBitReg.
- [NBitALU.v](./modules/NBitALU.v): Parameterized n-bit ALU, using parameterized n-bit function modules.
- [NBitMov.v](./modules/NBitMov.v): Parameterized n-bit MOV module, using 1-bit OR bit slices.
- [NBitNot.v](./modules/NBitNot.v): Parameterized n-bit NOT module, using 1-bit NOT bit slices.
- [NBitAdder.v](./modules/NBitAdder.v): Parameterized n-bit ADD module, using 1-bit ADD bit slices (1-bit Full Adders).
- [NBitSub.v](./modules/NBitSub.v): Parameterized n-bit SUB module, using 1-bit SUB bit slices (1-bit Full Adders; NBitALU hard-wires c_in to 0), implemented using 2's Complement addition.
- [NBitOr.v](./modules/NBitOr.v): Parameterized n-bit OR module, using 1-bit OR bit slices.
- [NBitAnd.v](./modules/NBitAnd.v): Parameterized n-bit AND module, using 1-bit AND bit slices.
- [NBitSLT.v](./modules/NBitSLT.v): Parameterized n-bit SLT module, using 1-bit SLT bit slices.
- [NBitReg.v](./modules/NBitReg.v): Parameterized n-bit register module, using 1-bit register (D Flip FLop) bit slices.
- [Xor.v](./modules/Xor.v): Helper XOR module (implemented using standard primitives) for SLT function.

Testbenches:

- [NBitTop_tb.v](./testbenches/NBitTop_tb.v): Testbench for parameterized n-bit top module.
- [NBitALU_tb.v](./testbenches/NBitALU_tb.v): Testbench for parametrized n-bit ALU.
- [NBitSLT_tb.v](./testbenches/NBitSLT_tb.v): Testbench for parameterized n-bit SLT.
- [NBitReg_tb.v](./testbenches/NBitReg_tb.v): Testbench for parameterized n-bit register.

## 2. Hierarchy of ALU and Register Design

- NBitTop instantiates NBitALU and NBitReg (register) and connects them with a wire
  - NBitALU instantiates NBitMov, NBitNot, NBitAdder, NBitSub, NBitOr, NBitAnd, and NBitSLT, and uses a MUX to select the appropriate output based on an ALUOp
    - NBitMov generates n ors, with one or input hard-wired to 0s
    - NBitNot generates n nots
    - NBitAdder generates n FAs
    - NBitSub generates n nots to invert the second input and n FAs to perform 2's complement addition (NBitALU hardwires 1 to c_in)
    - NBitOr generates n ors
    - NBitAnd generates n ands
    - NBitSLT generates n nots and n FAs just as NBitSub to compute the subtraction result of the inputs,, 1 xor for detecting overflow, and uses a MUX to select the appropriate bit (0 or 1) to output
      - Xor instantiates 2 nots, 2 ands, and 1 or for a structural implementation of xor
  - NBitReg generates n dffs

## 3. Simulation Waveforms

See the [waveforms](./waveforms/) directory.

Please note that for the `NBitTop_tb*.png`s, the error_flag(s) is set for 1 time unit (with the 2 time unit clock cycles I had used); this is intended, as being a sequential element, the new, appropriate value of the register is only set on the positive edge of the clock. `R0` will be delayed compared to `verify`. Please look through all five images and see that `R0` reflects `verify` (and of course, the actual ALUOp with inputs R2 and R3) after 1 time unit.

Also note that for the `NBitALU_tb*.png`s, there are two wires/outputs `c_out` and `c_out2`, which are the carry outs of the NBitAdder and NBitSub modules, respectively.

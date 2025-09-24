RISC-V 5-Stage Pipelined Processor
Overview

This project is a 32-bit RISC-V processor designed and implemented from the ground up using SystemVerilog. The processor supports the RV32I instruction set architecture and features a fully pipelined datapath with 5 stages:

Instruction Fetch (IF)

Instruction Decode (ID)

Execute (EX)

Memory (MEM)

Write Back (WB)

The design was synthesized and tested using Vivado, then simulated and deployed on an FPGA platform.

Features

Implements the RV32I ISA

5-stage pipeline with hazard detection and forwarding logic

Branch handling with pipeline flushes

Instruction and data memory modules

Testbench support for simulation and debugging

Synthesized for FPGA deployment

Technologies

SystemVerilog – core design and testbenches

Vivado – synthesis and simulation

FPGA (tested on Basys3 board)

RARS – for program assembly/testing

Repository Structure
├── src/               # SystemVerilog source files  
│   ├── alu.sv  
│   ├── control.sv  
│   ├── datapath.sv  
│   ├── hazard_unit.sv  
│   ├── top.sv  
│   └── ...  
├── test/              # Testbenches and sample programs  
├── docs/              # Design notes, block diagrams  
└── README.md          # Project overview

How to Run

Open project in Vivado.

Add source files (src/) and testbench files (test/).

Run behavioral simulation to verify pipeline functionality.

Synthesize the design and deploy to FPGA.

Example Programs

Arithmetic operations (add, sub, mul)

Memory access (lw, sw)

Branch/jump instructions (beq, jal)

Future Improvements

Implement support for additional RISC-V extensions (e.g., RV32M for multiplication/division).

Improve branch prediction to reduce pipeline stalls.

Expand testing with more complex benchmark programs.

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/05/2025 09:28:10 PM
// Design Name: 
// Module Name: HazardHandling
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module HazardForwardingUnit (
    input  logic        clk,
    input  logic        reset,

    // Decode stage
    input  logic [4:0]  IF_ID_rs1,
    input  logic [4:0]  IF_ID_rs2,

    // Execute stage
    input  logic [4:0]  ID_EX_rs1,
    input  logic [4:0]  ID_EX_rs2,
    input  logic [4:0]  ID_EX_rd,
    input  logic        ID_EX_RegWrite,
    input  logic        ID_EX_MemRead,

    // Memory stage
    input  logic [4:0]  EX_MEM_rd,
    input  logic        EX_MEM_RegWrite,

    // Write-back stage
    input  logic [4:0]  MEM_WB_rd,
    input  logic        MEM_WB_RegWrite,

    output logic [1:0]  forwardA,
    output logic [1:0]  forwardB,

    output logic        stall,
    output logic        PCWrite,
    output logic        IF_ID_Write
);

    // Forwarding logic (EX stage)
    always_comb begin
        // Default to no forwarding
        forwardA = 2'b00;
        forwardB = 2'b00;

        // EX hazard
        if (EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs1))
            forwardA = 2'b10;
        else if (MEM_WB_RegWrite && (MEM_WB_rd != 0) && (MEM_WB_rd == ID_EX_rs1))
            forwardA = 2'b01;

        if (EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs2))
            forwardB = 2'b10;
        else if (MEM_WB_RegWrite && (MEM_WB_rd != 0) && (MEM_WB_rd == ID_EX_rs2))
            forwardB = 2'b01;
    end

    // Load-use hazard detection (stalling)
    always_comb begin
        if (ID_EX_MemRead && ((ID_EX_rd == IF_ID_rs1) || (ID_EX_rd == IF_ID_rs2))) begin
            stall       = 1;
            PCWrite     = 0;
            IF_ID_Write = 0;
        end else begin
            stall       = 0;
            PCWrite     = 1;
            IF_ID_Write = 1;
        end
    end

endmodule

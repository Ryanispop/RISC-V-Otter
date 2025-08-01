`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Ryan Salute
// 
// Create Date: 02/06/2025 12:16:51 AM
// Design Name: 
// Module Name: BRANCH_ADDR_GEN
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Create the jal, branch, and jalr instructions that feed into the PC using the I, J, and B type instructions along with the PC and rs1. 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module BRANCH_ADDR_GEN(
input [31:0] I_type, J_type, B_type, rs1, PC,
output [31:0] jal, branch, jalr
    );
    
assign jal = PC + J_type;
assign branch = PC + B_type;
assign jalr = rs1 + I_type;
    
    
endmodule

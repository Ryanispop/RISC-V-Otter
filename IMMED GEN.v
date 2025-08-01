`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Ryan Salute
// 
// Create Date: 02/05/2025 11:26:35 PM
// Design Name: 
// Module Name: IMMED GEN
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Creates the U, I, S, J, and B type immediate instructions
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module IMMED_GEN(
input [31:7] ir,
output [31:0] U_type, I_type, S_type, J_type, B_type
    );

assign U_type = {ir[31:12],12'b0};
assign I_type = {{21{ir[31]}}, ir[30:25], ir[24:20]};
assign S_type = {{21{ir[31]}}, ir[30:25], ir[11:7]};
assign B_type = {{20{ir[31]}}, ir[7], ir[30:25], ir[11:8], 1'b0};
assign J_type = {{12{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0};

endmodule

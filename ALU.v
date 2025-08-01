`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Ryan Salute
// 
// Create Date: 01/28/2025 08:52:57 PM
// Design Name: 
// Module Name: ALU
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: performs all of the 11 RISC-V MCU required operations
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module ALU(
input [31:0] OP_1, OP_2,
input [3:0] ALU_FUN,
output reg [31:0] RESULT
    );
    
    always @ (*)
       begin 
          case (ALU_FUN) 
          4'b0000:      RESULT = OP_1 + OP_2;                                       //add
          4'b1000:      RESULT = OP_1 - OP_2;                                       //sub
          4'b0110:      RESULT = OP_1 | OP_2;                                       //or
          4'b0111:      RESULT = OP_1 & OP_2;                                       //and
          4'b0100:      RESULT = OP_1 ^ OP_2;                                       //xor
          4'b0101:      RESULT = OP_1 >> OP_2[4:0];                                 //srl
          4'b0001:      RESULT = OP_1 << OP_2[4:0];                                 //sll
          4'b1101:      RESULT = $signed(OP_1) >>> OP_2[4:0] ;                      //sra
          4'b0010:      RESULT = ($signed(OP_1) < $signed(OP_2)) ? 32'd1 : 32'd0;   //slt
          4'b0011:      RESULT = (OP_1 < OP_2) ? 32'd1 : 32'd0;                     //sltu
          4'b1001:      RESULT = OP_1;                                              //lui
          default: RESULT = 32'hdeadbeef;                                                  // default case
          endcase 
	   end
    
    
endmodule

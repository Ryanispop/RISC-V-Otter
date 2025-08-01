`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Ryan Salute
// 
// Create Date: 01/22/2025 07:12:22 PM
// Design Name: 
// Module Name: PC
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


module PC #(parameter n = 32)(
input CLK,
input PC_WE,
input rst,
input [1:0] PC_SEL,
input [n-1:0] jalr,
input [n-1:0] branch,
input [n-1:0] jal,
output reg [n-1:0] PC

    );

reg [n-1:0] data;



          
always @ (*)
       begin 
          case (PC_SEL) 
          0:      data = PC + 4;
          1:      data = jalr;
          2:      data = branch;
          3:      data = jal;
          
          default data = PC;
          endcase 
	   end
    
always @(posedge CLK)
    begin 
       if (rst == 1)       // synch clr
          PC <= 0;
       else if (PC_WE == 1)   // synch load
          PC <= data; 
    end

endmodule

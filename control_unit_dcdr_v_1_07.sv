`timescale 1ns / 1ps
///////////////////////////////////////////////////////////////////////////
// Company: Ratner Surf Designs
// Engineer: James Ratner
// 
// Create Date: 01/29/2019 04:56:13 PM
// Design Name: 
// Module Name: CU_DCDR
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: Decodes the instructins given and then controls the MCU according to the input.
// 
// Instantiation Template:
//
// CU_DCDR my_cu_dcdr(
//   .br_eq     (xxxx), 
//   .br_lt     (xxxx), 
//   .br_ltu    (xxxx),
//   .opcode    (xxxx),    
//   .func7     (xxxx),    
//   .func3     (xxxx),    
//   .ALU_FUN   (xxxx),
//   .PC_SEL    (xxxx),
//   .srcA_SEL  (xxxx),
//   .srcB_SEL  (xxxx), 
//   .RF_SEL    (xxxx)   );
//
// 
// Revision:
// Revision 1.00 - Created (02-01-2020) - from Paul, Joseph, & Celina
//          1.01 - (02-08-2020) - removed  else's; fixed assignments
//          1.02 - (02-25-2020) - made all assignments blocking
//          1.03 - (05-12-2020) - reduced func7 to one bit
//          1.04 - (05-31-2020) - removed misleading code
//          1.05 - (12-10-2020) - added comments
//          1.06 - (02-11-2021) - fixed formatting issues
//          1.07 - (12-26-2023) - changed signal names
//
// Additional Comments:
// 
///////////////////////////////////////////////////////////////////////////

module CU_DCDR(
   input [6:0] opcode,   //-  ir[6:0]
   input func7,          //-  ir[30]
   input [2:0] func3,    //-  ir[14:12] 
   output logic [3:0] ALU_FUN,
   output logic srcA_SEL,
   output logic [1:0] srcB_SEL, 
   output logic [1:0] RF_SEL,
   output logic REG_WRITE,
   output logic MEM_WRITE,
   output logic MEM_READ2,
   output logic [2:0] BR_TYPE
      );
    
   //- datatypes for RISC-V opcode types
   typedef enum logic [6:0] {
        LUI    = 7'b0110111,
        AUIPC  = 7'b0010111,
        JAL    = 7'b1101111,
        JALR   = 7'b1100111,
        BRANCH = 7'b1100011,
        LOAD   = 7'b0000011,
        STORE  = 7'b0100011,
        OP_IMM = 7'b0010011,
        OP_RG3 = 7'b0110011,
        sys   = 7'b1110011
   } opcode_t;
   opcode_t OPCODE; //- define variable of new opcode type
    
   assign OPCODE = opcode_t'(opcode); //- Cast input enum 
   
   
   typedef enum logic [2:0] {
        NO_BRANCH = 3'd0,
        BEQ       = 3'd1,
        BNE       = 3'd2,
        BLT       = 3'd3,
        BGE       = 3'd4,
        BLTU      = 3'd5,
        BGEU      = 3'd6
    } branch_t;

   //- datatype for func3Symbols tied to values
   typedef enum logic [2:0] {
        //BRANCH labels
        ADD_SUB = 3'b000,
        SLL = 3'b001,
        XOR = 3'b100,
        SRL_SRA = 3'b101,
        OR = 3'b110,
        AND = 3'b111,
        SLTU = 3'b011, 
        SLT  = 3'b010
   } func3_t;    
   func3_t FUNC3; //- define variable of new opcode type
   
   assign FUNC3 = func3_t'(func3);
   
   
       
   always_comb
   begin 
      //- schedule all values to avoid latch
      srcB_SEL = 2'b00;     RF_SEL = 2'b00; 
      srcA_SEL = 1'b0;   ALU_FUN  = 4'b0000;
      REG_WRITE = 1'b0;
      MEM_WRITE = 1'b0;
      MEM_READ2 = 1'b0;
      BR_TYPE = NO_BRANCH;

		
      case(OPCODE)
         LUI:
         begin
            srcB_SEL = 3'b000;
            ALU_FUN = 4'b1001; 
            srcA_SEL = 2'b01; 
            RF_SEL = 2'b11; 
            REG_WRITE = 1'b1;
         end
		 
		 AUIPC:
         begin
            srcA_SEL = 2'b01; 
            srcB_SEL = 3'b011; 
            RF_SEL = 2'b11;
            REG_WRITE = 1'b1;
         end
         
		 BRANCH: begin
            unique case (FUNC3)
            3'b000: BR_TYPE = BEQ;
            3'b001: BR_TYPE = BNE;
            3'b100: BR_TYPE = BLT;
            3'b101: BR_TYPE = BGE;
            3'b110: BR_TYPE = BLTU;
            3'b111: BR_TYPE = BGEU;
            default: BR_TYPE = NO_BRANCH;
        endcase
         end
		 	
         JAL:
         begin
            RF_SEL = 2'b00; 
            REG_WRITE = 1'b1;
			end
			
	     JALR:
         begin
            RF_SEL = 2'b00; 
            REG_WRITE = 1'b1;
		 end
			
         LOAD: 
         begin
            srcB_SEL = 3'b001; 
            RF_SEL = 2'b10; 
            MEM_READ2 = 1'b1;
         end
			
         STORE:
         begin
            srcB_SEL = 3'b010; 
            MEM_WRITE = 1'b1;
         end
			
         OP_IMM:
         begin
            srcB_SEL = 2'b01;
            RF_SEL = 2'b11;
            REG_WRITE = 1'b1;
            case(FUNC3)
               ADD_SUB: ALU_FUN = 4'b0000; 
               SLL:     ALU_FUN = 4'b0001;
               SLT:     ALU_FUN = 4'b0010;
               SLTU:    ALU_FUN = 4'b0011;
               XOR:     ALU_FUN = 4'b0100;
               SRL_SRA: ALU_FUN = (func7) ? 4'b1101 : 4'b0101;
               OR:      ALU_FUN = 4'b0110;
               AND:     ALU_FUN = 4'b0111;
            endcase
         end

         OP_RG3:
         begin
            srcB_SEL = 2'b00;
            RF_SEL = 2'b11;
            REG_WRITE = 1'b1;
            case (FUNC3)
               ADD_SUB: ALU_FUN = (func7) ? 4'b1000 : 4'b0000;
               SLL:     ALU_FUN = 4'b0001;
               SLT:     ALU_FUN = 4'b0010;
               SLTU:    ALU_FUN = 4'b0011;
               XOR:     ALU_FUN = 4'b0100;
               SRL_SRA: ALU_FUN = (func7) ? 4'b1101 : 4'b0101;
               OR:      ALU_FUN = 4'b0110;
               AND:     ALU_FUN = 4'b0111;
            endcase    
         end
         
         sys:
         begin
            RF_SEL = 3'b01;
            case(func3)
                3'b001: //csrrw
                begin
                    srcA_SEL = 2'b00;
                    ALU_FUN = 4'b1001;
                end
                
                3'b011: //csrrc
                begin
                    srcA_SEL = 2'b10;
                    srcB_SEL = 3'b100;
                    ALU_FUN = 4'b0111;
                    
                end
                
                3'b010: //csrrs
                begin
                    srcA_SEL = 2'b00;
                    srcB_SEL = 3'b100;
                    ALU_FUN = 4'b0110;
                    
                end
                
                3'b000:  //mret
                begin
                    REG_WRITE = 1'b1;
                end
            endcase
         end
         
         
         default: ;
      endcase
   end
   

endmodule
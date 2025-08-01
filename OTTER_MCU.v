`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/11/2025 11:52:58 PM
// Design Name: 
// Module Name: OTTER_MCU
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Top level RISC-V MCU with limited functions
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module OTTER_MCU(
input clk, RST, intr,
input [31:0] iobus_in,
output iobus_wr,
output [31:0] iobus_addr, iobus_out
    );
    
//PC wires
reg [31:0] data, PC;

//memory wires
wire [31:0] ir, DOUT2;

//REG file wires
wire [31:0] rs1, rs2;
reg [31:0] w_data;

//immed gen wires
wire [31:0] U_type, I_type, S_type, B_type, J_type;

//branch addr gen wires
wire [31:0] jalr, branch, jal;

//ALU wires
wire [31:0] result;
reg [31:0] ALU_srcA, ALU_srcB; 

//CU decoder wires
wire [2:0] srcB_SEL, PC_SEL;
wire [1:0] srcA_SEL, RF_SEL;
wire [3:0] ALU_FUN;

//CU FSM wires
wire PC_WE, RF_WE, memWE2, memRDEN1, memRDEN2, reset, intr_wire;

//Branch cond gen wires
wire br_eq, br_lt, br_ltu;

//CSR wires
wire [31:0] mepc, mtvec, csr_RD;
wire csr_WE, intr_taken, mret_exec, mstatus;

//PC sel
always @ (*)
       begin 
          case (PC_SEL) 
          0:      data = PC + 4;
          1:      data = jalr;
          2:      data = branch;
          3:      data = jal;
          4:      data = mtvec;
          5:      data = mepc;
          6:      data = 0;
          7:      data = 0;
          
          default data = PC;
          endcase 
	   end

//PC reg    
always @(posedge clk)
    begin 
       if (reset == 1)       // synch clr
          PC <= 0;
       else if (PC_WE == 1)   // synch load
          PC <= data; 
    end

Memory OTTER_MEMORY (
    .MEM_CLK   (clk),
    .MEM_RDEN1 (memRDEN1), 
    .MEM_RDEN2 (memRDEN2), 
    .MEM_WE2   (memWE2),
    .MEM_ADDR1 (PC[15:2]),
    .MEM_ADDR2 (result),
    .MEM_DIN2  (rs2),  
    .MEM_SIZE  (ir[13:12]),
    .MEM_SIGN  (ir[14]),
    .IO_IN     (iobus_in),
    .IO_WR     (iobus_wr),
    .MEM_DOUT1 (ir),
    .MEM_DOUT2 (DOUT2)  );

//RF mux
always @ (*)
       begin 
          case (RF_SEL) 
          0:      w_data = PC + 4;
          1:      w_data = csr_RD;
          2:      w_data = DOUT2;
          3:      w_data = result;
          
          default w_data = 0;
          endcase 
	   end

    
RegFile my_regfile (
    .w_data (w_data),
    .clk    (clk), 
    .en     (RF_WE),
    .adr1   (ir[19:15]),
    .adr2   (ir[24:20]),
    .w_adr  (ir[11:7]),
    .rs1    (rs1), 
    .rs2    (rs2)  );
assign iobus_out = rs2;   
 
    
IMMED_GEN MY_IG (
.ir(ir[31:7]),
.U_type(U_type), 
.I_type(I_type), 
.S_type(S_type), 
.J_type(J_type), 
.B_type(B_type)
);

BRANCH_ADDR_GEN MY_BAG(
.I_type(I_type), 
.J_type(J_type), 
.B_type(B_type), 
.rs1(rs1), 
.PC(PC),
.jal(jal), 
.branch(branch), 
.jalr(jalr)
);   

//ALU source a mux
always @ (*)
       begin 
          case (srcA_SEL) 
          0:      ALU_srcA = rs1;
          1:      ALU_srcA = U_type;
          2:      ALU_srcA = !rs1;
          3:      ALU_srcA = 0;
          
          default ALU_srcA = 0;
          endcase 
	   end
	   
//ALU source b mux
always @ (*)
       begin 
          case (srcB_SEL) 
          0:      ALU_srcB = rs2;
          1:      ALU_srcB = I_type;
          2:      ALU_srcB = S_type;
          3:      ALU_srcB = PC;
          4:      ALU_srcB = csr_RD;
          5:      ALU_srcB = 0;
          6:      ALU_srcB = 0;
          7:      ALU_srcB = 0;
          
          default ALU_srcB = 0;
          endcase 
	   end

 ALU my_alu(
.OP_1(ALU_srcA), 
.OP_2(ALU_srcB),
.ALU_FUN(ALU_FUN),
.RESULT(result)
    );
assign iobus_addr = result;

//Branch condition generator
assign br_eq = (rs1 == rs2);
assign br_lt = ($signed(rs1) < $signed(rs2));
assign br_ltu = (rs1 < rs2);


CU_DCDR my_cu_dcdr(
   .br_eq     (br_eq), 
   .br_lt     (br_lt), 
   .br_ltu    (br_ltu),
   .opcode    (ir[6:0]),    
   .func7     (ir[30]),    
   .func3     (ir[14:12]),
   .intr_taken(intr_taken),    
   .ALU_FUN   (ALU_FUN),
   .PC_SEL    (PC_SEL),
   .srcA_SEL  (srcA_SEL),
   .srcB_SEL  (srcB_SEL), 
   .RF_SEL    (RF_SEL)   );

assign intr_wire = intr && mstatus;

CU_FSM my_fsm(
        .intr     (intr_wire),
        .clk      (clk),
        .RST      (RST),
        .opcode   (ir[6:0]),   // ir[6:0]
        .func3    (ir[14:12]), // ir[14:12]
        .PC_WE    (PC_WE),
        .RF_WE    (RF_WE),
        .memWE2   (memWE2),
        .memRDEN1 (memRDEN1),
        .memRDEN2 (memRDEN2),
        .reset    (reset),
        .csr_WE   (csr_WE),
        .intr_taken(intr_taken),
        .mret_exec(mret_exec)       );


CSR  my_csr (
    .CLK        (clk),
    .RST        (reset),
    .MRET_EXEC  (mret_exec),
    .INT_TAKEN  (intr_taken),
    .ADDR       (ir[31:20]),
    .PC         (PC),
    .WD         (result),
    .WR_EN      (csr_WE),
    .RD         (csr_RD),
    .CSR_MEPC   (mepc),
    .CSR_MTVEC  (mtvec),
    .CSR_MSTATUS_MIE (mstatus)    );

endmodule

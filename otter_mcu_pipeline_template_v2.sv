`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: PIPELINED_OTTER_CPU
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

  typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
 } opcode_t;
        
typedef struct packed{
    opcode_t opcode;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic memWrite;
    logic memRead2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] pc;
    logic [2:0] br_type;
} instr_t;

typedef struct packed{
    logic [31:0] I_TYPE;
    logic [31:0] S_TYPE;
    logic [31:0] B_TYPE;
    logic [31:0] U_TYPE;
    logic [31:0] J_TYPE;
} types;

typedef enum logic [2:0] {
        NO_BRANCH = 3'd0,
        BEQ       = 3'd1,
        BNE       = 3'd2,
        BLT       = 3'd3,
        BGE       = 3'd4,
        BLTU      = 3'd5,
        BGEU      = 3'd6
} branch_t;

module OTTER_MCU(input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);           
    logic [6:0] opcode;
    logic [31:0] pc, pc_value, next_pc, jalr_pc, branch_pc, jump_pc, int_pc,A,B,
        I_immed,S_immed,U_immed,aluBin,aluAin,aluResult,rfIn,csr_reg, mem_data;
    
    logic [31:0] IR;
    logic memRead1,memRead2;
    
    logic pcWrite,regWrite,memWrite, alu_srcA,mem_op,IorD,pcWriteCond,memRead;
    logic [1:0] alu_srcB, rf_sel, wb_sel, mSize;
    logic [1:0] pc_sel;
    logic [3:0]alu_fun;
    logic opA_sel;
    logic flush;
    logic flush_id;
    logic br_lt,br_eq,br_ltu;
    
    
    logic [31:0] rs1, rs2;

    logic [31:0] dout2;
              
//==== Instruction Fetch ===========================================

     logic [31:0] if_de_pc;
     
     
     
//     assign pcWrite = 1'b1; 	//Hardwired high, assuming now hazards
//    assign memRead1 = 1'b1; 	//Fetch new instruction every cycle

    //PC sel
    always_comb begin
        case (pc_sel)
            2'b00: next_pc = pc + 4;    // PC + 4
            2'b01: next_pc = branch_pc;   // For branches
            2'b10: next_pc = jalr_pc;     // For jalr
            2'b11: next_pc = jump_pc;     // For jal
            default: next_pc = pc;
        endcase
    end

    always_ff @(posedge CLK or posedge RESET) begin
        if (RESET)
            pc <= 32'b0;
        else if (pcWrite)      // pcWrite is assumed to be always 1 for now
            pc <= next_pc;
    end
   
    logic ld_use_hz, cntrl_haz, hold_cntrl_haz

    always_ff @(posedge CLK) begin
        if (!ld_use_hz)
            if_de_pc <= pc;
        hold_cntrl_haz <= cntrl_haz;
    end

    assign pcWrite = !ld_use_hz;
    assign memRead1 = !ld_use_hz;
     
//==== Instruction Decode ===========================================
    logic [31:0] de_ex_opA;
    logic [31:0] de_ex_opB;
    logic [31:0] de_ex_rs2;

    //=== Immediate Generation ===//
    logic [31:0] I_type, S_type, U_type, B_type, J_type;
    IMMED_GEN immgen (
        .ir       (IR[31:7]),
        .U_type   (U_type),
        .I_type   (I_type),
        .S_type   (S_type),
        .J_type   (J_type),
        .B_type   (B_type)
    );
    
    types DECODE_TYPE, DE_EX_TYPE;
    assign DECODE_TYPE.I_TYPE = I_type;
    assign DECODE_TYPE.J_TYPE = J_type;
    assign DECODE_TYPE.B_TYPE = B_type;
    assign DECODE_TYPE.U_TYPE = U_type;
    assign DECODE_TYPE.S_TYPE = S_type; 
    
    CU_DCDR my_cu_dcdr(
   .opcode    (IR[6:0]),    
   .func7     (IR[30]),    
   .func3     (IR[14:12]),   
   .ALU_FUN   (alu_fun),
   .srcA_SEL  (alu_srcA),
   .srcB_SEL  (alu_srcB), 
   .RF_SEL    (rf_sel),
   .REG_WRITE (regWrite),
   .MEM_WRITE (memWrite),
   .MEM_READ2 (memRead),
   .BR_TYPE (de_inst.br_type)
   );
   
    instr_t de_ex_inst, de_inst;
    
    opcode_t OPCODE;
    assign OPCODE = opcode_t'(IR[6:0]);
    
    assign de_inst.rs1_addr=IR[19:15];
    assign de_inst.rs2_addr=IR[24:20];
    assign de_inst.rd_addr=IR[11:7];
    assign de_inst.opcode=OPCODE;
   
    assign de_inst.rs1_used=    de_inst.rs1_addr != 0
                                && de_inst.opcode != LUI
                                && de_inst.opcode != AUIPC
                                && de_inst.opcode != JAL;

    assign de_inst.rs2_used  = (OPCODE == OP || OPCODE == BRANCH || OPCODE == STORE);
    assign de_inst.rd_used   = (de_inst.rd_addr != 0) && (OPCODE != STORE) && (OPCODE != BRANCH);
    assign de_inst.alu_fun   = alu_fun;
    assign de_inst.memWrite  = memWrite;
    assign de_inst.memRead2  = memRead;
    assign de_inst.regWrite  = regWrite;
    assign de_inst.rf_wr_sel = rf_sel;
    assign de_inst.mem_type  = IR[14:12];
    assign de_inst.pc        = if_de_pc;
     
    //ALU source a mux
    always_comb begin
        unique case (alu_srcA)
            1'd0:   A = rs1;
            1'd1:   A = U_type;
            default: A = 32'd0;
        endcase
    end

    // ALU source B mux (SystemVerilog version)
    always_comb begin
        unique case (alu_srcB)
            2'd0:   B = rs2;
            2'd1:   B = I_type;
            2'd2:   B = S_type;
            2'd3:   B = if_de_pc;
            default: B = 32'd0;
        endcase
    end

    always_ff @(posedge CLK) begin
        if (cntrl_haz || hold_cntrl_haz || ld_use_hz) begin
            de_ex_inst <= '0;
            DE_EX_TYPE <= '0;
            de_ex_opA  <= 32'b0;
            de_ex_opB  <= 32'b0;
            de_ex_rs2  <= 32'b0;
        end else begin
            de_ex_inst <= de_inst;
            DE_EX_TYPE <= DECODE_TYPE;
            de_ex_opA  <= A;
            de_ex_opB  <= B;
            de_ex_rs2  <= rs2;
        end
    end


	//=== Register File ===//
    RegFile regfile (
        .w_data   (rfIn),
        .clk      (CLK),
        .en       (mem_wb_inst.regWrite),
        .adr1     (de_inst.rs1_addr),
        .adr2     (de_inst.rs2_addr),
        .w_adr    (mem_wb_inst.rd_addr),
        .rs1      (rs1),
        .rs2      (rs2)
    );
	
//==== Execute ======================================================
     logic [31:0] ex_mem_rs2;
     logic [31:0] ex_mem_aluRes;
     instr_t ex_mem_inst;
     types EX_MEM_TYPE;
     logic [31:0] opA_forwarded;
     logic [31:0] opB_forwarded;
     logic [1:0] forwardA, forwardB;


//---------HAZARD HANDLING, FORWARDING-------------//
    logic [1:0] fsel1, fsel2;
    logic [6:0] ex_load_op;
    assign ex_load_op = de_ex_inst.ir[6:0];
    
    HazardForwardingUnit HazardUnit(.opcode(ex_load_op),
    .de_adr1(de_inst.rs1_addr),
    .de_adr2(de_inst.rs2_addr),
    .ex_adr1(de_ex_inst.rs1_addr),
    .ex_adr2(de_ex_inst.rs2_addr),
    .ex_rd(de_ex_inst.rd_addr),
    .mem_rd(ex_mem_inst.rd_addr),
    .wb_rd(wb_t.rd_addr),
    .pc_source(pc_sel),
    .mem_regWrite(ex_mem_inst.regWrite),
    .wb_regWrite(wb_t.regWrite),
    .de_rs1_used(de_inst.rs1_used),
    .de_rs2_used(de_inst.rs2_used),
    .ex_rs1_used(de_ex_inst.rs1_used),
    .ex_rs2_used(de_ex_inst.rs2_used),
    .fsel1(fsel1),
    .fsel2(fsel2),
    .load_use_haz(ld_use_hz),
    .control_haz(cntrl_haz));

    always_comb begin
        unique case (fsel1)
            2'b00: aluAin = de_ex_opA;
            2'b10: aluAin = ex_mem_aluRes;
            2'b01: aluAin = rfIn;
            default: aluAin = 32'd0;
        endcase
    end

    always_comb begin
        unique case (fsel2)
            2'b00: aluBin = de_ex_opB;
            2'b10: aluBin = ex_mem_aluRes;
            2'b01: aluBin = rfIn;
            default: aluBin = 32'd0;
        endcase
    end

     //RISC-V ALU
    ALU my_alu(
    .OP_1(aluAin), 
    .OP_2(aluBin),
    .ALU_FUN(de_ex_inst.alu_fun),
    .RESULT(aluResult)
        );

    //Branch addr gen
    assign jump_pc = de_ex_inst.pc + DE_EX_TYPE.J_TYPE;
    assign jalr_pc = aluAin + DE_EX_TYPE.I_TYPE; 
    assign branch_pc = de_ex_inst.pc + DE_EX_TYPE.B_TYPE;
    
    BranchUnit BranchUnit(.IR(de_ex_inst.ir),
    .RS1(aluAin),
    .RS2(aluBin),
    .PC_SOURCE(pc_sel));

    

    
    always_ff@(posedge CLK) begin
        ex_mem_inst <= de_ex_inst;
        EX_MEM_TYPE <= DE_EX_TYPE;
        opA_forwarded <= de_ex_opA;
        opB_forwarded <= de_ex_opB; 
        ex_mem_rs2 <= de_ex_rs2;
        ex_mem_aluRes <= aluResult;
    end

    

//==== Memory ======================================================
     
    logic [31:0] wb_dout2, mem_wb_alu_result; 
    assign IOBUS_ADDR = ex_mem_aluRes;
    assign IOBUS_OUT = ex_mem_rs2;
    
    instr_t mem_wb_inst;
    types MEM_WB_TYPE;
     
     
    Memory OTTER_MEMORY (
    .MEM_CLK   (CLK),
    .MEM_RDEN1 (memRead1), 
    .MEM_RDEN2 (ex_mem_inst.memRead2), 
    .MEM_WE2   (ex_mem_inst.memWrite),
    .MEM_ADDR1 (pc[15:2]),
    .MEM_ADDR2 (ex_mem_aluRes),
    .MEM_DIN2  (ex_mem_rs2),  
    .MEM_SIZE  (ex_mem_inst.mem_type[1:0]),
    .MEM_SIGN  (ex_mem_inst.mem_type[2]),
    .IO_IN     (IOBUS_IN),
    .IO_WR     (IOBUS_WR),
    .MEM_DOUT1 (IR),
    .MEM_DOUT2 (dout2)
    );
    
    
    always_ff@(posedge CLK) begin
        mem_wb_inst <= ex_mem_inst;
        MEM_WB_TYPE <= EX_MEM_TYPE;
        wb_dout2 <= dout2;
        mem_wb_alu_result <= ex_mem_aluRes;
    end
    
     
//==== Write Back ==================================================
     

always_comb begin
        unique case (mem_wb_inst.rf_wr_sel)
            2'd0:   rfIn = mem_wb_inst.pc + 4;
            2'd1:   rfIn = csr_reg;
            2'd2:   rfIn = wb_dout2;
            2'd3:   rfIn = mem_wb_alu_result;
            default: rfIn = 32'd0;
        endcase
    end
 
 

       
            
endmodule

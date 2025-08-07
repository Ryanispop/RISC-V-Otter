`timescale 1ns / 1ps

module BranchUnit (
    input logic [31:0] IR,      // Instruction register
    input logic [31:0] RS1,     // Source register 1
    input logic [31:0] RS2,     // Source register 2
    output logic [1:0] PC_SOURCE // Control signal for PC update
);

    // Extracting opcode and funct3 for branch decision
    logic [6:0] opcode;
    logic [2:0] funct3;

    assign opcode = IR[6:0];
    assign funct3 = IR[14:12];

    // Branch comparison signals
    logic EQ, LT, LTU;

    // Calculate comparison results
    assign EQ = (RS1 == RS2);
    assign LT = ($signed(RS1) < $signed(RS2));
    assign LTU = (RS1 < RS2);

    always_comb begin
        PC_SOURCE = 2'b00; // Default: PC + 4

        if (opcode == 7'b1100011) begin  // BRANCH opcode
            case (funct3)
                3'b000: PC_SOURCE = EQ  ? 2'b01 : 2'b00; // BEQ
                3'b001: PC_SOURCE = !EQ ? 2'b01 : 2'b00; // BNE
                3'b100: PC_SOURCE = LT  ? 2'b01 : 2'b00; // BLT
                3'b101: PC_SOURCE = !LT ? 2'b01 : 2'b00; // BGE
                3'b110: PC_SOURCE = LTU ? 2'b01 : 2'b00; // BLTU
                3'b111: PC_SOURCE = !LTU ? 2'b01 : 2'b00; // BGEU
                default: PC_SOURCE = 2'b00; // Default no branch
            endcase
        end else if (opcode == 7'b1101111) begin
            PC_SOURCE = 2'b10; // JAL
        end else if (opcode == 7'b1100111) begin
            PC_SOURCE = 2'b11; // JALR
        end
    end

endmodule
`timescale 1ns / 1ps

// --- Main Decoder ---
module main_decoder (
    input  wire [6:0] opcode,
    // Add funct3/funct7 if needed for R-type ALUOp decoding
    // input wire [2:0] funct3,
    // input wire [6:0] funct7,
    output reg        RegWrite, MemRead, MemWrite, MemToReg, ALUSrc,
    output reg [2:0]  ALUOp
);
    localparam OP_RTYPE = 7'b0110011; localparam OP_ITYPE = 7'b0010011;
    localparam OP_LOAD  = 7'b0000011; localparam OP_STORE = 7'b0100011;
    // ALU Operation Codes (match alu module)
    localparam ALU_ADD  = 3'b000; localparam ALU_SUB  = 3'b001;
    // ... other ALU opcodes ...

    always @(*) begin
        // Default control signals (NOP-like)
        RegWrite=0; MemRead=0; MemWrite=0; MemToReg=0; ALUSrc=0; ALUOp=ALU_ADD;
        case (opcode)
            OP_RTYPE: begin
                RegWrite = 1'b1;
                // Set ALUOp based on funct3/funct7 for ADD, SUB, etc.
                ALUOp = ALU_ADD; // Placeholder - Add full R-type decoding
            end
            OP_ITYPE: begin // ADDI, SLTI, LW offset calc, etc.
                RegWrite = 1'b1; ALUSrc = 1'b1;
                // Set ALUOp based on funct3 for ADDI, SLTI, etc.
                ALUOp = ALU_ADD; // Placeholder
            end
            OP_LOAD:  begin // LW
                RegWrite = 1'b1; MemRead = 1'b1; MemToReg = 1'b1; ALUSrc = 1'b1;
                ALUOp = ALU_ADD; // For address calculation
            end
            OP_STORE: begin // SW
                MemWrite = 1'b1; ALUSrc = 1'b1;
                ALUOp = ALU_ADD; // For address calculation
            end
            default:  begin /* NOP */ end
        endcase
    end
endmodule

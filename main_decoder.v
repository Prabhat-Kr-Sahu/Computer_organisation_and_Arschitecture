`timescale 1ns / 1ps

// --- Main Decoder ---
// This module is now FULLY CORRECTED.
// It takes funct3 and funct7 to handle different R-type instructions.
module main_decoder (
    input  wire [6:0] opcode,
    input  wire [2:0] funct3, // <-- ADDED THIS INPUT
    input  wire [6:0] funct7, // <-- ADDED THIS INPUT
    output reg        RegWrite, MemRead, MemWrite, MemToReg, ALUSrc,
    output reg [2:0]  ALUOp
);
    // ALU Operation Codes (must match your alu.v)
    localparam ALU_ADD  = 3'b000;
    localparam ALU_SUB  = 3'b001;
    localparam ALU_SLL  = 3'b011;
    localparam ALU_SRL  = 3'b100;
    localparam ALU_XOR  = 3'b101;
    localparam ALU_OR   = 3'b110;
    localparam ALU_AND  = 3'b111;

    always @(*) begin
        // Default control signal values (NOP)
        RegWrite = 1'b0;
        MemRead  = 1'b0;
        MemWrite = 1'b0;
        MemToReg = 1'b0;
        ALUSrc   = 1'b0;
        ALUOp    = ALU_ADD; // Default to ADD (safe)

        case (opcode)
            // R-type (add, sub, etc.)
            7'b0110011: begin
                RegWrite = 1'b1;
                ALUSrc   = 1'b0; // Operand B from register file
                
                // --- THIS IS THE NEW R-TYPE LOGIC ---
                // Decode funct3/funct7 to select the correct ALUOp
                case (funct3)
                    3'b000: begin
                        if (funct7 == 7'b0000000)
                            ALUOp = ALU_ADD; // ADD
                        else if (funct7 == 7'b0100000)
                            ALUOp = ALU_SUB; // SUB
                        else
                            ALUOp = ALU_ADD; // Default (or error)
                    end
                    3'b001: ALUOp = ALU_SLL; // SLL
                    3'b100: ALUOp = ALU_SRL; // SRL
                    3'b101: ALUOp = ALU_XOR; // XOR
                    3'b110: ALUOp = ALU_OR;  // OR
                    3'b111: ALUOp = ALU_AND; // AND
                    default: ALUOp = ALU_ADD; // Default
                endcase
            end

            // I-type (addi, etc)
            7'b0010011: begin
                RegWrite = 1'b1;
                ALUSrc   = 1'b1; // Operand B from immediate
                // ALUOp = ALU_ADD; // (Already default)
            end

            // U-type (LUI)
            // --- THIS IS YOUR FIX ---
            7'b0110111: begin 
                RegWrite = 1'b1;
                ALUSrc   = 1'b1; // Operand B from immediate
                // ALUOp = ALU_ADD; // (Already default)
            end

            // Load (LW)
            7'b0000011: begin 
                RegWrite = 1'b1;
                MemRead  = 1'b1;
                MemToReg = 1'b1; // Data comes from Memory
                ALUSrc   = 1'b1; // For address calculation
                // ALUOp = ALU_ADD; // (Already default)
            end

            // Store (SW)
            7'b0100011: begin 
                MemWrite = 1'b1;
                ALUSrc   = 1'b1; // For address calculation
                // ALUOp = ALU_ADD; // (Already default)
            end

            // Branch (B-type)
            7'b1100011: begin 
                ALUSrc = 1'b0;   // Compare two registers
                ALUOp  = ALU_SUB; // Use SUB for comparison
            end

            // JAL
            7'b1101111: begin
                RegWrite = 1'b1; // JAL writes PC+4 to rd
                // ALUOp = ALU_ADD; // (Default, not really used for JAL)
            end
            
            // JALR
            7'b1100111: begin
                RegWrite = 1'b1; // JALR writes PC+4 to rd
                ALUSrc = 1'b1;   // ALU calculates (rs1 + imm)
                // ALUOp = ALU_ADD; // (Already default)
            end

            default: begin 
                // All other opcodes (FENCE, SYSTEM, AUIPC) are NOPs
                RegWrite = 1'b0;
                MemRead  = 1'b0;
                MemWrite = 1'b0;
                ALUSrc   = 1'b0;
                ALUOp    = ALU_ADD;
            end
        endcase
    end
endmodule
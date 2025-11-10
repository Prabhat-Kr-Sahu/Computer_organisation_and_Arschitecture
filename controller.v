`timescale 1ns / 1ps

//==================================================
// ## 3. MID-LEVEL MODULES (Controller) - FULL IMPLEMENTATION
// This module connects the decoders and hazard unit to generate
// all control signals for the datapath.
//==================================================

module controller (
    input  clk, rst,
    // Inputs from Datapath (ID Stage)
    input  wire [6:0] opcode,
    input  wire [2:0] funct3,
    input  wire [6:0] funct7,
    // Inputs from Datapath for Hazard Unit
    input  wire [4:0] ID_rs1, ID_rs2,
    input  wire [4:0] EX_rs1, EX_rs2, EX_rd,
    input  wire [4:0] MEM_rd, WB_rd,
    input  wire       EX_RegWrite, MEM_RegWrite, WB_RegWrite,
    input  wire       EX_MemRead, MEM_MemRead,
    input  wire       Zero_ex,
    input  wire       Negative_ex, // NEW: For signed branches

    // Outputs to Datapath
    output wire [1:0] ForwardA_Sel,
    output wire [1:0] ForwardB_Sel,
    output wire [1:0] PCSrc,
    output wire       Stall,
    output wire       Flush,
    output wire       ALUSrc, RegWrite, MemRead, MemWrite, MemToReg,
    output wire [2:0] ALUOp
);

    // --- Internal Wires ---
    wire branch_condition; // Result of branch logic (e.g., Zero_ex for BEQ)

    // --- Instantiate Sub-modules ---

    // 1. Main Decoder: Generates primary control signals from opcode
    main_decoder u_main_decoder (
        .opcode(opcode),
        .RegWrite(RegWrite),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .MemToReg(MemToReg),
        .ALUSrc(ALUSrc),
        .ALUOp(ALUOp)
    );

    // 2. Hazard Unit: Detects all data and control hazards
    hazard_unit u_hazard_unit (
        .ID_rs1(ID_rs1), .ID_rs2(ID_rs2),
        .EX_rs1(EX_rs1), .EX_rs2(EX_rs2), .EX_rd(EX_rd),
        .MEM_rd(MEM_rd), .WB_rd(WB_rd),
        .EX_RegWrite(EX_RegWrite), .MEM_RegWrite(MEM_RegWrite), .WB_RegWrite(WB_RegWrite),
        .EX_MemRead(EX_MemRead),
        .MEM_MemRead(MEM_MemRead),
        .ForwardA_Sel(ForwardA_Sel),
        .ForwardB_Sel(ForwardB_Sel),
        .Stall(Stall),
        .Flush(Flush) // Assuming hazard unit handles flush for now
    );

    // --- Control Logic for PC Source ---
    // This logic is now expanded to handle all standard branch types.
    wire branch_taken;
    reg  branch_cond_met;

    always @(*) begin
        case (funct3)
            3'b000: branch_cond_met = Zero_ex;      // BEQ: branch if Z=1
            3'b001: branch_cond_met = ~Zero_ex;     // BNE: branch if Z=0
            3'b100: branch_cond_met = Negative_ex;  // BLT: branch if N=1
            3'b101: branch_cond_met = ~Negative_ex; // BGE: branch if N=0
            // Note: BLTU/BGEU would require a Carry flag from the ALU
            default: branch_cond_met = 1'b0;
        endcase
    end

    assign branch_taken = (opcode == 7'b1100011) && branch_cond_met;

    // PCSrc Mux Control:
    // 2'b00: PC + 4
    // 2'b01: Branch Target
    assign PCSrc = branch_taken ? 2'b01 : 2'b00;

endmodule
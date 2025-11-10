`timescale 1ns / 1ps

module hazard_unit (
    // Inputs from various pipeline stages
    input  wire [4:0] ID_rs1, ID_rs2,
    input  wire [4:0] EX_rs1, EX_rs2, EX_rd,
    input  wire [4:0] MEM_rd, WB_rd,
    input  wire       EX_RegWrite, MEM_RegWrite, WB_RegWrite,
    input  wire       EX_MemRead,
    input  wire       MEM_MemRead, // NEW: To detect LW in MEM stage

    // Outputs to control the pipeline
    output reg  [1:0] ForwardA_Sel,
    output reg  [1:0] ForwardB_Sel,
    output reg        Stall,
    output reg        Flush // Assuming Flush is handled here
);

    always @(*) begin
        // --- Default values ---
        ForwardA_Sel = 2'b00;
        ForwardB_Sel = 2'b00;
        Stall = 1'b0;
        Flush = 1'b0; // Placeholder for branch/jump logic

        // --- Forwarding Logic ---
        // Priority: EX/MEM hazards are checked first.

        // Forwarding for rs1
        // Priority 1: Forward from MEM stage (result of EX stage)
        // **FIX**: Do NOT forward if the instruction in MEM is a load (MEM_MemRead is high).
        if (MEM_RegWrite && (MEM_rd != 5'b0) && (MEM_rd == EX_rs1) && !MEM_MemRead) begin
            ForwardA_Sel = 2'b01; // Forward from EX/MEM ALU result
        end
        // Priority 2: Forward from WB stage (result of MEM stage, could be from ALU or memory)
        else if (WB_RegWrite && (WB_rd != 5'b0) && (WB_rd == EX_rs1)) begin
            ForwardA_Sel = 2'b10; // Forward from MEM/WB result
        end

        // Forwarding for rs2
        if (MEM_RegWrite && (MEM_rd != 5'b0) && (MEM_rd == EX_rs2) && !MEM_MemRead) begin
            ForwardB_Sel = 2'b01;
        end
        else if (WB_RegWrite && (WB_rd != 5'b0) && (WB_rd == EX_rs2)) begin
            ForwardB_Sel = 2'b10;
        end

        // --- Stall Logic for Load-Use Hazard ---
        // If instruction in ID stage uses a register that an instruction
        // in EX stage is loading into, we must stall.
        if (EX_MemRead && (EX_rd != 5'b0) && ((EX_rd == ID_rs1) || (EX_rd == ID_rs2))) begin
            Stall = 1'b1;
        end
    end

endmodule

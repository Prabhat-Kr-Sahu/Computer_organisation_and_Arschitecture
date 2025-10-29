`timescale 1ns / 1ps

// --- Hazard Unit ---
module hazard_unit (
    // Inputs from ID Stage (for load-use stall detection)
    input  wire [4:0] ID_rs1_addr, ID_rs2_addr,
    // Inputs from ID/EX Register (destination & control for load-use)
    input  wire [4:0] EX_rd_addr,
    input  wire        EX_RegWrite,
    input  wire        EX_MemRead, // Control signal in EX stage
    // EX-stage source addresses (these are the register operands used in EX)
    input  wire [4:0] EX_rs1_addr, EX_rs2_addr,
    // Inputs from EX/MEM Register (older instruction ahead in pipeline)
    input  wire [4:0] MEM_rd_addr,
    input  wire        MEM_RegWrite,
    // Inputs from MEM/WB Register (older instruction ahead of that)
    input  wire [4:0] WB_rd_addr,
    input  wire        WB_RegWrite,
    // Outputs
    output reg  [1:0] ForwardA_Sel_out, ForwardB_Sel_out,
    output reg         Stall_out, Flush_out
);
    localparam FWD_ID  = 2'b00; localparam FWD_EXMEM = 2'b01; // From EX/MEM reg output
    localparam FWD_MEMWB = 2'b10; // From MEM/WB reg output

    // --- Stall Logic ---
    always @(*) begin
        // Check for Load-Use Hazard (Load in EX, Use in ID)
        if (EX_MemRead && EX_RegWrite && (EX_rd_addr != 5'b0) &&
           ((EX_rd_addr == ID_rs1_addr) || (EX_rd_addr == ID_rs2_addr)))
        begin
            Stall_out = 1'b1;
        end

        else begin
            Stall_out = 1'b0;
        end

        // Flush is for branches, not implemented here.
        Flush_out = 1'b0;
    end

    // --- Priority Forwarding Logic for Operand A (EX-stage rs1) ---
    always @(*) begin
        ForwardA_Sel_out = FWD_ID; // Default (no forwarding)

        // Priority 1: EX/MEM stage has the data -> forward to EX-stage operand
        if (MEM_RegWrite && (MEM_rd_addr != 5'b0) && (MEM_rd_addr == EX_rs1_addr)) begin
            ForwardA_Sel_out = FWD_EXMEM;
        end
        // Priority 2: MEM/WB stage has the data
        else if (WB_RegWrite && (WB_rd_addr != 5'b0) && (WB_rd_addr == EX_rs1_addr)) begin
            // Ensure EX/MEM isn't also providing the same register
            if (!(MEM_RegWrite && (MEM_rd_addr != 5'b0) && (MEM_rd_addr == EX_rs1_addr))) begin
                ForwardA_Sel_out = FWD_MEMWB;
            end
        end
        // Note: MUL forwarding can be handled via the same EX/MEM or MEM/WB paths
    end

    // --- Priority Forwarding Logic for Operand B (EX-stage rs2) ---
    always @(*) begin
        ForwardB_Sel_out = FWD_ID; // Default (no forwarding)

        // Priority 1: EX/MEM stage has the data
        if (MEM_RegWrite && (MEM_rd_addr != 5'b0) && (MEM_rd_addr == EX_rs2_addr)) begin
            ForwardB_Sel_out = FWD_EXMEM;
        end
        // Priority 2: MEM/WB stage has the data
        else if (WB_RegWrite && (WB_rd_addr != 5'b0) && (WB_rd_addr == EX_rs2_addr)) begin
            if (!(MEM_RegWrite && (MEM_rd_addr != 5'b0) && (MEM_rd_addr == EX_rs2_addr))) begin
                ForwardB_Sel_out = FWD_MEMWB;
            end
        end
    end
endmodule

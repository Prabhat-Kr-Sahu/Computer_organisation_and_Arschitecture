`timescale 1ns / 1ps

//==================================================
// ## 3. MID-LEVEL MODULES (Datapath and Controller)
//==================================================

// --- Controller ---
module controller (
    input clk, rst,
    // Status Inputs from Datapath pipeline registers
    input [6:0] opcode, funct7, // From IF/ID output
    input [2:0] funct3,         // From IF/ID output
    input [4:0] ID_rs1_addr, ID_rs2_addr, // From IF/ID output (instruction bits)
    input [4:0] EX_rd_addr,   // From ID/EX output
    input [4:0] MEM_rd_addr,  // From EX/MEM output
    input [4:0] WB_rd_addr,   // From MEM/WB output
    input       EX_RegWrite,  // From ID/EX output
    input       MEM_RegWrite, // From EX/MEM output
    input       WB_RegWrite,  // From MEM/WB output
    input       EX_MemRead,   // From ID/EX output (needed for hazard unit)
    // EX-stage source addresses (from ID/EX register) used for forwarding checks
    input [4:0] EX_rs1_addr, EX_rs2_addr,

    // Control Outputs to Datapath/Pipeline Registers
    output wire [1:0] ForwardA_Sel, ForwardB_Sel,
    output wire       Stall, Flush, // Stall to IF/ID reg + PC, Flush to IF/ID + ID/EX
    output wire       ALUSrc, RegWrite, MemRead, MemWrite, MemToReg, // To ID/EX reg
    output wire [2:0] ALUOp,       // To ID/EX reg
    output wire [1:0] PCSrc        // To pc_update (PC selection)
);
    // Internal signals from decoder
    wire [2:0] alu_op_main;
    wire       reg_write_main, mem_read_main, mem_write_main, mem_to_reg_main, alu_src_main;

    // Instantiate Main Decoder
    main_decoder u_decoder (
        .opcode(opcode),
        // Add funct3/funct7 if needed
        .RegWrite(reg_write_main), .MemRead(mem_read_main), .MemWrite(mem_write_main),
        .MemToReg(mem_to_reg_main), .ALUSrc(alu_src_main),
        .ALUOp(alu_op_main)
    );

    // Instantiate Hazard Unit
    hazard_unit u_hazard (
        // ID Stage Inputs (Source Regs for stall detection)
        .ID_rs1_addr(ID_rs1_addr), .ID_rs2_addr(ID_rs2_addr),
        // EX Stage Inputs (Dest Reg + Control for stall detection)
        .EX_rd_addr(EX_rd_addr), .EX_RegWrite(EX_RegWrite), .EX_MemRead(EX_MemRead),
        // EX-stage source addresses (for forwarding comparisons)
        .EX_rs1_addr(EX_rs1_addr), .EX_rs2_addr(EX_rs2_addr),
        // MEM Stage Inputs (Dest Reg + Control) -> used as EX/MEM in forwarding logic
        .MEM_rd_addr(MEM_rd_addr), .MEM_RegWrite(MEM_RegWrite),
        // WB Stage Inputs (Dest Reg + Control) -> used as MEM/WB in forwarding logic
        .WB_rd_addr(WB_rd_addr), .WB_RegWrite(WB_RegWrite),

        // Outputs
        .ForwardA_Sel_out(ForwardA_Sel), .ForwardB_Sel_out(ForwardB_Sel),
        .Stall_out(Stall), .Flush_out(Flush)
    );

    // Final Control Signals (apply stall) - These go to ID/EX Register Input
    assign RegWrite = reg_write_main & ~Stall;
    assign MemRead  = mem_read_main  & ~Stall;
    assign MemWrite = mem_write_main & ~Stall;
    assign MemToReg = mem_to_reg_main & ~Stall;
    assign ALUSrc   = alu_src_main   & ~Stall;
    assign ALUOp    = alu_op_main; // ALUOp doesn't get stalled out, just passed
    // Default PCSrc = PC+4. Full branch/jump control not implemented here yet.
    assign PCSrc = 2'b00;
    // NOTE: Stall signal also goes to PC and IF/ID enable logic in Datapath
    // NOTE: Flush signal also goes to IF/ID and ID/EX clear logic in Datapath

endmodule

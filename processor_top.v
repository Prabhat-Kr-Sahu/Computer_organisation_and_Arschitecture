`timescale 1ns / 1ps

//==================================================
// ## 4. TOP LEVEL MODULE
//==================================================

module processor_top (
    input clk,
    input rst
);
    // --- Wires connecting Controller and Datapath ---
    wire [1:0]  ForwardA_Sel, ForwardB_Sel;
    wire [1:0]  PCSrc;
    wire        ALUSrc, RegWrite, MemRead, MemWrite, MemToReg;
    wire [2:0]  ALUOp;
    wire        Stall, Flush;
    wire [6:0]  opcode, funct7;
    wire [2:0]  funct3;
    wire [4:0]  ID_rs1_addr, ID_rs2_addr, EX_rd_addr, MEM_rd_addr, WB_rd_addr;
    wire [4:0]  EX_rs1_addr, EX_rs2_addr; // EX-stage source addresses (for forwarding)
    wire        EX_RegWrite, MEM_RegWrite, WB_RegWrite, EX_MemRead;

    // --- Wires connecting Datapath and Memories ---
    wire [31:0] instr;
    wire [31:0] pc_out;
    wire [31:0] mem_addr;
    wire [31:0] mem_write_data;
    wire [31:0] mem_read_data;

    // --- Instantiate Controller ---
    controller u_controller (
        .clk(clk), .rst(rst),
        // Status Inputs (from Datapath pipeline registers via Datapath outputs)
        .opcode(opcode), .funct3(funct3), .funct7(funct7),
    .ID_rs1_addr(ID_rs1_addr), .ID_rs2_addr(ID_rs2_addr),
    .EX_rs1_addr(EX_rs1_addr), .EX_rs2_addr(EX_rs2_addr),
        .EX_rd_addr(EX_rd_addr), .MEM_rd_addr(MEM_rd_addr), .WB_rd_addr(WB_rd_addr),
        .EX_RegWrite(EX_RegWrite), .MEM_RegWrite(MEM_RegWrite), .WB_RegWrite(WB_RegWrite),
    .EX_MemRead(EX_MemRead), // Hazard unit needs MemRead from EX stage
    .PCSrc(PCSrc),
        // Control Outputs (to Datapath pipeline register inputs)
        .ForwardA_Sel(ForwardA_Sel), .ForwardB_Sel(ForwardB_Sel),
        .Stall(Stall), .Flush(Flush),
        .ALUSrc(ALUSrc), .RegWrite(RegWrite), .MemRead(MemRead), .MemWrite(MemWrite),
        .MemToReg(MemToReg),
        .ALUOp(ALUOp)
    );

    // --- Instantiate Datapath ---
    datapath u_datapath (
        .clk(clk), .rst(rst),
        // Control Inputs (from Controller, used to control MUXes/Units or go to pipeline regs)
        .ForwardA_Sel(ForwardA_Sel), .ForwardB_Sel(ForwardB_Sel), .PCSrc(PCSrc),
        .Stall(Stall), .Flush(Flush),
        // Pass control signals needed by ID/EX register
        .ALUSrc(ALUSrc), .RegWrite(RegWrite), .MemRead(MemRead), .MemWrite(MemWrite),
        .MemToReg(MemToReg), .ALUOp(ALUOp),

        // Status Outputs (to Controller)
        .opcode_out(opcode), .funct3_out(funct3), .funct7_out(funct7),
        .ID_rs1_addr_out(ID_rs1_addr), .ID_rs2_addr_out(ID_rs2_addr),
    .EX_rd_addr_out(EX_rd_addr), .EX_rs1_addr_out(EX_rs1_addr), .EX_rs2_addr_out(EX_rs2_addr),
    .MEM_rd_addr_out(MEM_rd_addr), .WB_rd_addr_out(WB_rd_addr),
        .EX_RegWrite_out(EX_RegWrite), .MEM_RegWrite_out(MEM_RegWrite), .WB_RegWrite_out(WB_RegWrite),
        .EX_MemRead_out(EX_MemRead),

        // Memory Interface (Connect to external memories)
        .instr_in(instr), .pc_out(pc_out),
        .mem_addr_out(mem_addr), .mem_write_data_out(mem_write_data),
        .mem_read_data_in(mem_read_data)
    );

    // --- Instantiate Memories ---
    instruction_memory u_imem (
        .addr(pc_out), // Connect PC output to memory address
        .data(instr)   // Connect memory data output to datapath instruction input
    );
    data_memory u_dmem (
        .clk(clk), .addr(mem_addr), .write_data(mem_write_data),
        .read_data(mem_read_data), .MemRead(MemRead), .MemWrite(MemWrite) // Control signals from Controller
    );

endmodule

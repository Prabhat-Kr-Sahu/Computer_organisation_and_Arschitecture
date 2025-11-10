//==================================================
// ## 4. TOP LEVEL MODULE
//==================================================

module processor_top (
    input clk,
    input rst
);
    // --- Control / status Wires between Controller and Datapath ---
    wire [1:0]  ForwardA_Sel, ForwardB_Sel; // ADDED BACK
    wire [1:0]  PCSrc;
    wire        ALUSrc, RegWrite, MemRead, MemWrite, MemToReg;
    wire [2:0]  ALUOp;
    wire        Stall, Flush;
    // NEW: Wires for pipelined memory control signals from datapath
    wire        MemWrite_mem;
    wire        MemRead_mem;
    wire        MEM_MemRead_signal; // NEW wire for lw hazard
    wire [6:0]  opcode, funct7;
    wire [2:0]  funct3;
    wire [4:0]  ID_rs1_addr, ID_rs2_addr, EX_rd_addr, MEM_rd_addr, WB_rd_addr;
    wire [4:0]  EX_rs1_addr, EX_rs2_addr; // EX-stage source addresses (for hazard unit)
    wire        EX_RegWrite, MEM_RegWrite, WB_RegWrite, EX_MemRead;
    wire        Zero_ex; // NEW wire for branch condition
    wire        Negative_ex; // NEW wire for signed branch condition

    // --- Wires connecting Datapath and Memories ---
    wire [31:0] instr;
    wire [31:0] pc_out;
    wire [31:0] mem_addr;
    wire [31:0] mem_write_data;
    wire [31:0] mem_read_data;

    // --- Optional debug wires (bubbled up from datapath) ---
    wire [1:0] dbg_forward_a_sel;
    wire [1:0] dbg_forward_b_sel;
    wire [31:0] dbg_operand_a_ex;
    wire [31:0] dbg_alu_result_ex;

    // --- Instantiate Controller ---
    controller u_controller (
        .clk(clk), .rst(rst),
        // Status Inputs
        .opcode(opcode), .funct3(funct3), .funct7(funct7),
        .ID_rs1(ID_rs1_addr), .ID_rs2(ID_rs2_addr),
        .EX_rs1(EX_rs1_addr), .EX_rs2(EX_rs2_addr), .EX_rd(EX_rd_addr),
        .MEM_rd(MEM_rd_addr), .WB_rd(WB_rd_addr),
        .EX_RegWrite(EX_RegWrite), .MEM_RegWrite(MEM_RegWrite), .WB_RegWrite(WB_RegWrite),
        .EX_MemRead(EX_MemRead),
        .MEM_MemRead(MEM_MemRead_signal),
        .Zero_ex(Zero_ex), // Connect Zero flag
        .Negative_ex(Negative_ex), // Connect Negative flag

        // Control Outputs
        .ForwardA_Sel(ForwardA_Sel), .ForwardB_Sel(ForwardB_Sel),
        .PCSrc(PCSrc),
        .Stall(Stall), .Flush(Flush),
        .ALUSrc(ALUSrc), .RegWrite(RegWrite), .MemRead(MemRead), .MemWrite(MemWrite),
        .MemToReg(MemToReg),
        .ALUOp(ALUOp)
    );

    // --- Instantiate Datapath ---
    datapath u_datapath (
        .clk(clk), .rst(rst),
        // Control Inputs (from Controller)
        .ForwardA_Sel(ForwardA_Sel), .ForwardB_Sel(ForwardB_Sel), // Connect from controller
        .PCSrc(PCSrc),
        .Stall(Stall), .Flush(Flush),
        .ALUSrc(ALUSrc), .RegWrite(RegWrite), .MemRead(MemRead), .MemWrite(MemWrite),
        .MemToReg(MemToReg), .ALUOp(ALUOp),

        // Status Outputs (to Controller)
        .opcode_out(opcode), .funct3_out(funct3), .funct7_out(funct7),
        .ID_rs1_addr_out(ID_rs1_addr), .ID_rs2_addr_out(ID_rs2_addr),
        .EX_rd_addr_out(EX_rd_addr), .EX_rs1_addr_out(EX_rs1_addr), .EX_rs2_addr_out(EX_rs2_addr),
        .MEM_rd_addr_out(MEM_rd_addr), .WB_rd_addr_out(WB_rd_addr),
        .EX_RegWrite_out(EX_RegWrite), .MEM_RegWrite_out(MEM_RegWrite), .WB_RegWrite_out(WB_RegWrite),
        .EX_MemRead_out(EX_MemRead),
        .Zero_ex(Zero_ex),
        .Negative_ex(Negative_ex),

        // NEW: Connect to the pipelined memory control outputs
        .MemWrite_mem_out(MemWrite_mem),
        .MemRead_mem_out(MemRead_mem),
        .MEM_MemRead_out(MEM_MemRead_signal), // Connect to controller

        // Memory Interface (Connect to external memories)
        .instr_in(instr), .pc_out(pc_out),
        .mem_addr_out(mem_addr), .mem_write_data_out(mem_write_data),
        .mem_read_data_in(mem_read_data),

        // Debug interface passed through (optional)
        .debug_reg_addr(5'd0), .debug_reg_data(), // adjust or connect to TB if needed
        .dbg_forward_a_sel(dbg_forward_a_sel),
        .dbg_forward_b_sel(dbg_forward_b_sel),
        .dbg_operand_a_ex(dbg_operand_a_ex),
        .dbg_alu_result_ex(dbg_alu_result_ex)
    );

    // --- Instantiate Memories ---
    instruction_memory u_imem (
        .addr(pc_out), // Connect PC output to memory address
        .data(instr)   // Connect memory data output to datapath instruction input
    );
    data_memory u_dmem (
        .clk(clk), .addr(mem_addr), .write_data(mem_write_data),
        .read_data(mem_read_data), 
        // FIX: Use the correctly pipelined signals from the MEM stage
        .MemRead(MemRead_mem), .MemWrite(MemWrite_mem) // Control signals from Controller
    );
//dawwadw
endmodule

 `timescale 1ns / 1ps
// datapath.v
// Pipelined datapath compatible with processor_top.v and controller.v
// - Uses forwarding_mux_a / forwarding_mux_b
// - Produces pipelined MemRead/MemWrite outputs for data memory
// - Exposes register addresses and control-status signals for the controller/hazard

module datapath (
    input  wire        clk,
    input  wire        rst,

    // Control Inputs from Controller (hazard unit provides forwarding selects)
    input  wire [1:0]  ForwardA_Sel,
    input  wire [1:0]  ForwardB_Sel,
    input  wire [1:0]  PCSrc,
    input  wire        Stall,
    input  wire        Flush,
    // Control signals for the current ID stage (these are control outputs from controller but
    // are applied to ID/EX via pipeline_register_id_ex)
    input  wire        ALUSrc,
    input  wire        RegWrite,
    input  wire        MemRead,
    input  wire        MemWrite,
    input  wire        MemToReg,
    input  wire [2:0]  ALUOp,

    // Status Outputs TO Controller/Hazard
    output wire [6:0]  opcode_out,
    output wire [2:0]  funct3_out,
    output wire [6:0]  funct7_out,
    output wire [4:0]  ID_rs1_addr_out,
    output wire [4:0]  ID_rs2_addr_out,
    output wire [4:0]  EX_rd_addr_out,
    output wire [4:0]  EX_rs1_addr_out,
    output wire [4:0]  EX_rs2_addr_out,
    output wire [4:0]  MEM_rd_addr_out,
    output wire [4:0]  WB_rd_addr_out,
    output wire        EX_RegWrite_out,
    output wire        MEM_RegWrite_out,
    output wire        WB_RegWrite_out,
    output wire        EX_MemRead_out,
    output wire        Zero_ex,
    output wire        Negative_ex,

    // NEW: pipelined control signals for data memory (outputs from EX/MEM reg)
    output wire        MemWrite_mem_out,
    output wire        MemRead_mem_out,
    output wire        MEM_MemRead_out,

    // Memory Interface (External)
    input  wire [31:0] instr_in,        // From instruction memory
    output wire [31:0] pc_out,          // To instruction memory
    output wire [31:0] mem_addr_out,    // To data memory
    output wire [31:0] mem_write_data_out, // To data memory
    input  wire [31:0] mem_read_data_in, // From data memory

    // Debug interface for testbench
    input  wire [4:0]  debug_reg_addr,
    output wire [31:0] debug_reg_data,

    // Extra debug outputs
    output wire [1:0]  dbg_forward_a_sel,
    output wire [1:0]  dbg_forward_b_sel,
    output wire [31:0] dbg_operand_a_ex,
    output wire [31:0] dbg_alu_result_ex
);

    // --- Internal wires ---
    // IF stage
    wire [31:0] pc_if;
    wire [31:0] pc_plus_4_if;

    // IF/ID registers outputs
    wire [31:0] pc_id;
    wire [31:0] instr_id;

    // ID stage
    wire [4:0]  rs1_addr_id, rs2_addr_id, rd_addr_id;
    wire [31:0] rs1_data_id, rs2_data_id;
    reg  [31:0] imm_id;

    // ID/EX registers outputs
    wire [31:0] pc_ex;
    wire [31:0] rs1_data_ex, rs2_data_ex, imm_ex;
    wire [4:0]  rs1_addr_ex, rs2_addr_ex, rd_addr_ex;
    wire [2:0]  ALUOp_ex;
    wire        ALUSrc_ex, MemRead_ex, MemWrite_ex, RegWrite_ex, MemToReg_ex;

    // EX stage
    wire [31:0] operand_a_ex, operand_b_ex;
    wire [31:0] alu_input_b_ex;
    wire [31:0] alu_result_ex;
    wire        negative_flag_ex;
    wire [31:0] branch_target_addr_ex;

    // EX/MEM register outputs
    wire [31:0] alu_result_mem, rs2_data_mem;
    wire [4:0]  rd_addr_mem;
    wire        MemRead_mem, MemWrite_mem, RegWrite_mem, MemToReg_mem;

    // MEM/WB register outputs
    wire [31:0] alu_result_wb, mem_read_data_wb;
    wire [4:0]  rd_addr_wb;
    wire        RegWrite_wb, MemToReg_wb;

    // WB stage
    wire        WB_Sel;
    wire [31:0] write_back_data_wb;

    // --- IF Stage: PC Update (module pc_update assumed available) ---
    pc_update u_pc_update (
        .clk(clk),
        .rst(rst),
        .Stall(Stall),
        .PCSrc(PCSrc),
        .branch_target(branch_target_addr_ex),
        .jalr_target(alu_result_ex),
        .pc_out(pc_if)
    );
    assign pc_out = pc_if;
    assign pc_plus_4_if = pc_if + 32'd4;

    // --- IF/ID register ---
    pipeline_register_if_id u_reg_if_id (
        .clk(clk), .rst(rst), .Stall(Stall), .Flush(Flush),
        .instr_in(instr_in), .pc_in(pc_plus_4_if),
        .instr_out(instr_id), .pc_out(pc_id)
    );

    // --- ID Stage: decode addresses and read register file ---
    assign rs1_addr_id = instr_id[19:15];
    assign rs2_addr_id = instr_id[24:20];
    assign rd_addr_id  = instr_id[11:7];

    // immediate generator (I,S,B,U,J types)
    always @(*) begin
        case (instr_id[6:0])
            7'b0010011, 7'b0000011, 7'b1100111: imm_id = {{20{instr_id[31]}}, instr_id[31:20]}; // I-type
            7'b0100011: imm_id = {{20{instr_id[31]}}, instr_id[31:25], instr_id[11:7]}; // S-type
            7'b1100011: imm_id = {{20{instr_id[31]}}, instr_id[7], instr_id[30:25], instr_id[11:8], 1'b0}; // B-type
            7'b0110111, 7'b0010111: imm_id = {instr_id[31:12], 12'b0}; // U-type
            7'b1101111: imm_id = {{12{instr_id[31]}}, instr_id[19:12], instr_id[20], instr_id[30:21], 1'b0}; // J-type
            default: imm_id = 32'hxxxxxxxx;
        endcase
    end

    // register file (module register_file assumed)
    register_file u_regfile (
        .clk(clk),
        .WriteEnable(RegWrite_wb),
        .ReadAddr1(rs1_addr_id), .ReadData1(rs1_data_id),
        .ReadAddr2(rs2_addr_id), .ReadData2(rs2_data_id),
        .WriteAddr(rd_addr_wb), .WriteData(write_back_data_wb),
        .debug_addr(debug_reg_addr), .debug_data(debug_reg_data)
    );

    // Expose ID-stage signals to controller/hazard
    assign opcode_out = instr_id[6:0];
    assign funct3_out = instr_id[14:12];
    assign funct7_out = instr_id[31:25];
    assign ID_rs1_addr_out = rs1_addr_id;
    assign ID_rs2_addr_out = rs2_addr_id;

    // --- ID/EX pipeline register ---
    pipeline_register_id_ex u_reg_id_ex (
        .clk(clk), .rst(rst), .Flush(Flush),
        // data inputs
        .pc_in(pc_id), .rs1_data_in(rs1_data_id), .rs2_data_in(rs2_data_id), .imm_in(imm_id),
        .rs1_addr_in(rs1_addr_id), .rs2_addr_in(rs2_addr_id), .rd_addr_in(rd_addr_id),
        // control inputs (from controller)
        .ALUOp_in(ALUOp), .ALUSrc_in(ALUSrc), .MemRead_in(MemRead), .MemWrite_in(MemWrite),
        .RegWrite_in(RegWrite), .MemToReg_in(MemToReg),
        // data outputs
        .pc_out(pc_ex), .rs1_data_out(rs1_data_ex), .rs2_data_out(rs2_data_ex), .imm_out(imm_ex),
        .rs1_addr_out(rs1_addr_ex), .rs2_addr_out(rs2_addr_ex), .rd_addr_out(rd_addr_ex),
        // control outputs
        .ALUOp_out(ALUOp_ex), .ALUSrc_out(ALUSrc_ex), .MemRead_out(MemRead_ex), .MemWrite_out(MemWrite_ex),
        .RegWrite_out(RegWrite_ex), .MemToReg_out(MemToReg_ex)
    );

    // Expose EX-stage addresses/status to controller/hazard
    assign EX_rd_addr_out     = rd_addr_ex;
    assign EX_rs1_addr_out    = rs1_addr_ex;
    assign EX_rs2_addr_out    = rs2_addr_ex;
    assign EX_RegWrite_out    = RegWrite_ex;
    assign EX_MemRead_out     = MemRead_ex;

    // --- EX Stage: forwarding muxes & ALU ---
    forwarding_mux_a u_fwd_a (
        .Data_ID(rs1_data_ex),
        .Data_MEM(alu_result_mem),
        .Data_WB(write_back_data_wb),
        .Sel(ForwardA_Sel),
        .Out(operand_a_ex)
    );

    forwarding_mux_b u_fwd_b (
        .Data_ID(rs2_data_ex),
        .Data_MEM(alu_result_mem),
        .Data_WB(write_back_data_wb),
        .Sel(ForwardB_Sel),
        .Out(operand_b_ex)
    );

    // debug outputs
    assign dbg_forward_a_sel = ForwardA_Sel;
    assign dbg_forward_b_sel = ForwardB_Sel;
    assign dbg_operand_a_ex  = operand_a_ex;

    // ALU input B selection
    assign alu_input_b_ex = (ALUSrc_ex) ? imm_ex : operand_b_ex;

    // branch target adder
    assign branch_target_addr_ex = pc_ex + imm_ex;

    // ALU (module alu assumed to provide Zero and Negative outputs)
    alu u_alu (
        .A(operand_a_ex),
        .B(alu_input_b_ex),
        .ALUOp(ALUOp_ex),
        .Result(alu_result_ex),
        .Zero(Zero_ex),
        .Negative(negative_flag_ex)
    );
    assign Negative_ex = negative_flag_ex;
    assign dbg_alu_result_ex = alu_result_ex;

    // --- EX/MEM pipeline register ---
    pipeline_register_ex_mem u_reg_ex_mem (
        .clk(clk), .rst(rst),
        .alu_result_in(alu_result_ex), .rs2_data_in(operand_b_ex),
        .rd_addr_in(rd_addr_ex),
        .MemRead_in(MemRead_ex), .MemWrite_in(MemWrite_ex), .RegWrite_in(RegWrite_ex), .MemToReg_in(MemToReg_ex),
        .alu_result_out(alu_result_mem), .rs2_data_out(rs2_data_mem),
        .rd_addr_out(rd_addr_mem),
        .MemRead_out(MemRead_mem), .MemWrite_out(MemWrite_mem), .RegWrite_out(RegWrite_mem), .MemToReg_out(MemToReg_mem)
    );

    // expose MEM-stage rd/regwrite to controller/hazard
    assign MEM_rd_addr_out = rd_addr_mem;
    assign MEM_RegWrite_out = RegWrite_mem;

    // pipeline control outputs for top-level / data memory / hazard unit
    assign MemWrite_mem_out = MemWrite_mem;
    assign MemRead_mem_out  = MemRead_mem;
    assign MEM_MemRead_out  = MemRead_mem; // used by controller/hazard to detect loads in MEM stage

    // --- MEM Stage: connect to external memory interface ---
    assign mem_addr_out = alu_result_mem;
    assign mem_write_data_out = rs2_data_mem;
    // mem_read_data_in arrives from external data memory

    // --- MEM/WB pipeline register ---
    pipeline_register_mem_wb u_reg_mem_wb (
        .clk(clk), .rst(rst),
        .alu_result_in(alu_result_mem), .mem_read_data_in(mem_read_data_in),
        .rd_addr_in(rd_addr_mem),
        .RegWrite_in(RegWrite_mem), .MemToReg_in(MemToReg_mem),
        .alu_result_out(alu_result_wb), .mem_read_data_out(mem_read_data_wb),
        .rd_addr_out(rd_addr_wb),
        .RegWrite_out(RegWrite_wb), .MemToReg_out(MemToReg_wb)
    );

    assign WB_rd_addr_out = rd_addr_wb;
    assign WB_RegWrite_out = RegWrite_wb;

    // --- WB Stage: select data to write back ---
    assign WB_Sel = MemToReg_wb ? 1'b1 : 1'b0;
    writeback_mux u_wb_mux (
        .Data_ALU(alu_result_wb),
        .Data_MEM(mem_read_data_wb),
        .Sel(WB_Sel),
        .Out(write_back_data_wb)
    );

endmodule

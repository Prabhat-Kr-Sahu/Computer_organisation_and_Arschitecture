`timescale 1ns / 1ps

//==================================================
// ## 3. MID-LEVEL MODULES (Datapath and Controller)
//==================================================

// --- Datapath ---
////////endmodule 
module datapath (
    input clk, rst,
    // Control Inputs from Controller
    input [1:0] ForwardA_Sel, ForwardB_Sel, // ADDED BACK: Now controlled by hazard_unit in Controller
    input [1:0] PCSrc, // New input for PC selection
    input       Stall, Flush,
    // Control signals for the *current* ID stage
    input       ALUSrc, RegWrite, MemRead, MemWrite, MemToReg,
    input [2:0] ALUOp,

    // Status Outputs TO Controller
    output wire [6:0] opcode_out, funct7_out,
    output wire [2:0] funct3_out,
    output wire [4:0] ID_rs1_addr_out, ID_rs2_addr_out,
    output wire [4:0] EX_rd_addr_out,
    output wire [4:0] MEM_rd_addr_out,
    // EX-stage source addresses (from ID/EX register) used by Controller/Hazard Unit
    output wire [4:0] EX_rs1_addr_out, EX_rs2_addr_out,
    output wire [4:0] WB_rd_addr_out,
    output wire       EX_RegWrite_out,
    output wire       MEM_RegWrite_out,
    output wire       WB_RegWrite_out,
    output wire       EX_MemRead_out,
    output wire       Zero_ex, // New output for branch condition
    output wire       Negative_ex, // NEW: Output for signed branch condition

    // NEW: Pipelined control signals for Data Memory
    output wire MemWrite_mem_out,
    output wire MemRead_mem_out,
    output wire MEM_MemRead_out, // NEW: Output for pipelined MemRead signal for hazard unit

    // Memory Interface (External)
    input  wire [31:0] instr_in,        // From Instruction Memory
    output wire [31:0] pc_out,          // To Instruction Memory Addr
    output wire [31:0] mem_addr_out,    // To Data Memory Addr
    output wire [31:0] mem_write_data_out, // To Data Memory Write Data
    input  wire [31:0] mem_read_data_in, // From Data Memory Read Data
    
    // Debug interface for testbench
    input  wire [4:0]  debug_reg_addr,
    output wire [31:0] debug_reg_data
    ,
    // Extra debug outputs
    output wire [1:0] dbg_forward_a_sel,
    output wire [1:0] dbg_forward_b_sel,
    output wire [31:0] dbg_operand_a_ex,
    output wire [31:0] dbg_alu_result_ex
);
    // --- Internal Wires ---
    // Wires for forwarding unit
    wire [1:0] forward_a_sel_internal;
    wire [1:0] forward_b_sel_internal;

    // IF Stage
    wire [31:0] pc_if;
    wire [31:0] pc_plus_4_if;

    // IF/ID Register Outputs
    wire [31:0] pc_id, instr_id;

    // ID Stage
    wire [31:0] rs1_data_id, rs2_data_id;
    reg  [31:0] imm_id;
    wire [4:0]  rs1_addr_id, rs2_addr_id, rd_addr_id;

    // ID/EX Register Outputs
    wire [31:0] pc_ex, rs1_data_ex, rs2_data_ex, imm_ex;
    wire [4:0]  rs1_addr_ex, rs2_addr_ex, rd_addr_ex;
    wire [2:0]  ALUOp_ex;
    wire        ALUSrc_ex, MemRead_ex, MemWrite_ex, RegWrite_ex, MemToReg_ex;

    // EX Stage
    wire [31:0] operand_a_ex, operand_b_ex;
    wire [31:0] alu_input_b_ex;
    wire [31:0] alu_result_ex;
    wire        negative_flag_ex; // NEW: Internal wire for ALU's Negative flag
    wire [31:0] branch_target_addr_ex; // For branches and JAL

    // EX/MEM Register Outputs
    wire [31:0] alu_result_mem, rs2_data_mem;
    wire [4:0]  rd_addr_mem;
    wire        MemRead_mem, MemWrite_mem, RegWrite_mem, MemToReg_mem;

    // MEM/WB Register Outputs
    wire [31:0] alu_result_wb, mem_read_data_wb;
    wire [4:0]  rd_addr_wb;
    wire        RegWrite_wb, MemToReg_wb;

    // WB Stage
    wire        WB_Sel; // Control for writeback mux (0=ALU,1=MEM)
    wire [31:0] write_back_data_wb;

    // --- IF Stage --- // *** PC LOGIC MOVED TO pc_update MODULE ***
    pc_update u_pc_update (
        .clk(clk),
        .rst(rst),
        .Stall(Stall),
        .PCSrc(PCSrc),
        .branch_target(branch_target_addr_ex),
        .jalr_target(alu_result_ex),
        .pc_out(pc_if)
    );
    assign pc_out = pc_if; // Output current PC to instruction memory
    assign pc_plus_4_if = pc_if + 32'd4; // Needed for IF/ID register

    // --- IF/ID Pipeline Register ---
    pipeline_register_if_id u_reg_if_id (
        .clk(clk), .rst(rst), .Stall(Stall), .Flush(Flush),
        .instr_in(instr_in), .pc_in(pc_plus_4_if), // Store PC+4
        .instr_out(instr_id), .pc_out(pc_id)
    );

    // --- ID Stage ---
    // Extract register addresses from instruction
    assign rs1_addr_id = instr_id[19:15];
    assign rs2_addr_id = instr_id[24:20];
    assign rd_addr_id  = instr_id[11:7];

    // *** IMPLEMENTED IMMEDIATE GENERATOR ***
    // Generates the correct immediate value based on instruction opcode
    always @(*) begin
        case (instr_id[6:0]) // opcode
            // I-type (ALU imm, Load, JALR)
            7'b0010011, 7'b0000011, 7'b1100111:
                imm_id = {{20{instr_id[31]}}, instr_id[31:20]};
            // S-type (Store)
            7'b0100011:
                imm_id = {{20{instr_id[31]}}, instr_id[31:25], instr_id[11:7]};
            // B-type (Branch)
            7'b1100011:
                imm_id = {{20{instr_id[31]}}, instr_id[7], instr_id[30:25], instr_id[11:8], 1'b0};
            // U-type (LUI, AUIPC)
            7'b0110111, 7'b0010111:
                imm_id = {instr_id[31:12], 12'b0};
            // J-type (JAL)
            7'b1101111:
                imm_id = {{12{instr_id[31]}}, instr_id[19:12], instr_id[20], instr_id[30:21], 1'b0};
            default:
                imm_id = 32'hxxxxxxxx; // Default to X for unknown opcodes
        endcase
    end

    // Instantiate Register File
    register_file u_regfile (
        .clk(clk), .WriteEnable(RegWrite_wb), // Write enable from WB stage
        .ReadAddr1(rs1_addr_id), .ReadData1(rs1_data_id),
        .ReadAddr2(rs2_addr_id), .ReadData2(rs2_data_id),
        .WriteAddr(rd_addr_wb), .WriteData(write_back_data_wb), // Write data from WB stage
        .debug_addr(debug_reg_addr), .debug_data(debug_reg_data) // Debug ports
    );

    // Outputs for Controller/Hazard Unit
    assign opcode_out = instr_id[6:0];
    assign funct3_out = instr_id[14:12];
    assign funct7_out = instr_id[31:25];
    assign ID_rs1_addr_out = rs1_addr_id;
    assign ID_rs2_addr_out = rs2_addr_id;
    // EX-stage source addresses (propagate the ID/EX register addresses)
    assign EX_rs1_addr_out = rs1_addr_ex;
    assign EX_rs2_addr_out = rs2_addr_ex;

    // --- ID/EX Pipeline Register ---
    pipeline_register_id_ex u_reg_id_ex (
        .clk(clk), .rst(rst), .Flush(Flush), // Stall does NOT affect this reg directly
        // Data Inputs
        .pc_in(pc_id), .rs1_data_in(rs1_data_id), .rs2_data_in(rs2_data_id), .imm_in(imm_id),
        .rs1_addr_in(rs1_addr_id), .rs2_addr_in(rs2_addr_id), .rd_addr_in(rd_addr_id),
        // Control Signal Inputs (from Controller)
        .ALUOp_in(ALUOp), .ALUSrc_in(ALUSrc), .MemRead_in(MemRead), .MemWrite_in(MemWrite),
    .RegWrite_in(RegWrite), .MemToReg_in(MemToReg),
        // Data Outputs
        .pc_out(pc_ex), .rs1_data_out(rs1_data_ex), .rs2_data_out(rs2_data_ex), .imm_out(imm_ex),
        .rs1_addr_out(rs1_addr_ex), .rs2_addr_out(rs2_addr_ex), .rd_addr_out(rd_addr_ex),
        // Control Signal Outputs
    .ALUOp_out(ALUOp_ex), .ALUSrc_out(ALUSrc_ex), .MemRead_out(MemRead_ex), .MemWrite_out(MemWrite_ex),
    .RegWrite_out(RegWrite_ex), .MemToReg_out(MemToReg_ex)
    );
    // Outputs for Controller/Hazard Unit (from ID/EX Reg)
    assign EX_rd_addr_out = rd_addr_ex;
    assign EX_RegWrite_out = RegWrite_ex;
    assign EX_MemRead_out = MemRead_ex; // Pass MemRead status needed by Hazard Unit


    // --- EX Stage ---
    // Forwarding MUXes select ALU inputs. The 'Sel' signals now come from the controller.
    forwarding_mux_a u_fwd_a (
        .Data_ID(rs1_data_ex),       // Default from ID/EX reg
        .Data_MEM(alu_result_mem),   // Forward from EX/MEM reg output
        .Data_WB(write_back_data_wb),// Forward from MEM/WB reg output -> WB Mux output
        .Sel(ForwardA_Sel), .Out(operand_a_ex)
    );
    forwarding_mux_b u_fwd_b (
        .Data_ID(rs2_data_ex),
        .Data_MEM(alu_result_mem),
        .Data_WB(write_back_data_wb),
        .Sel(ForwardB_Sel), .Out(operand_b_ex)
    );

    // Debug exports: show which forwarding sel is active and EX operands
    assign dbg_forward_a_sel = ForwardA_Sel;
    assign dbg_forward_b_sel = ForwardB_Sel;
    assign dbg_operand_a_ex = operand_a_ex;

    // ALU Input B MUX (Select between RegData or Immediate)
    assign alu_input_b_ex = ALUSrc_ex ? imm_ex : operand_b_ex;

    // *** ADDED BRANCH TARGET ADDER ***
    // Calculates target for conditional branches and JAL
    assign branch_target_addr_ex = pc_ex + imm_ex;

    // Instantiate ALU
    // *** MODIFIED ALU to output Zero and Negative flags ***
    alu u_alu (
        .A(operand_a_ex), .B(alu_input_b_ex), .ALUOp(ALUOp_ex),
        .Result(alu_result_ex), .Zero(Zero_ex), .Negative(negative_flag_ex)
    );

    // Connect the new Negative flag to the datapath output
    assign Negative_ex = negative_flag_ex;

    // Debug ALU result before passing to EX/MEM regs
    assign dbg_alu_result_ex = alu_result_ex;


    // --- EX/MEM Pipeline Register --- // *** INSTANTIATED ***
    pipeline_register_ex_mem u_reg_ex_mem (
        .clk(clk), .rst(rst),
        // Data Inputs
        .alu_result_in(alu_result_ex), .rs2_data_in(operand_b_ex), // Pass rs2 data for SW
        .rd_addr_in(rd_addr_ex),
        // Control Inputs
        .MemRead_in(MemRead_ex), .MemWrite_in(MemWrite_ex), .RegWrite_in(RegWrite_ex), .MemToReg_in(MemToReg_ex),
        // Data Outputs
        .alu_result_out(alu_result_mem), .rs2_data_out(rs2_data_mem),
        .rd_addr_out(rd_addr_mem),
        // Control Outputs
        .MemRead_out(MemRead_mem), .MemWrite_out(MemWrite_mem), .RegWrite_out(RegWrite_mem), .MemToReg_out(MemToReg_mem)
    );
    // Outputs for Controller/Hazard Unit (from EX/MEM Reg)
    assign MEM_rd_addr_out = rd_addr_mem;
    assign MEM_RegWrite_out = RegWrite_mem;

    // NEW Assignments for pipelined signals
    assign MemWrite_mem_out = MemWrite_mem;
    assign MemRead_mem_out = MemRead_mem;
    assign MEM_MemRead_out = MemRead_mem; // Assign to new output


    // --- MEM Stage ---
    // Memory Address and Write Data determined
    assign mem_addr_out = alu_result_mem;     // Address is ALU result from EX/MEM reg
    assign mem_write_data_out = rs2_data_mem; // Write data is rs2 from EX/MEM reg

    // mem_read_data_in comes from external memory module


    // --- MEM/WB Pipeline Register --- // *** INSTANTIATED ***
    pipeline_register_mem_wb u_reg_mem_wb (
        .clk(clk), .rst(rst),
        // Data Inputs
        .alu_result_in(alu_result_mem), .mem_read_data_in(mem_read_data_in),
        .rd_addr_in(rd_addr_mem),
        // Control Inputs
        .RegWrite_in(RegWrite_mem), .MemToReg_in(MemToReg_mem),
        // Data Outputs
        .alu_result_out(alu_result_wb), .mem_read_data_out(mem_read_data_wb),
        .rd_addr_out(rd_addr_wb),
        // Control Outputs
        .RegWrite_out(RegWrite_wb), .MemToReg_out(MemToReg_wb)
    );
    // Outputs for Controller/Hazard Unit (from MEM/WB Reg)
    assign WB_rd_addr_out = rd_addr_wb;
    assign WB_RegWrite_out = RegWrite_wb;


    // --- WB Stage ---
    // *** IMPLEMENTED WB_Sel Logic ***
    // Selects based on MemToReg (from MEM/WB reg)
    assign WB_Sel = MemToReg_wb ? 1'b1 : 1'b0; // 1 -> MEM, 0 -> ALU

    // Instantiate Writeback MUX
    writeback_mux u_wb_mux (
        .Data_ALU(alu_result_wb),
        .Data_MEM(mem_read_data_wb),
        .Sel(WB_Sel),
        .Out(write_back_data_wb) // Final data to write to Register File
    );

endmodule

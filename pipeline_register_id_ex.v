`timescale 1ns / 1ps

// --- ID/EX Pipeline Register ---
module pipeline_register_id_ex (
    input  wire        clk, rst, Flush,
    // Data Inputs
    input  wire [31:0] pc_in, rs1_data_in, rs2_data_in, imm_in,
    input  wire [4:0]  rs1_addr_in, rs2_addr_in, rd_addr_in,
    // Control Signal Inputs
    input  wire [2:0]  ALUOp_in,
    input  wire        ALUSrc_in, MemRead_in, MemWrite_in, RegWrite_in,
    input  wire        MemToReg_in,
    // Data Outputs
    output reg  [31:0] pc_out, rs1_data_out, rs2_data_out, imm_out,
    output reg  [4:0]  rs1_addr_out, rs2_addr_out, rd_addr_out,
    // Control Signal Outputs
    output reg  [2:0]  ALUOp_out,
    output reg         ALUSrc_out, MemRead_out, MemWrite_out, RegWrite_out,
    output reg         MemToReg_out
);
    always @(posedge clk or posedge rst) begin
        if (rst || Flush) begin // Flush with NOP control signals
            pc_out <= 0; rs1_data_out <= 0; rs2_data_out <= 0; imm_out <= 0;
            rs1_addr_out <= 0; rs2_addr_out <= 0; rd_addr_out <= 0;
            ALUOp_out <= 0; ALUSrc_out <= 0; MemRead_out <= 0; MemWrite_out <= 0;
            RegWrite_out <= 0; MemToReg_out <= 0;
        end else begin // Normal operation (no stall logic needed here)
            pc_out <= pc_in; rs1_data_out <= rs1_data_in; rs2_data_out <= rs2_data_in; imm_out <= imm_in;
            rs1_addr_out <= rs1_addr_in; rs2_addr_out <= rs2_addr_in; rd_addr_out <= rd_addr_in;
            ALUOp_out <= ALUOp_in; ALUSrc_out <= ALUSrc_in; MemRead_out <= MemRead_in; MemWrite_out <= MemWrite_in;
            RegWrite_out <= RegWrite_in; MemToReg_out <= MemToReg_in;
        end
    end
endmodule

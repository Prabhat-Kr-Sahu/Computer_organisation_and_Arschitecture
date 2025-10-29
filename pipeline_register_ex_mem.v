`timescale 1ns / 1ps

// --- EX/MEM Pipeline Register --- // *** IMPLEMENTED ***
module pipeline_register_ex_mem (
    input clk, rst,
    // Data Inputs
    input [31:0] alu_result_in, rs2_data_in, // rs2 needed for SW
    input [4:0]  rd_addr_in,
    // Control Inputs
    input MemRead_in, MemWrite_in, RegWrite_in, MemToReg_in,
    // Output Data
    output reg [31:0] alu_result_out, rs2_data_out,
    output reg [4:0]  rd_addr_out,
    // Output Control
    output reg MemRead_out, MemWrite_out, RegWrite_out, MemToReg_out
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
             alu_result_out <= 0; rs2_data_out <= 0; rd_addr_out <= 0;
             MemRead_out <= 0; MemWrite_out <= 0; RegWrite_out <= 0; MemToReg_out <= 0;
        end else begin
             alu_result_out <= alu_result_in;
             rs2_data_out <= rs2_data_in; rd_addr_out <= rd_addr_in;
             MemRead_out <= MemRead_in; MemWrite_out <= MemWrite_in;
             RegWrite_out <= RegWrite_in; MemToReg_out <= MemToReg_in;
        end
    end
endmodule

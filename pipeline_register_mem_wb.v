`timescale 1ns / 1ps

// --- MEM/WB Pipeline Register --- // *** IMPLEMENTED ***
module pipeline_register_mem_wb (
    input clk, rst,
    // Data Inputs
    input [31:0] alu_result_in, mem_read_data_in,
    input [4:0]  rd_addr_in,
    // Control Inputs
    input RegWrite_in, MemToReg_in,
    // Data Outputs
    output reg [31:0] alu_result_out, mem_read_data_out,
    output reg [4:0]  rd_addr_out,
    // Control Outputs
    output reg RegWrite_out, MemToReg_out
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            alu_result_out <= 0; mem_read_data_out <= 0; rd_addr_out <= 0;
            RegWrite_out <= 0; MemToReg_out <= 0;
        end else begin
            alu_result_out <= alu_result_in;
            mem_read_data_out <= mem_read_data_in; rd_addr_out <= rd_addr_in;
            RegWrite_out <= RegWrite_in; MemToReg_out <= MemToReg_in;
        end
    end
endmodule

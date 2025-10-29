`timescale 1ns / 1ps

// --- IF/ID Pipeline Register ---
module pipeline_register_if_id (
    input  wire        clk, rst, Stall, Flush,
    input  wire [31:0] instr_in, pc_in,
    output reg  [31:0] instr_out, pc_out
);
    always @(posedge clk or posedge rst) begin
        if (rst || Flush) begin
            instr_out <= 32'h00000013; pc_out <= 32'h0; // NOP
        end else if (~Stall) begin
            instr_out <= instr_in; pc_out <= pc_in;
        end
        // If Stall, hold current value
    end
endmodule

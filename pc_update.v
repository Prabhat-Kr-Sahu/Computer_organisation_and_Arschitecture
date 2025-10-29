`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/29/2025 06:46:55 PM
// Design Name: 
// Module Name: pc_update
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
module pc_update (
    input  wire        clk,
    input  wire        rst,
    input  wire        Stall,
    input  wire [1:0]  PCSrc,           // 00: PC+4, 01: Branch/JAL, 10: JALR
    input  wire [31:0] branch_target,   // From EX stage branch adder
    input  wire [31:0] jalr_target,     // From EX stage main ALU
    output wire [31:0] pc_out           // Current PC to instruction memory
);

    reg [31:0] pc_reg;
    wire [31:0] pc_plus_4;
    reg  [31:0] pc_target;
    wire [31:0] next_pc;

    // Output current PC
    assign pc_out = pc_reg;

    // PC+4 calculation
    assign pc_plus_4 = pc_reg + 32'd4;

    // Choose target (combinational)
    // For JALR, align LSB to 0 per RISC-V spec (optional but safe)
    always @(*) begin
        case (PCSrc)
            2'b01: pc_target = branch_target;            // Branch or JAL
            2'b10: pc_target = {jalr_target[31:1], 1'b0}; // JALR: clear LSB
            default: pc_target = pc_plus_4;              // Sequential
        endcase
    end

    // Stall: hold PC_reg when Stall is asserted
    assign next_pc = Stall ? pc_reg : pc_target;

    // PC register with synchronous update and synchronous reset to 0
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc_reg <= 32'h0;
        end else begin
            pc_reg <= next_pc;
        end
    end

endmodule

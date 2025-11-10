`timescale 1ns / 1ps

// --- Register File ---
module register_file (
    input  wire        clk,
    input  wire        WriteEnable,
    input  wire [4:0]  ReadAddr1,
    input  wire [4:0]  ReadAddr2,
    input  wire [4:0]  WriteAddr,
    input  wire [31:0] WriteData,
    output wire [31:0] ReadData1,
    output wire [31:0] ReadData2,
    // Debug ports for testbench
    input  wire [4:0]  debug_addr,
    output wire [31:0] debug_data
);
    reg [31:0] registers [31:0];
    assign ReadData1 = (ReadAddr1 == 5'b0) ? 32'h0 : registers[ReadAddr1];
    assign ReadData2 = (ReadAddr2 == 5'b0) ? 32'h0 : registers[ReadAddr2];
    // Debug read port
    assign debug_data = (debug_addr == 5'b0) ? 32'h0 : registers[debug_addr];
    always @(posedge clk) begin
        if (WriteEnable && (WriteAddr != 5'b0)) begin
            registers[WriteAddr] <= WriteData;
        end
    end
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) registers[i] = 32'h0;
    end
endmodule

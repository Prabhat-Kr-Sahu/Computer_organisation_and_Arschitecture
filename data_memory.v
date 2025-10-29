`timescale 1ns / 1ps

// --- Data Memory --- // Changed comment
module data_memory ( // Renamed module
    input  wire        clk,
    input  wire [31:0] addr,
    input  wire [31:0] write_data,
    input  wire        MemRead,
    input  wire        MemWrite,
    output reg  [31:0] read_data
);
    // 1024-entry, 32-bit wide memory (4KB) - Reverted size assumption based on context
    reg [31:0] dmem [1023:0]; // Renamed internal memory array

    // Read uses 10 address bits [11:2] for 1024 words
    always @(posedge clk) begin
        if (MemRead) read_data <= dmem[addr[11:2]]; // Use correct indexing and array name
    end

    // Write uses 10 address bits [11:2] for 1024 words
    always @(posedge clk) begin
        if (MemWrite) dmem[addr[11:2]] <= write_data; // Use correct indexing and array name
    end

endmodule


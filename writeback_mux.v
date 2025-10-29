`timescale 1ns / 1ps

// --- Writeback MUX ---
module writeback_mux (
    input  wire [31:0] Data_ALU, Data_MEM,
    input  wire        Sel, // MemToReg control
    output wire [31:0] Out
);
    assign Out = Sel ? Data_MEM : Data_ALU; // Simple mux between ALU result and memory data
endmodule

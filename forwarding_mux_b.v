`timescale 1ns / 1ps

// --- Forwarding MUX B ---
module forwarding_mux_b (
    input  wire [31:0] Data_ID,    // From ID/EX register
    input  wire [31:0] Data_MEM,   // From EX/MEM register
    input  wire [31:0] Data_WB,    // From MEM/WB register writeback
    input  wire [1:0]  Sel,
    output wire [31:0] Out
);
    assign Out = (Sel == 2'b00) ? Data_ID :    // No forwarding
                 (Sel == 2'b01) ? Data_MEM :    // Forward from EX/MEM
                 (Sel == 2'b10) ? Data_WB :     // Forward from MEM/WB
                 32'h0;                         // Default case
endmodule

// --- Instruction Memory ---
module instruction_memory (
    input  wire [31:0] addr,
    output wire [31:0] data
);
    // 2048-entry, 32-bit wide memory (8KB) - Increased Size
    reg [31:0] memory [2047:0];

    // Read uses 11 address bits [12:2] for 2048 words
    assign data = memory[addr[12:2]];

endmodule

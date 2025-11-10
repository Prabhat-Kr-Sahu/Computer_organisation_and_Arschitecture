module forwarding_mux_b (
    input  wire [31:0] Data_ID,    // From ID/EX register (no forward)
    input  wire [31:0] Data_MEM,   // From EX/MEM register (forward from EX/MEM)
    input  wire [31:0] Data_WB,    // From MEM/WB register (forward from MEM/WB)
    input  wire [1:0]  Sel,
    output reg  [31:0] Out
);
    always @(*) begin
        case (Sel)
            2'b00: Out = Data_ID;   // No forwarding
            2'b01: Out = Data_MEM;  // Forward from EX/MEM
            2'b10: Out = Data_WB;   // Forward from MEM/WB
            default: Out = 32'h0;
        endcase
    end
endmodule

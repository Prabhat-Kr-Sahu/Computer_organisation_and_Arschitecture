`timescale 1ns / 1ps

// --- ALU ---
module alu (
    input  wire [31:0] A,
    input  wire [31:0] B,
    input  wire [2:0]  ALUOp,
    output reg  [31:0] Result,
    output reg         Zero,
    output reg         Negative // NEW: For signed comparisons
);
    localparam ALU_ADD  = 3'b000;
    localparam ALU_SUB  = 3'b001;
    localparam ALU_MUL_OP = 3'b010; // Renamed to avoid clash
    localparam ALU_SLL  = 3'b011;
    localparam ALU_SRL  = 3'b100;
    localparam ALU_XOR  = 3'b101;
    localparam ALU_OR   = 3'b110;
    localparam ALU_AND  = 3'b111;

    // Combinational logic for ALU operations
    always @(*) begin
        // Default outputs
        Result = 32'hxxxxxxxx;
        Zero = 1'b0;
        Negative = 1'b0; // NEW: Default to not negative

        case (ALUOp)
            ALU_ADD:  Result = A + B;
            ALU_SUB:  Result = A - B;
            ALU_SLL:  Result = A << B[4:0];
            ALU_SRL:  Result = A >> B[4:0];
            ALU_XOR:  Result = A ^ B;
            ALU_OR:   Result = A | B;
            ALU_AND:  Result = A & B;
            default: Result = 32'hDEADBEEF;
        endcase

        // Set Zero and Negative flags based on the final result
        if (Result == 32'b0)
            Zero = 1'b1;

        Negative = Result[31]; // NEW: Set Negative flag if MSB is 1
    end

endmodule

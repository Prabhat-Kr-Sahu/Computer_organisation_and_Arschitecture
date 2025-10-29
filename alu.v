`timescale 1ns / 1ps

// --- ALU ---
module alu (
    input  wire [31:0] A,
    input  wire [31:0] B,
    input  wire [2:0]  ALUOp,
    output reg  [31:0] Result,
    output wire        Zero
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
    end

    // Zero flag is asserted if the result is 0
    assign Zero = (Result == 32'b0);

endmodule

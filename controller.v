`timescale 1ns / 1ps

// --- THIS IS THE FIXED MODULE DEFINITION ---
// I have added all the missing output ports that your
// processor_top module expects.
module controller (
    input  clk, rst,
    // Inputs from Datapath (ID Stage)
    input  wire [6:0] opcode,
    input  wire [2:0] funct3,
    input  wire [6:0] funct7,
    // Inputs from Datapath for Hazard Unit
    input  wire [4:0] ID_rs1, ID_rs2,
    input  wire [4:0] EX_rs1, EX_rs2, EX_rd,
    input  wire [4:0] MEM_rd, WB_rd,
    input  wire       EX_RegWrite, MEM_RegWrite, WB_RegWrite,
    input  wire       EX_MemRead, MEM_MemRead,
    input  wire       Zero_ex,
    input  wire       Negative_ex,

    // Outputs to Datapath
    output wire [1:0] ForwardA_Sel,
    output wire [1:0] ForwardB_Sel,
    output wire [1:0] PCSrc,
    output wire       Stall,
    output wire       Flush,
    output wire       ALUSrc, RegWrite, MemRead, MemWrite, MemToReg,
    output wire [2:0] ALUOp
);
// --- END OF FIXED MODULE DEFINITION ---


    // --- Internal Wires ---
    wire       branch_taken;
    reg        branch_cond_met;

    // --- Internal wires for decoder outputs ---
    wire       dec_ALUSrc, dec_RegWrite, dec_MemRead, dec_MemWrite, dec_MemToReg;
    wire [2:0] dec_ALUOp;

    // 1. Main Decoder: Generates primary control signals from opcode
    main_decoder u_main_decoder (
        .opcode(opcode),
        .funct3(funct3), 
        .funct7(funct7), 
        .RegWrite(dec_RegWrite),
        .MemRead(dec_MemRead),
        .MemWrite(dec_MemWrite),
        .MemToReg(dec_MemToReg),
        .ALUSrc(dec_ALUSrc),
        .ALUOp(dec_ALUOp)
    );
    
    // 2. Hazard Unit (Assuming this module exists and is correct)
    /*
    hazard_unit u_hazard_unit (
        .ID_rs1(ID_rs1), .ID_rs2(ID_rs2),
        .EX_rs1(EX_rs1), .EX_rs2(EX_rs2), .EX_rd(EX_rd),
        .MEM_rd(MEM_rd), .WB_rd(WB_rd),
        .EX_RegWrite(EX_RegWrite), .MEM_RegWrite(MEM_RegWrite), .WB_RegWrite(WB_RegWrite),
        .EX_MemRead(EX_MemRead),
        .MEM_MemRead(MEM_MemRead),
        .ForwardA_Sel(ForwardA_Sel),
        .ForwardB_Sel(ForwardB_Sel),
        .Stall(Stall),
        .Flush(Flush)
    );
    */
    
    // --- TEMPORARY FIX if no hazard unit: ---
    // If you don't have a hazard unit yet, uncomment these lines
    // This will allow compilation, but your pipeline WILL NOT handle hazards.
     assign ForwardA_Sel = 2'b00;
     assign ForwardB_Sel = 2'b00;
     assign Stall = 1'b0;
     assign Flush = 1'b0;
    // --- End of temporary fix ---


    // 3. Control Logic for PC Source
    // This logic is now expanded to handle all standard branch types.
    always @(*) begin
        case (funct3)
            3'b000: branch_cond_met = Zero_ex;    // BEQ: branch if Z=1
            3'b001: branch_cond_met = ~Zero_ex;   // BNE: branch if Z=0
            3'b100: branch_cond_met = Negative_ex;  // BLT: branch if N=1
            3'b101: branch_cond_met = ~Negative_ex; // BGE: branch if N=0
            // Note: BLTU/BGEU would require a Carry flag from the ALU
            default: branch_cond_met = 1'b0;
        endcase
    end

    assign branch_taken = (opcode == 7'b1100011) && branch_cond_met;

    // PCSrc Mux Control:
    // 2'b00: PC + 4
    // 2'b01: Branch Target
    // 2'b10: JALR Target (Needs to be added for JALR)
    // 2'b11: JAL Target (Needs to be added for JAL)
    
    // --- SIMPLIFIED PCSrc for now (assumes no JAL/JALR logic) ---
    // You will need to expand this to handle JAL and JALR opcodes
    assign PCSrc = branch_taken ? 2'b01 : 2'b00;


    // --- 4. NOP INJECTION LOGIC ---
    // If Stall is asserted, force control signals to 0 (a NOP)
    assign RegWrite = dec_RegWrite & ~Stall;
    assign MemRead  = dec_MemRead  & ~Stall;
    assign MemWrite = dec_MemWrite & ~Stall;
    assign MemToReg = dec_MemToReg & ~Stall;
    assign ALUSrc   = dec_ALUSrc   & ~Stall;
    assign ALUOp    = Stall ? 3'b000 : dec_ALUOp; // Force to a known safe op (ADD)
    
endmodule
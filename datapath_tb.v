`timescale 1ns/1ps

module tb0000_datapath();
    // Clock & reset
    reg clk = 0;
    reg rst = 1;
    
    // Loop counters
    integer i;
    integer cycle;

    // Debug interface for register inspection
    reg  [4:0]  debug_reg_addr = 5'd0;
    wire [31:0] debug_reg_data;

    // Control signals (driven by testbench to emulate Controller behavior)
    reg [1:0] ForwardA_Sel = 2'b00;
    reg [1:0] ForwardB_Sel = 2'b00;
    reg [1:0] PCSrc = 2'b00;
    reg       Stall = 1'b0;
    reg       Flush = 1'b0;
    reg       ALUSrc = 1'b1;   // use immediate
    reg       RegWrite = 1'b1; // enable register writes
    reg       MemRead = 1'b0;
    reg       MemWrite = 1'b0;
    reg       MemToReg = 1'b0; // write ALU result back
    reg [2:0] ALUOp = 3'b000; // ADD

    // Instruction / Memory interfaces
    reg  [31:0] instr_in;
    wire [31:0] pc_out;
    wire [31:0] mem_addr_out;
    wire [31:0] mem_write_data_out;
    reg  [31:0] mem_read_data_in = 32'd0;

    // Status outputs from datapath
    wire [6:0] opcode_out;
    wire [2:0] funct3_out;
    wire [6:0] funct7_out;
    wire [4:0] ID_rs1_addr_out, ID_rs2_addr_out;
    wire [4:0] EX_rd_addr_out, MEM_rd_addr_out, WB_rd_addr_out;
    wire [4:0] EX_rs1_addr_out, EX_rs2_addr_out;
    wire       EX_RegWrite_out, MEM_RegWrite_out, WB_RegWrite_out;
    wire       EX_MemRead_out;
    wire       Zero_ex;
    // Debug signals from datapath
    wire [1:0] dbg_forward_a_sel;
    wire [1:0] dbg_forward_b_sel;
    wire [31:0] dbg_operand_a_ex;
    wire [31:0] dbg_alu_result_ex;

    // Add task to check register values safely using debug interface
    task check_register_state;
        input [4:0] reg_num;
        input [31:0] expected_value;
        begin
            debug_reg_addr = reg_num;
            #1; // Allow combinational logic to settle
            if(debug_reg_data === expected_value)
                $display("✓ R%0d = %0d (correct)", reg_num, expected_value);
            else
                $display("✗ R%0d = %0d (expected %0d)", reg_num, 
                    debug_reg_data, expected_value);
        end
    endtask

    // Instantiate DUT
    datapath dut (
        .clk(clk), .rst(rst),
        .ForwardA_Sel(ForwardA_Sel), .ForwardB_Sel(ForwardB_Sel),
        .PCSrc(PCSrc), .Stall(Stall), .Flush(Flush),
        .ALUSrc(ALUSrc), .RegWrite(RegWrite), 
        .MemRead(MemRead), .MemWrite(MemWrite), 
        .MemToReg(MemToReg), .ALUOp(ALUOp),
        .opcode_out(opcode_out), .funct7_out(funct7_out),
        .funct3_out(funct3_out),
        .ID_rs1_addr_out(ID_rs1_addr_out), 
        .ID_rs2_addr_out(ID_rs2_addr_out),
        .EX_rd_addr_out(EX_rd_addr_out), 
        .MEM_rd_addr_out(MEM_rd_addr_out),
        .EX_rs1_addr_out(EX_rs1_addr_out), 
        .EX_rs2_addr_out(EX_rs2_addr_out),
        .WB_rd_addr_out(WB_rd_addr_out),
        .EX_RegWrite_out(EX_RegWrite_out), 
        .MEM_RegWrite_out(MEM_RegWrite_out), 
        .WB_RegWrite_out(WB_RegWrite_out),
        .EX_MemRead_out(EX_MemRead_out), 
        .Zero_ex(Zero_ex),
        .instr_in(instr_in), .pc_out(pc_out),
        .mem_addr_out(mem_addr_out), 
        .mem_write_data_out(mem_write_data_out),
        .mem_read_data_in(mem_read_data_in),
        .debug_reg_addr(debug_reg_addr),
        .debug_reg_data(debug_reg_data)
        , .dbg_forward_a_sel(dbg_forward_a_sel)
        , .dbg_forward_b_sel(dbg_forward_b_sel)
        , .dbg_operand_a_ex(dbg_operand_a_ex)
        , .dbg_alu_result_ex(dbg_alu_result_ex)
    );

    // Small instruction memory (16 words)
    reg [31:0] instr_mem [0:15];

    // Clock generation: 10ns period
    always #5 clk = ~clk;

    // Initialize instruction memory
    initial begin
        // addi x1, x0, 5
        instr_mem[0] = 32'h00500093;
        // addi x2, x1, 3
        instr_mem[1] = 32'h00308113;
        // Fill rest with NOPs
        for (i = 2; i < 16; i = i + 1) 
            instr_mem[i] = 32'h00000013;
    end

    // Drive instruction input from memory (safe combinational ROM)
    always @(*) begin
        instr_in = instr_mem[pc_out[5:2]];
    end

    // Optional VCD dump for waveform debugging (works with Icarus)
    initial begin
        $dumpfile("datapath_tb.vcd");
        $dumpvars(0, tb_datapath);
    end

    // Test sequence
    initial begin
        $display("Starting datapath testbench...");
        $display("Cycle PC     Op  Rs1 Rs2 Rd_ex Rd_mem Rd_wb  ALU_Result    FwdA FwdB");
        
        // Reset sequence
        rst = 1;
        repeat(2) @(posedge clk);
        rst = 0;

        // Run for enough cycles to see results
        for (cycle = 0; cycle < 12; cycle = cycle + 1) begin
            @(posedge clk);
            
            // Print pipeline state including forwarding and ALU result
            $display("%4d  %08h %02h %2d  %2d   %2d    %2d     %2d    ALU:%08h FwdA:%b FwdB:%b A_in:%08h A_ex:%08h", 
                cycle, pc_out, opcode_out,
                ID_rs1_addr_out, ID_rs2_addr_out,
                EX_rd_addr_out, MEM_rd_addr_out, WB_rd_addr_out,
                mem_addr_out, // ALU (from EX/MEM)
                dbg_forward_a_sel, dbg_forward_b_sel,
                dbg_operand_a_ex, dbg_alu_result_ex
            );

            // Check registers at specific cycles (after writes should have occurred)
            case (cycle)
                5: begin
                    $display("-- Checking register x1 at cycle %0d --", cycle);
                    check_register_state(1, 5);  // First ADDI should complete
                end
                7: begin
                    $display("-- Checking register x2 at cycle %0d --", cycle);
                    check_register_state(2, 8);  // Second ADDI should complete (5+3)
                end
                10: begin
                    $display("\nFinal Register State:");
                    check_register_state(1, 5);
                    check_register_state(2, 8);
                end
            endcase

            // Demo forwarding when needed: enable forwarding from EX/MEM before dependent EX stage
            if (cycle == 2) begin
                ForwardA_Sel = 2'b01;  // Forward from EX/MEM stage (matches forwarding_mux encoding)
                $display("Enabling EX/MEM->EX forwarding for x1 at cycle %0d", cycle);
            end
            if (cycle == 4) begin
                ForwardA_Sel = 2'b00;  // Reset forwarding
                $display("Disabling forwarding at cycle %0d", cycle);
            end
        end

        $display("\nTestbench completed");
        $finish;
    end

    // Timeout watchdog
    initial begin
        #5000; // 5000ns timeout to be safe
        $display("Timeout - simulation took too long!");
        $finish;
    end

    // Monitor decoded instructions
    always @(instr_in) begin
        case (instr_in[6:0])
            7'b0010011: begin // ADDI
                $display("Decoded ADDI rd=x%0d, rs1=x%0d, imm=%0d", 
                    instr_in[11:7],  // rd
                    instr_in[19:15], // rs1
                    {{20{instr_in[31]}}, instr_in[31:20]} // immediate
                );
            end
            default: ;
        endcase
    end

endmodule
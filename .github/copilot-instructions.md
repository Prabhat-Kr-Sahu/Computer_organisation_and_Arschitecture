## Quick start for AI coding agents

This repository implements a pipelined RISC-style processor in Verilog. The goal of this document is to give an AI coding agent the minimal, high-value context needed to make correct, low-risk edits.

- Entry point / top-level: `processor_top.v` (connects datapath, controller, memory, and pipeline registers).
- Pipeline stages and registers: IF/ID/EX/MEM/WB are implemented across `pipeline_register_if_id.v`, `pipeline_register_id_ex.v`, `pipeline_register_ex_mem.v`, `pipeline_register_mem_wb.v` and the stage modules (`datapath.v`, `controller.v`).
- PC & fetch: `pc_update.v` implements PC update policy. Important example: `PCSrc` encodings are 2'b00 = PC+4, 2'b01 = Branch/JAL, 2'b10 = JALR; `S"tall` holds PC when high.
- Hazarding & forwarding: `hazard_unit.v` and `forwarding_mux_a.v` / `forwarding_mux_b.v` handle pipeline hazards and forwarding decisions. When adding/removing bypass paths, update both the hazard unit and the forwarding mux sources.
- Execution units: `alu.v`, `multiplier.v` (special ops). Memory interfaces: `instruction_memory.v`, `data_memory.v`.
- Register writeback: `writeback_mux.v` and `register_file.v` control results written to registers.

Design and change guidance (concrete):

- When modifying control signals (branch/jump behavior): update `main_decoder.v` / `controller.v`, adjust pipeline register port lists (`pipeline_register_*.v`) to carry any new control bits, and update `pc_update.v` to interpret new `PCSrc`/control semantics.
- When changing data-path widths or ALU behavior: update `datapath.v`, `alu.v`, and ensure `register_file.v` and memory modules use the same widths.
- To add a new pipeline-forwarding or hazard rule: modify `hazard_unit.v` and the appropriate `forwarding_mux_*.v` files together. Tests/verification should focus on the hazard cases (load-use, branch interlocks, JALR).

Signals & naming conventions observed:

- Lowercase with underscores for signals and filenames (e.g., `branch_target`, `pc_plus_4`).
- 32-bit datapath: most data, PC and ALU buses are 32-bit wide.
- Modules expose `clk` and `rst` where appropriate; synchronous logic uses posedge clock and posedge rst.

Build / simulation notes (repository inspection):

- No repository-level build scripts (Makefile, scripts, or CI configs) were found. Typical developer flows used with this codebase are:
  - Vendor tool: Vivado/ISE project for synthesis and implementation (not included in repo).
  - Simulation: run a Verilog simulator (Icarus Verilog, ModelSim) over the `.v` files and a testbench (not shipped here).

What to inspect first for common tasks:

- Fix branch target/PC bugs: `pc_update.v`, `datapath.v`, `hazard_unit.v`.
- Debug wrong register values: `register_file.v`, `writeback_mux.v`, and check pipeline registers for control bit propagation.
- Add instruction or ALU op: `datapath.v` + `alu.v` + update `main_decoder.v`/`controller.v` to set control signals.

Small contract for changes an AI should follow

- Inputs: change descriptions should include the target behavior and failing test vectors or waveform snippets.
- Outputs: produce edits touching the minimal set of files (decoder/controller, datapath, pipeline registers, and testbench) and a short rationale comment in the commit.
- Error modes: avoid breaking existing port lists; if port order changes, update every instantiation.

If anything here is unclear or you'd like more detail (example testbenches to add, preferred simulator, or CI integration), say which area to expand and I will iterate.

iverilog -o testCPU_Pipeline.vvp controller.v data_mem.v instr_mem.v pc.v rf_pipelined.v forwardingUnit.v hazardDetection.v sign_extend* ALU/*.v Flops/*.v cpu.v t_CPU.tb

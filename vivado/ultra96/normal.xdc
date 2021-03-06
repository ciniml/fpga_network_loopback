set_false_path -from [get_pins {Top_i/VexRiscv_inst/inst/jtagBridge_1/system_rsp_payload_data_reg[*]/C}] -to [get_pins {Top_i/VexRiscv_inst/inst/jtagBridge_1/jtag_readArea_full_shifter_reg[*]/D}]
set_false_path -from [get_pins Top_i/VexRiscv_inst/inst/jtagBridge_1/flowCCByToggle_1/inputArea_data_fragment_reg*/C] -to [get_pins {Top_i/VexRiscv_inst/inst/jtagBridge_1/flowCCByToggle_1/outputArea_flow_m2sPipe_payload_fragment_reg[*]/D}]
set_false_path -from [get_pins Top_i/VexRiscv_inst/inst/jtagBridge_1/flowCCByToggle_1/inputArea_target_reg*/C] -to [get_pins Top_i/VexRiscv_inst/inst/jtagBridge_1/flowCCByToggle_1/inputArea_target_buffercc/buffers_0_reg/D]
set_false_path -from [get_pins Top_i/VexRiscv_inst/inst/jtagBridge_1/flowCCByToggle_1/inputArea_data_last_reg/C] -to [get_pins Top_i/VexRiscv_inst/inst/jtagBridge_1/flowCCByToggle_1/outputArea_flow_m2sPipe_payload_last_reg/D]
set_false_path -from [get_pins Top_i/VexRiscv_inst/inst/jtagBridge_1/system_rsp_payload_error_reg/C] -to [get_pins {Top_i/VexRiscv_inst/inst/jtagBridge_1/jtag_readArea_full_shifter_reg[1]/D}]
set_false_path -from [get_pins Top_i/VexRiscv_inst/inst/jtagBridge_1/system_rsp_valid_reg/C] -to [get_pins {Top_i/VexRiscv_inst/inst/jtagBridge_1/jtag_readArea_full_shifter_reg[0]/D}]

set_property IOSTANDARD LVCMOS18 [get_ports DRAM_0_STAT_CATTRIP_0]
set_property PACKAGE_PIN J18 [get_ports DRAM_0_STAT_CATTRIP_0]
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk]

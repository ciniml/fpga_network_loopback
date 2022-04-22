.PHONY: all clean restore device-tree open

PROJECT_NAME := fpga_network_loopback
PROJECT_DIR := project
BD_NAME := design_top
BITSTREAM := $(PROJECT_DIR)/$(PROJECT_NAME).runs/impl_1/$(BD_NAME)_wrapper.bit
VIVADO ?= vivado

all: $(BITSTREAM)

restore: $(PROJECT_DIR)/$(PROJECT_NAME).xpr

clean:
	-@$(RM) -rf $(PROJECT_DIR) .Xil NA kernel.tmp kernel
	-@$(RM) *.jou *.str *.btree *.log

open: $(PROJECT_DIR)/$(PROJECT_NAME).xpr
	$(VIVADO) $<&

$(PROJECT_DIR)/$(PROJECT_NAME).xpr:
	$(VIVADO) -mode batch -source restore_project.tcl -tclargs $(PROJECT_NAME)

$(BITSTREAM): $(PROJECT_DIR)/$(PROJECT_NAME).xpr $(SRCS) $(PROJECT_DIR)/$(PROJECT_NAME).srcs/sources_1/bd/$(BD_NAME)/$(BD_NAME).bd
	$(VIVADO) -mode batch -source implement.tcl -tclargs $(PROJECT_NAME) $(BD_NAME)

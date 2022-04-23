.PHONY: all clean restore device-tree open overlay deploy

PROJECT_NAME := fpga_network_loopback
PROJECT_DIR := project
BD_NAME := design_top
BITSTREAM := $(PROJECT_DIR)/$(PROJECT_NAME).runs/impl_1/$(BD_NAME)_wrapper.bit
BITSTREAM_BIN := $(BD_NAME)_wrapper.bit.bin
VIVADO ?= vivado
BOOTGEN ?= bootgen
DEVICE_USER ?= fpga
DEVICE_IP ?= 192.168.2.18
DEVICE_DIR ?= /home/fpga/fpga_network_loopback
OVERLAY_DIR ?= /config/device-tree/overlays/fpga

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

$(BITSTREAM_BIN): $(BITSTREAM)
	$(BOOTGEN) -image ./bitstream_to_bin.bif -arch zynqmp -w -o $@

overlay.dtbo: overlay.dts
	dtc -I dts -O dtb -@ overlay.dts -o $@

device-tree: overlay.dtbo

overlay: overlay.dtbo $(BITSTREAM_BIN)

deploy: overlay.dtbo $(BITSTREAM_BIN)
	ssh $(DEVICE_USER)@$(DEVICE_IP) mkdir -p $(DEVICE_DIR)
	scp overlay.dtbo $(DEVICE_USER)@$(DEVICE_IP):$(DEVICE_DIR)/
	scp $(BITSTREAM_BIN) $(DEVICE_USER)@$(DEVICE_IP):$(DEVICE_DIR)/
	ssh $(DEVICE_USER)@$(DEVICE_IP) sudo sh -c "\"mkdir -p $(OVERLAY_DIR)\""
	ssh $(DEVICE_USER)@$(DEVICE_IP) sudo sh -c "\"cp $(DEVICE_DIR)/$(BITSTREAM_BIN) /lib/firmware\""
	ssh $(DEVICE_USER)@$(DEVICE_IP) sudo sh -c "\"cp $(DEVICE_DIR)/overlay.dtbo $(OVERLAY_DIR)/dtbo\""
	ssh $(DEVICE_USER)@$(DEVICE_IP) sh -c "dmesg | tail -n 20"
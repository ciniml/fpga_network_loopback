.PHONY: all clean restore device-tree open overlay deploy

all: ip/ethernet_service.zip

clean:
	-@$(RM) -rf $(PROJECT_DIR) .Xil NA kernel.tmp kernel
	-@$(RM) *.jou *.str *.btree *.log


ip/ethernet_service.zip:
	vitis_hls ./build.tcl
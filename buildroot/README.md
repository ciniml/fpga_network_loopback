# Buildroot external tree to build xgmac kernel module

## How to use

Configure the buildroot to use this external tree.

Run the commands below at the root of the buildroot. (Replace the `path to this repository` to the path to this repository root.)

Then select `External options->xgmac kernel_module` and save the buildroot configuration. 

```
export FPGA_NETWORK_LOOPBACK_PATH=(path to this repository)
make BR2_EXTERNAL=`realpath $FPGA_NETWORK_LOOPBACK_PATH/buildroot/xgmac` menuconfig 
```

Build the kernel and rootfs.

```
make
```

You can see the xgmac kernel module `xgmac.ko` in `/lib/modules/(kernel version)/extra`
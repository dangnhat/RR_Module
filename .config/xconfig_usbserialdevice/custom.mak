## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,e430X linker.cmd package/cfg/usbserialdevice_pe430X.oe430X

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/usbserialdevice_pe430X.xdl
	$(SED) 's"^\"\(package/cfg/usbserialdevice_pe430Xcfg.cmd\)\"$""\"C:/Users/nhatpham/Desktop/msp430_workspace/usbserialdevice_MSP_EXP430F5529LP_TI_MSP430F5529/.config/xconfig_usbserialdevice/\1\""' package/cfg/usbserialdevice_pe430X.xdl > $@
	-$(SETDATE) -r:max package/cfg/usbserialdevice_pe430X.h compiler.opt compiler.opt.defs

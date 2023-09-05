cti-dts-path := platform/t19x/jakku/kernel-dts/cti

DTC_INCLUDE += $(tegra-dtstree)/$(cti-dts-path)

cti-dtb-y += tegra194-xavier-nx-cti-NGX003.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX003-RPI-IMX219-2CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX003-ARDU-IMX477-2CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX004.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX004-RPI-IMX219-2CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX004-ARDU-IMX477-2CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX006.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX007.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX007-FSM-IMX290-3CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX007-FSM-IMX290-4CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX007-FSM-IMX296-4CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX007-FSM-IMX304-3CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX007-FSM-IMX412-3CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX007-FSM-IMX412-4CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX007-FSM-IMX415-3CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX007-FSM-IMX415-4CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX007-FSM-IMX464-3CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX007-FSM-IMX464-4CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX007-FSM-IMX530-3CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX007-FSM-IMX565-3CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX007-FSM-IMX565-4CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX007-FSM-IMX585-3CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX007-FSM-IMX585-4CAM.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX012.dtb
cti-dtb-y += tegra194-xavier-nx-cti-NGX012-RPI-IMX219-1CAM.dtb

#prefix the cti dts path to all the dtb/dtbo files defined in
#cti-dtb-y and cti-dtbo-y so the kernel knows where to find them
ifneq ($(cti-dtb-y),)
cti-dtb-y := $(addprefix $(cti-dts-path)/,$(cti-dtb-y))
endif
ifneq ($(cti-dtbo-y),)
cti-dtbo-y := $(addprefix $(cti-dts-path)/,$(cti-dtbo-y))
endif

#add the cti-dtbs/cti-dtbos to the kernel's build list
dtb-y += $(cti-dtb-y)
dtbo-y += $(cti-dtbo-y)


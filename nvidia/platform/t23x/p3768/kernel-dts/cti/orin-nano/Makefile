
#define cti-dts-path relative to <sources>/hardware/nvidia
cti-dts-path := platform/t23x/p3768/kernel-dts/cti/orin-nano

DTC_INCLUDE += $(tegra-dtstree)/$(cti-dts-path)

cti-dtb-y :=
#cti-dtbo-y :=

#CTI DTB Files

#CTI Nano dtbs
cti-dtb-y += tegra234-orin-nano-cti-NGX003.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX003-RPI-IMX219-2CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007-FSM-IMX290-2CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007-FSM-IMX290-4CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007-FSM-IMX296-4CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007-FSM-IMX304-2CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007-FSM-IMX412-2CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007-FSM-IMX412-4CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007-FSM-IMX415-2CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007-FSM-IMX415-4CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007-FSM-IMX464-2CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007-FSM-IMX464-4CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007-FSM-IMX565-2CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007-FSM-IMX565-4CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007-FSM-IMX585-2CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007-FSM-IMX585-4CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007-FSM-IMX678-2CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX007-FSM-IMX678-4CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX012.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX012-RPI-IMX219-1CAM.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX015.dtb
cti-dtb-y += tegra234-orin-nano-cti-NGX015-ECON-NILECAM21-4CAM.dtb

#CTI Overlays
#cti-dtbo-y += cti_overlay/tegra234-orin-agx-cti-overlay-pcie-ep.dtbo


#prefix the cti dts path to all the dtb/dtbo files defined in 
#cti-dtb-y and cti-dtbo-y so the kernel knows where to find them
ifneq ($(cti-dtb-y),)
cti-dtb-y := $(addprefix $(cti-dts-path)/,$(cti-dtb-y))
endif
#ifneq ($(cti-dtbo-y),)
#cti-dtbo-y := $(addprefix $(cti-dts-path)/,$(cti-dtbo-y))
#endif

#add the cti-dtbs/cti-dtbos to the kernel's build list
dtb-y += $(cti-dtb-y)
#dtbo-y += $(cti-dtbo-y)

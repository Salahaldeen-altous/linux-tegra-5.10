
#define cti-dts-path relative to <sources>/hardware/nvidia
cti-dts-path := platform/t23x/concord/kernel-dts/cti

DTC_INCLUDE += $(tegra-dtstree)/$(cti-dts-path)

cti-dtb-y :=
cti-dtbo-y :=

#CTI DTB Files
cti-dtb-y += tegra234-orin-agx-cti-AGX201.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX201-FSM-IMX462.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX201-ECON-e-CAM82.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX201-JCB002-ECON-NILECAM21.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX201-JCB002-ISX021.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX201-JCB002-C2.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX201-JCB002-C2-C1-MIXNMATCH.dtb 
cti-dtb-y += tegra234-orin-agx-cti-AGX201-JCB002-ECON-NILECAM82.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX201-JCB002-LI-HAWK.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX201-JCB002-SL-ZEDX.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX201-JCB006-ECON-NEDUCAM25.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX201-JCB006-AVT-CSI2-8CAM.dtb

cti-dtb-y += tegra234-orin-agx-cti-AGX202.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX202-FSM-IMX462.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX202-ECON-e-CAM82.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX202-JCB002-ECON-NILECAM21.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX202-JCB002-ECON-NILECAM82.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX202-JCB002-ISX021.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX202-JCB002-C2.dtb 
cti-dtb-y += tegra234-orin-agx-cti-AGX202-JCB002-C2-C1-MIXNMATCH.dtb 
cti-dtb-y += tegra234-orin-agx-cti-AGX202-JCB005-AVT-CSI2-4CAM.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX202-JCB005-AVT-CSI2-6CAM.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX202-JCB006-AVT-CSI2-8CAM.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX202-JCB002-LI-HAWK.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX202-JCB002-SL-ZEDX.dtb
cti-dtb-y += tegra234-orin-agx-cti-AGX202-JCB006-ECON-NEDUCAM25.dtb
#CTI Overlays
cti-dtbo-y += cti_overlay/tegra234-orin-agx-cti-overlay-pcie-ep.dtbo


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



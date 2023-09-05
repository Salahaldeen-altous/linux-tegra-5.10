
#define cti-custom-dts-path relative to <sources>/hardware/nvidia
cti-custom-dts-path := platform/t23x/concord/kernel-dts/cti-custom

#include the standard cti dts path 
DTC_INCLUDE += $(tegra-dtstree)/$(cti-dts-path)

cti-custom-dtb-y :=
cti-custom-dtbo-y :=

#CTI Custom DTB Files
cti-custom-dtb-y += tegra234-orin-agx-cti-AGX920.dtb

cti-custom-dtb-y += tegra234-orin-agx-cti-AGX922.dtb
cti-custom-dtb-y += tegra234-orin-agx-cti-AGX922-usb5Gbps.dtb
cti-custom-dtb-y += tegra234-orin-agx-cti-AGX922-dual-pciex4.dtb
cti-custom-dtb-y += tegra234-orin-agx-cti-AGX922-dual-pciex4-usb5Gbps.dtb

cti-custom-dtb-y += tegra234-orin-agx-cti-AGX922-mcp.dtb
#CTI Custom Overlays


#prefix the cti dts path to all the dtb/dtbo files defined in 
#cti-custom-dtb-y and cti-custom-dtbo-y so the kernel knows where to find them
ifneq ($(cti-custom-dtb-y),)
cti-custom-dtb-y := $(addprefix $(cti-custom-dts-path)/,$(cti-custom-dtb-y))
endif
ifneq ($(cti-custom-dtbo-y),)
cti-custom-dtbo-y := $(addprefix $(cti-custom-dts-path)/,$(cti-custom-dtbo-y))
endif

#add the cti-dtbs/cti-dtbos to the kernel's build list
dtb-y += $(cti-custom-dtb-y)
dtbo-y += $(cti-custom-dtbo-y)

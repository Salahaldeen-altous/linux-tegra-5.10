
#define cti-testing-dts-path relative to <sources>/hardware/nvidia
cti-testing-dts-path := platform/t23x/concord/kernel-dts/cti-testing

#include the standard cti dts path 
DTC_INCLUDE += $(tegra-dtstree)/$(cti-dts-path)

cti-testing-dtb-y :=
cti-testing-dtbo-y :=

#CTI Custom DTB Files
cti-testing-dtb-y += tegra234-orin-agx-cti-UAGX-AS-internal.dtb

#CTI Custom Overlays


#prefix the cti dts path to all the dtb/dtbo files defined in 
#cti-testing-dtb-y and cti-testing-dtbo-y so the kernel knows where to find them
ifneq ($(cti-testing-dtb-y),)
cti-testing-dtb-y := $(addprefix $(cti-testing-dts-path)/,$(cti-testing-dtb-y))
endif
ifneq ($(cti-testing-dtbo-y),)
cti-testing-dtbo-y := $(addprefix $(cti-testing-dts-path)/,$(cti-testing-dtbo-y))
endif

#add the cti-dtbs/cti-dtbos to the kernel's build list
dtb-y += $(cti-testing-dtb-y)
dtbo-y += $(cti-testing-dtbo-y)

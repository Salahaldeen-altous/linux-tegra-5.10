old-dtb := $(dtb-y)
old-dtbo := $(dtbo-y)
dtb-y :=
dtbo-y :=
makefile-path := platform/t19x/galen-industrial/kernel-dts

BUILD_19x_ENABLE=n
BUILD_18x_ENABLE=n
ifneq ($(filter y,$(CONFIG_ARCH_TEGRA_19x_SOC) $(CONFIG_ARCH_TEGRA_194_SOC)),)
BUILD_19x_ENABLE=y
endif
ifneq ($(filter y,$(CONFIG_ARCH_TEGRA_18x_SOC) $(CONFIG_ARCH_TEGRA_186_SOC)),)
BUILD_18x_ENABLE=y
endif

dtb-$(BUILD_19x_ENABLE) += tegra194-p2888-0008-e3366-1199.dtb
dtb-$(BUILD_19x_ENABLE) += tegra194-p2888-0008-p2822-0000.dtb
dtb-$(BUILD_19x_ENABLE) += tegra194-p2888-0008-p2822-0000-maxn.dtb
dtb-$(BUILD_19x_ENABLE) += tegra194-p2888-0008-p2822-0000-noecc.dtb
dtb-$(BUILD_19x_ENABLE) += tegra194-p2888-0008-p2822-0000-safety.dtb
dtbo-$(BUILD_19x_ENABLE) += tegra194-p2888-0008-p2822-0000-overlay.dtbo

#CTI Device Trees
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX101.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX101-FSM-IMX290-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX101-FSM-IMX296-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX101-FSM-IMX412-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX101-FSM-IMX415-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX101-FSM-IMX464-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX101-FSM-IMX565-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX101-FSM-IMX585-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX101-JCB005-AVT-CSI2-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX101-JCB005-AVT-CSI2-6CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX101-JCB006-AVT-CSI2-8CAM.dtb

dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX103.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX103-JCB005-AVT-CSI2-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX103-JCB005-AVT-CSI2-6CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX103-JCB006-AVT-CSI2-8CAM.dtb

dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX104.dtb

dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX105.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX105-JCB005-AVT-CSI2-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX105-JCB005-AVT-CSI2-6CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX105-JCB006-AVT-CSI2-8CAM.dtb

dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX108.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX108-FSM-IMX290-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX108-FSM-IMX296-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX108-FSM-IMX412-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX108-FSM-IMX415-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX108-FSM-IMX464-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX108-FSM-IMX565-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX108-FSM-IMX585-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX108-JCB005-AVT-CSI2-4CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX108-JCB005-AVT-CSI2-6CAM.dtb
dtb-$(BUILD_19x_ENABLE) += cti/tegra194-agxi-cti-AGX108-JCB006-AVT-CSI2-8CAM.dtb



ifneq ($(dtb-y),)
dtb-y := $(addprefix $(makefile-path)/,$(dtb-y))
endif
ifneq ($(dtbo-y),)
dtbo-y := $(addprefix $(makefile-path)/,$(dtbo-y))
endif

dtb-y += $(old-dtb)
dtbo-y += $(old-dtbo)

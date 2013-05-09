obj-$(CONFIG_POWER_AVS_OMAP)		+= smartreflex.o

ifneq ($(CONFIG_POWER_TI_HARDWARE_VOLTAGE_CONTROL),)

# OMAP Common
omap-volt-common			=  omap_vc.o omap_vp.o

# OMAP SoC specific
ifneq ($(CONFIG_ARCH_OMAP3),)
obj-$(CONFIG_POWER_TI_HARDWARE_VOLTAGE_CONTROL) += $(omap-volt-common)
endif

ifneq ($(CONFIG_ARCH_OMAP4),)
obj-$(CONFIG_POWER_TI_HARDWARE_VOLTAGE_CONTROL) += $(omap-volt-common)
endif

ifneq ($(CONFIG_SOC_OMAP5),)
obj-$(CONFIG_POWER_TI_HARDWARE_VOLTAGE_CONTROL) += $(omap-volt-common)
endif

endif

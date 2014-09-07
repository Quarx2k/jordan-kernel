#
# Makefile for the linux kernel.
#

# Common support
obj-y := id.o io.o control.o mux.o devices.o serial.o gpmc.o timer-gp.o

omap-2-3-common				= irq.o sdrc.o omap_hwmod.o \
					  omap_hwmod_common_data.o
prcm-common				= prcm.o powerdomain.o
clock-common				= clock.o clockdomain.o

obj-$(CONFIG_ARCH_OMAP2) += $(omap-2-3-common) $(prcm-common) $(clock-common)
obj-$(CONFIG_ARCH_OMAP3) += $(omap-2-3-common) $(prcm-common) $(clock-common)

obj-$(CONFIG_OMAP_MCBSP) += mcbsp.o
obj-$(CONFIG_ION_OMAP)			+= omap_ion.o
# SMP support ONLY available for OMAP4
obj-$(CONFIG_SMP)			+= omap-smp.o omap-headsmp.o
obj-$(CONFIG_LOCAL_TIMERS)		+= timer-mpu.o

# Functions loaded to SRAM
obj-$(CONFIG_ARCH_OMAP2420)		+= sram242x.o
obj-$(CONFIG_ARCH_OMAP2430)		+= sram243x.o
obj-$(CONFIG_ARCH_OMAP3)		+= sram34xx.o

# SMS/SDRC
obj-$(CONFIG_ARCH_OMAP2)		+= sdrc2xxx.o
# obj-$(CONFIG_ARCH_OMAP3)		+= sdrc3xxx.o

# Power Management
ifeq ($(CONFIG_PM),y)
obj-$(CONFIG_ARCH_OMAP2)		+= pm24xx.o
obj-$(CONFIG_ARCH_OMAP24XX)		+= sleep24xx.o
obj-$(CONFIG_ARCH_OMAP3)		+= pm.o pm34xx.o sleep34xx.o cpuidle34xx.o
obj-$(CONFIG_PM_DEBUG)			+= pm-debug.o
obj-y					+= wakeup-timer.o
obj-$(CONFIG_OMAP_SMARTREFLEX)		+= smartreflex.o
endif

# PRCM
obj-$(CONFIG_ARCH_OMAP2)		+= cm.o
obj-$(CONFIG_ARCH_OMAP3)		+= cm.o
obj-$(CONFIG_ARCH_OMAP4)		+= cm4xxx.o

# Clock framework
obj-$(CONFIG_ARCH_OMAP2)		+= clock24xx.o
obj-$(CONFIG_ARCH_OMAP3)		+= clock34xx.o
obj-$(CONFIG_OMAP_PM_SRF)		+=  resource34xx.o

obj-$(CONFIG_OMAP_MBOX_FWK)		+= mailbox_mach.o
mailbox_mach-objs			:= mailbox.o

# EMU peripherals
obj-$(CONFIG_OMAP3_EMU)		+= emu.o

iommu-y					+= iommu2.o
iommu-$(CONFIG_ARCH_OMAP3)		+= omap3-iommu.o

obj-$(CONFIG_OMAP_IOMMU)		+= $(iommu-y)

# Debobs
obj-$(CONFIG_OMAP3_DEBOBS)	+= debobs.o

ifneq ($(CONFIG_MPU_BRIDGE),)
obj-y                                  += dspbridge.o
endif

# Specific board support
obj-$(CONFIG_MACH_MAPPHONE)             += board-mapphone.o \
                                           board-mapphone-camera.o \
                                           board-mapphone-panel.o \
                                           board-mapphone-spi.o \
                                           board-mapphone-cpcap-client.o \
                                           board-mapphone-sensors.o \
                                           board-mapphone-hsmmc.o \
                                           board-mapphone-keypad.o \
                                           board-mapphone-wifi.o \
                                           board-mapphone-gpio.o \
                                           board-mapphone-usb.o \
					   board-mapphone-padconf.o

obj-$(CONFIG_EMU_UART_DEBUG)		+= board-mapphone-emu_uart.o

# Platform specific device init code
obj-y					+= usb-musb.o
obj-$(CONFIG_MACH_OMAP2_TUSB6010)	+= usb-tusb6010.o
obj-y					+= usb-ehci.o

onenand-$(CONFIG_MTD_ONENAND_OMAP2)	:= gpmc-onenand.o
obj-y					+= $(onenand-m) $(onenand-y)

smc91x-$(CONFIG_SMC91X)			:= gpmc-smc91x.o
obj-y					+= $(smc91x-m) $(smc91x-y)

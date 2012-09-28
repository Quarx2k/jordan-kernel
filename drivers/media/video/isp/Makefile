# Makefile for OMAP3 ISP driver

isp-mod-objs += \
	isp.o ispccdc.o ispmmu.o \
	isppreview.o ispresizer.o ispcsi2.o

ifndef CONFIG_VIDEO_OMAP3_HP3A
isp-mod-objs += \
	isph3a.o isphist.o isp_af.o
else
isp-mod-objs += \
	isp_mem_process.o
endif

obj-$(CONFIG_VIDEO_OMAP34XX_ISP_PREVIEWER) += \
	omap_previewer.o

obj-$(CONFIG_VIDEO_OMAP34XX_ISP_RESIZER) += \
	omap_resizer.o

obj-$(CONFIG_VIDEO_OMAP3) += isp-mod.o

ifdef CONFIG_VIDEO_OMAP3_HP3A
EXTRA_CFLAGS += -Idrivers/media/video/hp3a
endif
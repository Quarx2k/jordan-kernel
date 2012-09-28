# Makefile for OMAP3 ISP driver

ifdef CONFIG_ARCH_OMAP3410
isp-mod-objs += \
	isp.o ispccdc.o ispmmu.o
else
isp-mod-objs += \
	isp.o ispccdc.o ispmmu.o \
	isppreview.o ispresizer.o ispcsi2.o

ifndef CONFIG_VIDEO_OMAP3_HP3A
isp-mod-objs += \
	isph3a.o isphist.o isp_af.o 
endif

obj-$(CONFIG_VIDEO_OLDOMAP34XX_ISP_PREVIEWER) += \
	omap_previewer.o

obj-$(CONFIG_VIDEO_OLDOMAP34XX_ISP_RESIZER) += \
	omap_resizer.o

endif

obj-$(CONFIG_VIDEO_OLDOMAP3) += isp-mod.o

ifdef CONFIG_VIDEO_OMAP3_HP3A
EXTRA_CFLAGS += -Idrivers/media/video/hp3a
endif

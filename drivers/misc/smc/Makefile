ifeq ($(CONFIG_SMC_DRIVER_DEBUG_SUPPORT),y)
	EXTRA_CFLAGS += -O0 -DDEBUG -D_DEBUG
else
	EXTRA_CFLAGS += -DNDEBUG
endif
EXTRA_CFLAGS += -DLINUX
EXTRA_CFLAGS += -Wundef
EXTRA_CFLAGS += -Wno-unused-parameter -Wsign-compare -Wstrict-prototypes -fno-strict-aliasing
EXTRA_CFLAGS += -DSMODULE_SMC_OMAP3XXX_POWER_MANAGEMENT

ifdef S_VERSION_BUILD
EXTRA_CFLAGS += -DS_VERSION_BUILD=$(S_VERSION_BUILD)
endif

obj-$(CONFIG_SMC_DRIVER)	+= bridge_pub2sec.o scxlnx_conn.o scxlnx_device.o scxlnx_sm_comm.o scxlnx_util.o scx_public_crypto_AES.o scx_public_crypto.o scx_public_crypto_DES.o scx_public_crypto_Digest.o scx_public_dma.o

#Android makefile to build kernel as a part of Android Build
ifneq ($(BUILD_KERNEL),)

KERNEL_SRCDIR := kernel/omap-moto-cw
KERNEL_OUT := $(ANDROID_PRODUCT_OUT)/obj/KERNEL_OBJ
KERNEL_CONFIG := $(KERNEL_OUT)/.config
TARGET_PREBUILT_INT_KERNEL := $(KERNEL_OUT)/arch/arm/boot/zImage-dtb
KERNEL_HEADERS_INSTALL := $(KERNEL_OUT)/usr
KERNEL_MODULES_INSTALL := system
KERNEL_MODULES_OUT := $(TARGET_OUT)/lib/modules
KERNEL_IMG=$(KERNEL_OUT)/arch/arm/boot/Image
# relative path from KERNEL_OUT to kernel source directory
KERNEL_SOURCE_RELATIVE_PATH := ../../../../../../$(KERNEL_SRCDIR)
PRODUCT_PREBUILT_KERNEL := $(TARGET_PREBUILT_KERNEL)

ifeq ($(TARGET_USES_UNCOMPRESSED_KERNEL),true)
$(error TARGET_USES_UNCOMPRESSED_KERNEL is not supported)
else
TARGET_PREBUILT_KERNEL := $(TARGET_PREBUILT_INT_KERNEL)
endif

# use eng_ defconfig for engineering Android variants
ifeq ($(TARGET_BUILD_VARIANT), eng)
KERNEL_DEFCONFIG := eng_${KERNEL_DEFCONFIG}
endif
SOURCE_DEFCONFIG  := $(KERNEL_SRCDIR)/arch/arm/configs/$(KERNEL_DEFCONFIG)

define mv-modules
mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.order`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`;\
ko=`find $$mpath/kernel -type f -name *.ko`;\
for i in $$ko; do mv $$i $(KERNEL_MODULES_OUT)/; done;\
fi
endef

define clean-module-folder
mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.order`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`; rm -rf $$mpath;\
fi
endef

define update-prebuilts
if [ -f $(TARGET_PREBUILT_INT_KERNEL) -a\
     -f $(PRODUCT_PREBUILT_KERNEL) ]; then\
  cp -f $(TARGET_PREBUILT_INT_KERNEL) $(PRODUCT_PREBUILT_KERNEL);\
fi;
endef

define do-kernel-config
	( cp $(3) $(2) && $(7) -C $(4) O=$(1) ARCH=$(5) CROSS_COMPILE=$(6) oldconfig ) || ( rm -f $(2) && false )
endef

GIT_HOOKS_DIR := $(KERNEL_SRCDIR)/.git/hooks
inst_hook: $(GIT_HOOKS_DIR)/pre-commit $(GIT_HOOKS_DIR)/checkpatch.pl

$(GIT_HOOKS_DIR)/pre-commit:  $(KERNEL_SRCDIR)/scripts/pre-commit
	@-cp -f $< $@
	@-chmod ugo+x $@

$(GIT_HOOKS_DIR)/checkpatch.pl:  $(KERNEL_SRCDIR)/scripts/checkpatch.pl
	@-cp -f $< $@
	@-chmod ugo+x $@

$(KERNEL_OUT):
	mkdir -p $(KERNEL_OUT)

$(KERNEL_CONFIG): $(KERNEL_OUT) $(SOURCE_DEFCONFIG) inst_hook
	$(call do-kernel-config,$(KERNEL_OUT),$@,$(SOURCE_DEFCONFIG),$(KERNEL_SRCDIR),arm,arm-eabi-,$(MAKE))

$(TARGET_PREBUILT_INT_KERNEL): $(KERNEL_OUT) $(KERNEL_CONFIG) $(KERNEL_HEADERS_INSTALL) $(TARGET_PREBUILT_INT_DTB)
	$(MAKE) -C $(KERNEL_SRCDIR) KBUILD_RELSRC=$(KERNEL_SOURCE_RELATIVE_PATH) O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi-
	$(update-prebuilts)

$(KERNEL_HEADERS_INSTALL): $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C $(KERNEL_SRCDIR) O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- headers_install

kerneltags: $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C $(KERNEL_SRCDIR) O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- tags

kernelconfig: $(KERNEL_OUT) $(KERNEL_CONFIG)
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C $(KERNEL_SRCDIR) O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- menuconfig
	cp $(KERNEL_OUT)/.config $(SOURCE_DEFCONFIG)

endif

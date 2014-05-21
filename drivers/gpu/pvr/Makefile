#
ccflags-y = -Idrivers/video/omap2 \
	-Idrivers/gpu/pvr/include4\
	-Idrivers/gpu/pvr/services4/include \
	-Idrivers/gpu/pvr/services4/srvkm/hwdefs \
	-Idrivers/gpu/pvr/services4/srvkm/include \
	-Idrivers/gpu/pvr/services4/srvkm/bridged \
	-Idrivers/gpu/pvr/services4/srvkm/bridged/sgx \
	-Idrivers/gpu/pvr/services4/srvkm/devices/sgx \
	-Idrivers/gpu/pvr/services4/srvkm/env/linux \
	-Idrivers/gpu/pvr/services4/system/include \
	-DLINUX -D__linux__ \
	-DANDROID \
	-DPVR_BUILD_DIR="\"omap3630_android\"" \
	-DSUPPORT_SGX \
	-DTRANSFER_QUEUE \
	-DPVR_SECURE_HANDLES \
	-DSUPPORT_LISR_MISR_SYNC \
	-DDISPLAY_CONTROLLER=omaplfb \
	-DPVR_LINUX_MEM_AREA_POOL_MAX_PAGES=10800 \
	-DPVR_LINUX_MEM_AREA_USE_VMAP \
	-DPVR_LINUX_MEM_AREA_POOL_ALLOW_SHRINK \
	-DSUPPORT_PERCONTEXT_PB \
	-DSUPPORT_HW_RECOVERY \
	-DSUPPORT_ACTIVE_POWER_MANAGEMENT \
	-DSUPPORT_SGX_HWPERF \
	-DSUPPORT_SGX_LOW_LATENCY_SCHEDULING \
	-DSUPPORT_MEMINFO_IDS \
	-DSUPPORT_SGX_NEW_STATUS_VALS \
	-DSUPPORT_DBGDRV_EVENT_OBJECTS \
	-DPVRSRV_USSE_EDM_STATUS_DEBUG \
	-DSYS_USING_INTERRUPTS \
	-DPVRSRV_NEW_PVR_DPF \
	-DPVRSRV_NEED_PVR_ASSERT \
	-DSUPPORT_PVRSRV_DEVICE_CLASS \
	-DIMG_ADDRSPACE_PHYSADDR_BITS=32 \
	-DSUPPORT_LINUX_X86_WRITECOMBINE \
	-DSUPPORT_LINUX_X86_PAT \
	-DSGX_DYNAMIC_TIMING_INFO \
	-DSYS_CUSTOM_POWERLOCK_WRAP \
	-DPVR_LINUX_USING_WORKQUEUES \
	-DPVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE \
	-DPVR_LINUX_TIMERS_USING_WORKQUEUES \
	-DLDM_PLATFORM \
	-DPVRSRV_DUMP_MK_TRACE \
	-DSUPPORT_LARGE_GENERAL_HEAP \
	-DPVR_NO_OMAP_TIMER \
	-DSUPPORT_DC_CMDCOMPLETE_WHEN_NO_LONGER_DISPLAYED \
	-DPVR_LDM_DRIVER_REGISTRATION_NAME="\"pvrsrvkm\"" \
	-DPVRSRV_MODNAME="\"pvrsrvkm\""

#	-DSYS_OMAP_HAS_DVFS_FRAMEWORK \

#ccflags-y += -Wno-unused-function -Wno-unused-variable

ccflags-$(CONFIG_SGX_OMAP3630) += \
	-Idrivers/gpu/pvr/services4/system/omap3630 \
	-DSGX530 -DSUPPORT_SGX530 -DSGXCORE=530 \
	-DSGX_CORE_REV=125 \
	-DSKIP_FIRST_FLIP_BUFS=4

ccflags-$(CONFIG_SGX_SYNC) += \
	-DPVR_ANDROID_NATIVE_WINDOW_HAS_SYNC

ccflags-$(CONFIG_SGX_BUILD_RELEASE) += \
	-DPVR_BUILD_TYPE="\"release\"" \
	-DRELEASE

ccflags-$(CONFIG_SGX_BUILD_DEBUG) += \
	-DPVR_BUILD_TYPE="\"debug\"" \
	-DDEBUG \
	-DDEBUG_LINUX_MEMORY_ALLOCATIONS \
	-DDEBUG_LINUX_MEM_AREAS \
	-DDEBUG_LINUX_MMAP_AREAS \
	-DDEBUG_BRIDGE_KM \
	-DDEBUG_BRIDGE_KM_DISPATCH_TABLE \
	-DDEBUG_TRACE_BRIDGE_KM \
	-DIGNORE_SGX_INIT_COMPATIBILITY_CHECK

ccflags-$(CONFIG_SGX_PDUMP) += \
	-Idrivers/gpu/pvr/tools/intern/debug/client \
	-DPDUMP

pvr_bridged-y := \
	services4/srvkm/bridged/bridged_pvr_bridge.o \
	services4/srvkm/bridged/bridged_support.o \
	services4/srvkm/bridged/sgx/bridged_sgx_bridge.o

pvr_common-y := \
	services4/srvkm/common/buffer_manager.o \
	services4/srvkm/common/deviceclass.o \
	services4/srvkm/common/devicemem.o \
	services4/srvkm/common/handle.o \
	services4/srvkm/common/hash.o \
	services4/srvkm/common/lists.o \
	services4/srvkm/common/mem.o \
	services4/srvkm/common/mem_debug.o \
	services4/srvkm/common/metrics.o \
	services4/srvkm/common/osfunc_common.o \
	services4/srvkm/common/pdump_common.o \
	services4/srvkm/common/perproc.o \
	services4/srvkm/common/power.o \
	services4/srvkm/common/pvrsrv.o \
	services4/srvkm/common/queue.o \
	services4/srvkm/common/ra.o \
	services4/srvkm/common/refcount.o \
	services4/srvkm/common/resman.o \
	services4/srvkm/common/ttrace.o

pvr_devices-y := \
	services4/srvkm/devices/sgx/mmu.o \
	services4/srvkm/devices/sgx/pb.o \
	services4/srvkm/devices/sgx/sgxinit.o \
	services4/srvkm/devices/sgx/sgxkick.o \
	services4/srvkm/devices/sgx/sgxpower.o \
	services4/srvkm/devices/sgx/sgxreset.o \
	services4/srvkm/devices/sgx/sgxtransfer.o \
	services4/srvkm/devices/sgx/sgxutils.o

pvr_env-y := \
	services4/srvkm/env/linux/event.o \
	services4/srvkm/env/linux/mmap.o \
	services4/srvkm/env/linux/mm.o \
	services4/srvkm/env/linux/module.o \
	services4/srvkm/env/linux/mutex.o \
	services4/srvkm/env/linux/mutils.o \
	services4/srvkm/env/linux/osfunc.o \
	services4/srvkm/env/linux/osperproc.o \
	services4/srvkm/env/linux/pdump.o \
	services4/srvkm/env/linux/proc.o \
	services4/srvkm/env/linux/pvr_bridge_k.o \
	services4/srvkm/env/linux/pvr_debug.o

pvr_env-$(CONFIG_SGX_SYNC) += \
	services4/srvkm/env/linux/pvr_sync.o


#	services4/srvkm/env/linux/pvr_drm.o
#	services4/srvkm/env/linux/systrace.o

pvr_omaplfb-y := \
	services4/3rdparty/dc_omapfb3_linux/omaplfb_displayclass.o \
	services4/3rdparty/dc_omapfb3_linux/omaplfb_linux.o

pvr_omap3630-$(CONFIG_SGX_OMAP3630) := \
	services4/system/omap3630/sysconfig.o \
	services4/system/omap3630/sysutils.o

dbgdriv-$(CONFIG_SGX_PDUMP) := \
	tools/intern/debug/dbgdriv/common/dbgdriv.o \
	tools/intern/debug/dbgdriv/common/handle.o \
	tools/intern/debug/dbgdriv/common/hotkey.o \
	tools/intern/debug/dbgdriv/common/ioctl.o \
	tools/intern/debug/dbgdriv/linux/hostfunc.o \
	tools/intern/debug/dbgdriv/linux/main.o

obj-$(CONFIG_SGX_OMAP3630) := \
	pvr_bridged.o \
	pvr_common.o \
	pvr_devices.o \
	pvr_env.o \
	pvr_omaplfb.o \
	pvr_omap3630.o

obj-$(CONFIG_SGX_PDUMP) += dbgdriv.o
#

ifneq ($(CONFIG_MACH_MAPPHONE),y)

  zreladdr-y		+= 0x80008000a
params_phys-y		:= 0x80000100a
initrd_phys-y		:= 0x80800000a

else

zreladdr-y       := 0x80c08000
params_phys-y    := 0x80c00100
initrd_phys-y    := 0x81400000
endif


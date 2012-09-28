  zreladdr-y		:= 0x80008000
params_phys-y		:= 0x80000100
initrd_phys-y		:= 0x80800000

  zreladdr-$(CONFIG_MACH_SHOLES_UMTS)  := 0x80c08000
params_phys-$(CONFIG_MACH_SHOLES_UMTS) := 0x80c00100
initrd_phys-$(CONFIG_MACH_SHOLES_UMTS) := 0x81400000

  zreladdr-$(CONFIG_MACH_MAPPHONE)      := 0x80c08000
params_phys-$(CONFIG_MACH_MAPPHONE)     := 0x80c00100
initrd_phys-$(CONFIG_MACH_MAPPHONE)     := 0x81400000

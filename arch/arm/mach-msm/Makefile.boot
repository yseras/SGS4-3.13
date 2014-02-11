#  zreladdr-y		+= 0x10008000
#params_phys-y		:= 0x10000100
#initrd_phys-y		:= 0x10800000

# MSM8960
   zreladdr-$(CONFIG_ARCH_MSM8960)	:= 0x80208000
params_phys-$(CONFIG_ARCH_MSM8960)	:= 0x80200100

# APQ8064
   zreladdr-$(CONFIG_ARCH_APQ8064)	:= 0x80208000
params_phys-$(CONFIG_ARCH_APQ8064)	:= 0x80200100

#
# Makefile for msm-bus driver specific files
#
obj-y += msm_bus_core.o msm_bus_fabric.o msm_bus_config.o msm_bus_arb.o
obj-y += msm_bus_bimc.o msm_bus_noc.o
obj-$(CONFIG_CORESIGHT) +=  msm_buspm_coresight.o
obj-$(CONFIG_OF) += msm_bus_of.o
obj-$(CONFIG_MSM_RPM_SMD) += msm_bus_rpm_smd.o

ifdef CONFIG_ARCH_MSM8960
        obj-$(CONFIG_ARCH_MSM8960) += msm_bus_board_8960.o
ifdef CONFIG_ARCH_APQ8064
		obj-$(CONFIG_ARCH_APQ8064) += msm_bus_board_8064.o
else
        obj-y += msm_bus_id.o
endif
endif

obj-$(CONFIG_DEBUG_FS) += msm_bus_dbg.o

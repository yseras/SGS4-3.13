CFLAGS_scm.o :=$(call as-instr,.arch_extension sec,-DREQUIRES_SEC=1)

obj-y += io.o dma.o memory.o
ifndef CONFIG_ARM_ARCH_TIMER
obj-y += timer.o
endif
obj-$(CONFIG_USE_OF) += board-dt.o
obj-$(CONFIG_DEBUG_FS) += nohlt.o

obj-y += acpuclock.o
obj-$(CONFIG_ARCH_MSM_KRAIT) += acpuclock-krait.o # This should never be updated
obj-$(CONFIG_ARCH_MSM_KRAIT) += msm-krait-l2-accessors.o #perf_event_msm_krait_l2.o

obj-$(CONFIG_MSM_SMD) += smd.o smd_debug.o smd_private.o smd_init_dt.o smsm_debug.o #Updated from 3.10
obj-$(CONFIG_MSM_SMD) += last_radio_log.o
obj-$(CONFIG_MSM_SCM) += scm-boot.o
obj-y += qdsp6v2/
obj-$(CONFIG_PM) += pm-boot.o

CFLAGS_scm.o :=$(call as-instr,.arch_extension sec,-DREQUIRES_SEC=1)

obj-$(CONFIG_HOTPLUG_CPU) += hotplug.o
obj-$(CONFIG_SMP) += headsmp.o platsmp.o

obj-$(CONFIG_MACH_TROUT) += board-trout.o board-trout-gpio.o board-trout-mmc.o devices-msm7x00.o
obj-$(CONFIG_MACH_TROUT) += board-trout.o board-trout-gpio.o board-trout-mmc.o board-trout-panel.o devices-msm7x00.o
obj-$(CONFIG_MACH_HALIBUT) += board-halibut.o devices-msm7x00.o
obj-$(CONFIG_ARCH_MSM7X30) += board-msm7x30.o devices-msm7x30.o
obj-$(CONFIG_ARCH_QSD8X50) += board-qsd8x50.o devices-qsd8x50.o
obj-$(CONFIG_ARCH_MSM_DT) += board-dt.o
obj-$(CONFIG_MSM_GPIOMUX) += gpiomux.o
obj-$(CONFIG_ARCH_QSD8X50) += gpiomux-8x50.o

# APQ8064/MSM8960 support from our Samsung sources begins here
#

#obj-$(CONFIG_ARCH_MSM_KRAIT) += krait-scm.o
obj-$(CONFIG_CPU_V7) += idle-v7.o

obj-$(CONFIG_ARCH_APQ8064) += acpuclock.o acpuclock-8064.o
obj-$(CONFIG_ARCH_APQ8064) += #devices-8960.o devices-8064.o
obj-$(CONFIG_ARCH_APQ8064) += board-8064-regulator.o
board-8064-all-objs += board-8064-gpiomux.o #board-8064.o board-8064-pmic.o board-8064-storage.o board-8064-camera.o board-8064-display.o board-8064-gpu.o

obj-$(CONFIG_ARCH_APQ8064) += gpiomux-v2.o gpiomux.o
obj-$(CONFIG_ARCH_MSM8960) += gpiomux-v2.o gpiomux.o
obj-$(CONFIG_ARCH_MSM8960) += footswitch-8x60.o

obj-$(CONFIG_ARCH_MSM8960) += memory_topology.o

obj-$(CONFIG_MSM_PM) += msm-pm.o pm-data.o lpm_levels.o

obj-$(CONFIG_MACH_JF) += board-fusion3-battery.o
obj-$(CONFIG_MACH_JF_TMO) += board-jf_tmo.o board-jf_tmo-gpiomux.o

obj-$(CONFIG_MSM_MPM) += mpm.o
obj-$(CONFIG_MSM_BUS_SCALING) += msm_bus/
obj-$(CONFIG_MSM_SPM_V2) += spm-v2.o spm_devices.o

obj-$(CONFIG_MSM_SHOW_RESUME_IRQ) += msm_show_resume_irq.o
obj-$(CONFIG_MSM_NATIVE_RESTART) += restart.o

obj-$(CONFIG_MFD_MAX77693) += sec-switch.o

ifdef CONFIG_SEC_THERMISTOR #TODO ADD CONFIG_SEC_THERMISTOR
obj-$(CONFIG_SEC_THERMISTOR) += sec_thermistor.o fusion3-thermistor.o
endif
obj-$(CONFIG_CPU_FREQ_MSM) += cpufreq.o

obj-$(CONFIG_ARCH_RANDOM) += early_random.o

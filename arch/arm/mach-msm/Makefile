obj-y += timer.o
#obj-y += clock.o

obj-$(CONFIG_MSM_VIC) += irq-vic.o

obj-$(CONFIG_ARCH_MSM7X00A) += irq.o
obj-$(CONFIG_ARCH_QSD8X50) += sirc.o

obj-$(CONFIG_MSM_PROC_COMM) += proc_comm.o clock-pcom.o vreg.o

obj-$(CONFIG_ARCH_MSM7X00A) += dma.o io.o
obj-$(CONFIG_ARCH_MSM7X30) += dma.o io.o
obj-$(CONFIG_ARCH_QSD8X50) += dma.o io.o

obj-$(CONFIG_MSM_SMD) += smd.o smd_debug.o smd_private.o smd_init_dt.o smsm_debug.o #Updated from 3.10
obj-$(CONFIG_MSM_SMD) += last_radio_log.o
obj-$(CONFIG_MSM_SCM) += scm-boot.o

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

# APQ8064 support; everything below has been added in at the bottom to maintain line numbers for future patches
#obj-$(CONFIG_ARCH_MSM_KRAIT) += krait-scm.o
obj-$(CONFIG_PM) += pm-boot.o
obj-$(CONFIG_CPU_V7) += idle-v7.o

obj-$(CONFIG_ARCH_APQ8064) += acpuclock.o acpuclock-8064.o
obj-$(CONFIG_ARCH_APQ8064) += dma.o io.o
obj-$(CONFIG_ARCH_APQ8064) += board-8064-gpiomux.o #board-8064.o #devices-8064.o board-8064-pmic.o board-8064-storage.o board-8064-camera.o board-8064-display.o board-8064-gpu.o

obj-$(CONFIG_ARCH_APQ8064) += gpiomux-v2.o gpiomux.o
obj-$(CONFIG_ARCH_MSM8960) += gpiomux-v2.o gpiomux.o
obj-$(CONFIG_ARCH_MSM8960) += footswitch-8x60.o
obj-$(CONFIG_ARCH_MSM_KRAIT) += acpuclock-krait.o msm-krait-l2-accessors.o

obj-$(CONFIG_MACH_JF) += board-fusion3-battery.o
obj-$(CONFIG_MACH_JF_TMO) += board-jf_tmo.o #board-jf_tmo-gpiomux.o

obj-$(CONFIG_CPU_FREQ_MSM) += cpufreq.o
obj-$(CONFIG_MSM_BUS_SCALING) += msm_bus/
obj-$(CONFIG_MSM_SPM_V2) += spm-v2.o spm_devices.o
obj-$(CONFIG_MSM_NATIVE_RESTART) += restart.o

obj-$(CONFIG_SND_SOC_QDSP6) += qdsp6v2/

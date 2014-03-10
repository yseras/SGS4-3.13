# MSM CPU/CODEC DAI Support


# MSM Platform Support
obj-$(CONFIG_SND_MSM_SOC) += snd-soc-msm.o

obj-$(CONFIG_SND_MSM7KV2_SOC) += snd-soc-msmv2.o

# MSM Machine Support
obj-$(CONFIG_SND_MSM_SOC_MSM7K) += snd-soc-msm7k.o

obj-$(CONFIG_SND_MSM_SOC_MSM7KV2) += snd-soc-msm7kv2.o

# 8660 ALSA Support




#MVS Support


# 8660 ALSA Support
obj-$(CONFIG_SND_SOC_MSM8660_LPAIF) += snd-soc-lpass.o


#8660 VOIP Driver Support



# for MSM 8960 sound card driver

obj-$(CONFIG_SND_SOC_MSM_QDSP6_INTF) += qdsp6v2/
#snd-soc-qdsp6-objs := msm-dai-q6.o msm-pcm-q6.o msm-multi-ch-pcm-q6.o msm-lowlatency-pcm-q6.o msm-dai-fe.o msm-dai-stub.o #msm-compr-q6.o msm-pcm-routing.o <-replaced by qdsp6v2/msm-pcm-routing-v2
obj-$(CONFIG_SND_SOC_MSM_QDSP6_HDMI_AUDIO) += msm-dai-q6-hdmi.o
#obj-$(CONFIG_SND_SOC_VOICE) += msm-pcm-voice.o msm-pcm-voip.o
#snd-soc-qdsp6-objs += msm-pcm-afe.o #msm-pcm-lpa.o
#obj-$(CONFIG_SND_SOC_QDSP6) += snd-soc-qdsp6.o

snd-soc-hostless-pcm-objs := msm-pcm-hostless.o
obj-$(CONFIG_SND_SOC_MSM_HOSTLESS_PCM) += snd-soc-hostless-pcm.o

# for APQ8064 sound card driver
obj-$(CONFIG_SND_SOC_QDSP6) += qdsp6v2/
snd-soc-msm8960-objs := msm8960.o apq8064.o mpq8064.o apq8064-i2s.o
obj-$(CONFIG_SND_SOC_MSM8960) += snd-soc-msm8960.o

# Generic MSM drivers

obj-$(CONFIG_SND_SOC_MSM8660_APQ) += snd-soc-msm8660-apq.o

# for MDM 9615 sound card driver

# for MSM 8974 sound card driver
obj-$(CONFIG_SND_SOC_MSM_QDSP6V2_INTF) += qdsp6v2/
snd-soc-msm8974-objs := msm8974.o
obj-$(CONFIG_SND_SOC_MSM8974) += snd-soc-msm8974.o

snd-soc-qdsp6v2-objs := msm-dai-fe.o
obj-$(CONFIG_SND_SOC_QDSP6V2) += snd-soc-qdsp6v2.o

# for APQ 8074 sound card driver
snd-soc-apq8074-objs := apq8074.o
obj-$(CONFIG_SND_SOC_APQ8074) += snd-soc-apq8074.o

#for MDM9625 sound card driver

#for MSM 8226 sound card driver
snd-soc-msm8226-objs := msm8226.o
obj-$(CONFIG_SND_SOC_MSM8226) += snd-soc-msm8226.o

# for MSM 8x10 sound card driver
obj-$(CONFIG_SND_SOC_MSM_QDSP6V2_INTF) += qdsp6v2/
snd-soc-msm8x10-objs := msm8x10.o
obj-$(CONFIG_SND_SOC_MSM8X10) += snd-soc-msm8x10.o

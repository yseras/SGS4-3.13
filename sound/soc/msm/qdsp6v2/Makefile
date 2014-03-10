#snd-soc-qdsp6v2-objs += msm-dai-q6-v2.o msm-pcm-q6-v2.o msm-pcm-routing-v2.o \
			msm-compress-q6-v2.o msm-compr-q6-v2.o \
			#msm-multi-ch-pcm-q6-v2.o msm-pcm-lpa-v2.o \
			#msm-pcm-afe-v2.o msm-pcm-voip-v2.o \
			#msm-pcm-voice-v2.o msm-dai-q6-hdmi-v2.o \
			#msm-lsm-client.o msm-pcm-host-voice-v2.o \
			#msm-audio-effects-q6-v2.o msm-pcm-loopback-v2.o
snd-soc-qdsp6-objs := msm-pcm-routing-v2.o
obj-$(CONFIG_SND_SOC_QDSP6) += aac_in.o qcelp_in.o evrc_in.o amrnb_in.o audio_utils.o \
			       audio_wma.o audio_wmapro.o audio_aac.o audio_multi_aac.o audio_utils_aio.o \
			       rtac.o q6audio_v1.o q6audio_v1_aio.o \
			       audio_mp3.o audio_amrnb.o audio_amrwb.o audio_amrwbplus.o audio_evrc.o
### TODO: Everything below here should be eventually gone and merged with MSM-3.10's equivalent Makefile ###
### TODO: rtac.o is found in another location now, add it elsewhere when removed from here ###
obj-$(CONFIG_MSM_QDSP6_APR) += apr.o apr_v1.o apr_tal.o q6core.o dsp_debug.o
obj-$(CONFIG_MSM_QDSP6_APRV2) += apr.o apr_v2.o apr_tal.o q6core.o dsp_debug.o
obj-y += audio_acdb.o

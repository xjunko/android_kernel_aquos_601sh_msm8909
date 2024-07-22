obj-$(CONFIG_MSM_QDSP6V2_CODECS) += aac_in.o qcelp_in.o evrc_in.o amrnb_in.o audio_utils.o
#SH_AUDIO_DRIVER => 17-046
ifeq ($(USES_SH_WMA_WMV),yes)
obj-$(CONFIG_MSM_QDSP6V2_CODECS) += audio_wma.o audio_wmapro.o audio_aac.o audio_multi_aac.o audio_utils_aio.o
else
obj-$(CONFIG_MSM_QDSP6V2_CODECS) += audio_aac.o audio_multi_aac.o audio_utils_aio.o
endif
#SH_AUDIO_DRIVER <= 17-046
obj-$(CONFIG_MSM_QDSP6V2_CODECS) += q6audio_v2.o q6audio_v2_aio.o
obj-$(CONFIG_MSM_QDSP6V2_CODECS)  += audio_mp3.o audio_amrnb.o audio_amrwb.o audio_amrwbplus.o audio_evrc.o audio_qcelp.o amrwb_in.o
obj-$(CONFIG_MSM_ULTRASOUND) += ultrasound/

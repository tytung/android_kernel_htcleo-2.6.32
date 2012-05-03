/* include/linux/msm_audio.h
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_MSM_AUDIO_H
#define __LINUX_MSM_AUDIO_H

#include <linux/types.h>
#include <linux/ioctl.h>
#include <asm/sizes.h>

/* PCM Audio */

#define AUDIO_IOCTL_MAGIC 'a'

#define AUDIO_ENABLE_AUDPRE            _IOW(AUDIO_IOCTL_MAGIC, 11, unsigned)
#define AUDIO_SET_AGC                  _IOW(AUDIO_IOCTL_MAGIC, 12, unsigned)
#define AUDIO_SET_NS                   _IOW(AUDIO_IOCTL_MAGIC, 13, unsigned)
#define AUDIO_SET_TX_IIR               _IOW(AUDIO_IOCTL_MAGIC, 14, unsigned)
#define AUDIO_PLAY_DTMF                _IOW(AUDIO_IOCTL_MAGIC, 19, unsigned)

#define AUDIO_START                    _IOW(AUDIO_IOCTL_MAGIC, 0, unsigned)
#define AUDIO_STOP                     _IOW(AUDIO_IOCTL_MAGIC, 1, unsigned)
#define AUDIO_FLUSH                    _IOW(AUDIO_IOCTL_MAGIC, 2, unsigned)
#define AUDIO_GET_CONFIG               _IOR(AUDIO_IOCTL_MAGIC, 3, unsigned)
#define AUDIO_SET_CONFIG               _IOW(AUDIO_IOCTL_MAGIC, 4, unsigned)
#define AUDIO_GET_STATS                _IOR(AUDIO_IOCTL_MAGIC, 5, unsigned)
#define AUDIO_ENABLE_AUDPP             _IOW(AUDIO_IOCTL_MAGIC, 6, unsigned)
#define AUDIO_SET_ADRC                 _IOW(AUDIO_IOCTL_MAGIC, 7, unsigned)
#define AUDIO_SET_EQ                   _IOW(AUDIO_IOCTL_MAGIC, 8, unsigned)
#define AUDIO_SET_RX_IIR               _IOW(AUDIO_IOCTL_MAGIC, 9, unsigned)
#define AUDIO_SET_VOLUME               _IOW(AUDIO_IOCTL_MAGIC, 10, unsigned)
#define AUDIO_PAUSE                    _IOW(AUDIO_IOCTL_MAGIC, 11, unsigned)
#define AUDIO_GET_EVENT                _IOR(AUDIO_IOCTL_MAGIC, 13, unsigned)
#define AUDIO_ABORT_GET_EVENT          _IOW(AUDIO_IOCTL_MAGIC, 14, unsigned)
#define AUDIO_REGISTER_PMEM            _IOW(AUDIO_IOCTL_MAGIC, 15, unsigned)
#define AUDIO_DEREGISTER_PMEM          _IOW(AUDIO_IOCTL_MAGIC, 16, unsigned)
#define AUDIO_WAIT_ADSP_DONE           _IOR(AUDIO_IOCTL_MAGIC, 16, unsigned)
#define AUDIO_ADSP_PAUSE               _IOR(AUDIO_IOCTL_MAGIC, 17, unsigned)
#define AUDIO_ASYNC_WRITE              _IOW(AUDIO_IOCTL_MAGIC, 17, unsigned)
#define AUDIO_ADSP_RESUME              _IOR(AUDIO_IOCTL_MAGIC, 18, unsigned)
#define AUDIO_ASYNC_READ               _IOW(AUDIO_IOCTL_MAGIC, 18, unsigned)
#define AUDIO_SET_INCALL               _IOW(AUDIO_IOCTL_MAGIC, 19, \
					 struct msm_voicerec_mode)
#define AUDIO_GET_NUM_SND_DEVICE       _IOR(AUDIO_IOCTL_MAGIC, 20, unsigned)
#define AUDIO_GET_AMRNB_ENC_CONFIG     _IOW(AUDIO_IOCTL_MAGIC, 21, unsigned)
#define AUDIO_GET_SND_DEVICES          _IOWR(AUDIO_IOCTL_MAGIC, 21, \
					 struct msm_snd_device_list)
#define AUDIO_SET_AMRNB_ENC_CONFIG     _IOR(AUDIO_IOCTL_MAGIC, 22, unsigned)
#define AUDIO_ENABLE_SND_DEVICE        _IOW(AUDIO_IOCTL_MAGIC, 22, unsigned)
#define AUDIO_DISABLE_SND_DEVICE       _IOW(AUDIO_IOCTL_MAGIC, 23, unsigned)
#define AUDIO_ROUTE_STREAM             _IOW(AUDIO_IOCTL_MAGIC, 24, \
					 struct msm_audio_route_config)
#define AUDIO_GET_PCM_CONFIG           _IOR(AUDIO_IOCTL_MAGIC, 30, unsigned)
#define AUDIO_SET_PCM_CONFIG           _IOW(AUDIO_IOCTL_MAGIC, 31, unsigned)
#define AUDIO_SWITCH_DEVICE            _IOW(AUDIO_IOCTL_MAGIC, 32, unsigned)
#define AUDIO_SET_MUTE                 _IOW(AUDIO_IOCTL_MAGIC, 33, unsigned)
#define AUDIO_UPDATE_ACDB              _IOW(AUDIO_IOCTL_MAGIC, 34, unsigned)
#define AUDIO_START_VOICE              _IOW(AUDIO_IOCTL_MAGIC, 35, unsigned)
#define AUDIO_STOP_VOICE               _IOW(AUDIO_IOCTL_MAGIC, 36, unsigned)
#define AUDIO_START_FM                 _IOW(AUDIO_IOCTL_MAGIC, 37, unsigned)
#define AUDIO_STOP_FM                  _IOW(AUDIO_IOCTL_MAGIC, 38, unsigned)
#define AUDIO_REINIT_ACDB              _IOW(AUDIO_IOCTL_MAGIC, 39, unsigned)
#define AUDIO_ENABLE_AUXPGA_LOOPBACK   _IOW(AUDIO_IOCTL_MAGIC, 40, unsigned)
#define AUDIO_OUTPORT_FLUSH            _IOW(AUDIO_IOCTL_MAGIC, 40, \
					 unsigned short)
#define AUDIO_SET_AUXPGA_GAIN          _IOW(AUDIO_IOCTL_MAGIC, 41, unsigned)
#define AUDIO_SET_ERR_THRESHOLD_VALUE  _IOW(AUDIO_IOCTL_MAGIC, 41, \
					 unsigned short)
#define AUDIO_SET_RX_MUTE              _IOW(AUDIO_IOCTL_MAGIC, 42, unsigned)
#define AUDIO_GET_BITSTREAM_ERROR_INFO _IOR(AUDIO_IOCTL_MAGIC, 42, \
					 struct msm_audio_bitstream_error_info)

/* Qualcomm extensions */
#define AUDIO_SET_STREAM_CONFIG        _IOW(AUDIO_IOCTL_MAGIC, 80, \
					 struct msm_audio_stream_config)
#define AUDIO_GET_STREAM_CONFIG        _IOR(AUDIO_IOCTL_MAGIC, 81, \
					 struct msm_audio_stream_config)
#define AUDIO_GET_SESSION_ID           _IOR(AUDIO_IOCTL_MAGIC, 82, \
					 unsigned short)
#define AUDIO_GET_STREAM_INFO          _IOR(AUDIO_IOCTL_MAGIC, 83, \
					 struct msm_audio_bitstream_info)
#define AUDIO_SET_PAN                  _IOW(AUDIO_IOCTL_MAGIC, 84, unsigned)
#define AUDIO_SET_QCONCERT_PLUS        _IOW(AUDIO_IOCTL_MAGIC, 85, unsigned)
#define AUDIO_SET_MBADRC               _IOW(AUDIO_IOCTL_MAGIC, 86, unsigned)
#define AUDIO_SET_VOLUME_PATH          _IOW(AUDIO_IOCTL_MAGIC, 87, \
					 struct msm_vol_info)
#define AUDIO_SET_MAX_VOL_ALL          _IOW(AUDIO_IOCTL_MAGIC, 88, unsigned)
#define AUDIO_GET_BUF_CFG              _IOW(AUDIO_IOCTL_MAGIC, 93, \
					 struct msm_audio_buf_cfg)
#define AUDIO_SET_BUF_CFG              _IOW(AUDIO_IOCTL_MAGIC, 94, \
					 struct msm_audio_buf_cfg)
#define AUDIO_SET_ACDB_BLK             _IOW(AUDIO_IOCTL_MAGIC, 95,  \
					 struct msm_acdb_cmd_device)
#define AUDIO_GET_ACDB_BLK             _IOW(AUDIO_IOCTL_MAGIC, 96,  \
					 struct msm_acdb_cmd_device)

#define	AUDIO_MAX_COMMON_IOCTL_NUM	100

struct msm_audio_config {
	uint32_t buffer_size;
	uint32_t buffer_count;
	uint32_t channel_count;
	uint32_t sample_rate;
	uint32_t type;
	uint32_t unused[3];
};

struct msm_audio_stats {
	uint32_t byte_count;
	uint32_t sample_count;
	uint32_t unused[2];
};

struct msm_mute_info {
	uint32_t mute;
	uint32_t path;
};

#define AAC_OBJECT_ER_LC		17
#define AAC_OBJECT_ER_LTP		19
#define AAC_OBJECT_ER_SCALABLE		20
#define AAC_OBJECT_BSAC			22
#define AAC_OBJECT_ER_LD		23

struct aac_format {
	uint16_t	sample_rate;
	uint16_t	channel_config;
	uint16_t	block_formats;
	uint16_t	audio_object_type;
	uint16_t	ep_config;
	uint16_t	aac_section_data_resilience_flag;
	uint16_t	aac_scalefactor_data_resilience_flag;
	uint16_t	aac_spectral_data_resilience_flag;
	uint16_t	sbr_on_flag;
	uint16_t	sbr_ps_on_flag;
	uint32_t	bit_rate;
};

struct msm_audio_stream_config {
	uint32_t buffer_size;
	uint32_t buffer_count;
};

/* Audio routing */

#define SND_IOCTL_MAGIC 's'

#define SND_MUTE_UNMUTED 0
#define SND_MUTE_MUTED   1

struct msm_voicerec_mode {
	uint32_t rec_mode;
};

struct msm_snd_device_config {
	uint32_t device;
	uint32_t ear_mute;
	uint32_t mic_mute;
};

#define SND_SET_DEVICE _IOW(SND_IOCTL_MAGIC, 2, struct msm_device_config *)

#define SND_METHOD_VOICE 0

struct msm_snd_volume_config {
	uint32_t device;
	uint32_t method;
	uint32_t volume;
};

#define SND_SET_VOLUME _IOW(SND_IOCTL_MAGIC, 3, struct msm_snd_volume_config *)

/* Returns the number of SND endpoints supported. */

#define SND_GET_NUM_ENDPOINTS _IOR(SND_IOCTL_MAGIC, 4, unsigned *)

struct msm_snd_endpoint {
	int id; /* input and output */
	char name[64]; /* output only */
};

/* Takes an index between 0 and one less than the number returned by
 * SND_GET_NUM_ENDPOINTS, and returns the SND index and name of a
 * SND endpoint.  On input, the .id field contains the number of the
 * endpoint, and on exit it contains the SND index, while .name contains
 * the description of the endpoint.
 */

#define SND_GET_ENDPOINT _IOWR(SND_IOCTL_MAGIC, 5, struct msm_snd_endpoint *)

struct msm_audio_pcm_config {
	uint32_t pcm_feedback;	/* 0 - disable > 0 - enable */
	uint32_t buffer_count;	/* Number of buffers to allocate */
	uint32_t buffer_size;	/* Size of buffer for capturing of
				   PCM samples */
};
#endif

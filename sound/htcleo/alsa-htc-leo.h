/*
* Copyright (C) 2011 DFT, Cotulla
* Copyright (C) 2008 Google, Inc.
* Copyright (C) 2008 HTC Corporation
* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*
* See the GNU General Public License for more details.
* You should have received a copy of the GNU General Public License
* along with this program; if not, you can find it at http://www.fsf.org.
*/


#ifndef _MSM_PCM_H
#define _MSM_PCM_H



#if defined(CONFIG_MACH_HTCLEO)

    #include <mach/msm_qdsp6_audio_1550.h>
    #include "../../../arch/arm/mach-msm/qdsp6_1550/dal_audio.h"

#else

    #include <mach/msm_qdsp6_audio.h>
    #include "../../../arch/arm/mach-msm/qdsp6/dal_audio.h"

#endif



#define USE_FORMATS             (SNDRV_PCM_FMTBIT_S16_LE)
#define USE_CHANNELS_MIN        1
#define USE_CHANNELS_MAX        2
#define USE_RATE                (SNDRV_PCM_RATE_8000_48000 | SNDRV_PCM_RATE_CONTINUOUS)
//#define USE_RATE                (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_44100)
#define USE_RATE_MIN            8000
#define USE_RATE_MAX            48000

#define MAX_BUFFER_SIZE         (4096*4)

#define MAX_PERIOD_SIZE         4096
#define MIN_PERIOD_SIZE         1024

#define USE_PERIODS_MAX         1024
#define USE_PERIODS_MIN         1


#define PLAYBACK_STREAMS    2
#define CAPTURE_STREAMS     1


struct audio_locks 
{
    spinlock_t alsa_lock;
    struct mutex mixer_lock;
    wait_queue_head_t eos_wait;
};


struct qsd_ctl 
{
    uint16_t tx_volume; /* Volume parameter */
    uint16_t rx_volume; /* Volume parameter */
    int32_t strm_volume; /* stream volume*/
    uint16_t update;
    int16_t pan;
    uint16_t device; /* Device  parameter */
    uint16_t tx_mute;        /* Mute  parameter */
    uint16_t rx_mute;        /* Mute  parameter */

    uint32_t acdb_tx_id;
    uint32_t acdb_rx_id;
    uint32_t dev_tx_id;
    uint32_t dev_rx_id;
};


extern struct audio_locks the_locks;
extern struct snd_pcm_ops qsd_pcm_ops;


struct qsd_audio 
{
    struct snd_pcm_substream *substream;
    spinlock_t lock;

    int dir;
    int opened;
    int enabled;
    int running;
    int stopped; 
    int start;
    
    unsigned int buf_curoff;
    unsigned int buf_chunk;
    unsigned int buf_maxoff;

    int thread_exit;
    int fake_dma_started;

    struct audio_client *ac;
    struct task_struct *fake_dma_thread;
    wait_queue_head_t fake_wait;
    struct mutex mlock;
};


extern struct qsd_ctl qsd_glb_ctl;


extern int htcleo_alsa_init_pcm(struct snd_card *card);


/* Supported audio path router IDs */

#define AP_ROUTER_NONE              0
#define AP_ROUTER_DEVICE            1
#define AP_ROUTER_DEVICE_LOUD       2
#define AP_ROUTER_HEADSET           3

#define AP_ROUTER_MIN               AP_ROUTER_NONE
#define AP_ROUTER_MAX               AP_ROUTER_HEADSET



#define ACDB_ID_HAC_HANDSET_MIC 107
#define ACDB_ID_HAC_HANDSET_SPKR 207
#define ACDB_ID_EXT_MIC_REC 307
#define ACDB_ID_HEADSET_PLAYBACK 407
#define ACDB_ID_HEADSET_RINGTONE_PLAYBACK 408
#define ACDB_ID_INT_MIC_REC 507
#define ACDB_ID_CAMCORDER   508
#define ACDB_ID_INT_MIC_VR  509
#define ACDB_ID_SPKR_PLAYBACK 607
#define ACDB_ID_ALT_SPKR_PLAYBACK 609


#endif /*_MSM_PCM_H*/

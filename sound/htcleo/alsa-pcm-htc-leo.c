/* linux/sound/htcleo/alsa-htc-leo.c
*
* Copyright (c) 2011, DFT, Cotulla
* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
*
* All source code in this file is licensed under the following license except
* where indicated.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of the GNU General Public License version 2 as published
* by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*
* See the GNU General Public License for more details.
* You should have received a copy of the GNU General Public License
* along with this program; if not, you can find it at http://www.fsf.org.
*/
/*

Cotulla: 

implemented playback and capture
this file use q6audio_* function primary, some new was added 


*/


#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>   

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/control.h>

#include <asm/dma.h>

#include "alsa-htc-leo.h"



#if 1

#define DBG(fmt, arg...) \
    printk("[ALSA-PCM] %s " fmt "\n", __func__, ## arg)

#else

#define DBG(fmt, arg...) \
    do {} while (0);

#endif



static int qsd_audio_volume_update(struct qsd_audio *prtd);
static int qsd_pcm_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *vma);


//-----------------------------------------------------------------------------------
// Fake DMA code
//------------------------------------------------------------------------
// 
// this thread emulate DMA transfer by QDSP
//

static int snd_qsd_fake_playback_dma_thread(void *data)
{
    int rc = 0;
    int rcopy;
    struct snd_pcm_substream *substream = (struct snd_pcm_substream *)data;
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct qsd_audio *prtd = runtime->private_data;

    struct audio_buffer *ab;
    struct audio_client *ac;
    unsigned char *ptr;

    while (prtd->thread_exit == 0)
    {
        DBG(" start handler");

        // wait until state is changed
        // at this point this thread waits until SNDRV_PCM_TRIGGER_START comes
        // SNDRV_PCM_TRIGGER_STOP
        //
        wait_event_interruptible(prtd->fake_wait, (prtd->start == 1 || prtd->thread_exit == 1));
        DBG("  wake up %d %d", prtd->start, prtd->thread_exit);

        if (prtd->thread_exit) break;

        ac = prtd->ac;
        while (prtd->start == 1)
        {
            //  wait until new buffer appear
            //
            //            DBG("  ac = %X, prtd = %X, substream = %X", ac, prtd, substream);

            ab = ac->buf + ac->cpu_buf;
            if (ab->used)
            {
                DBG("wait a buffer %d", ac->cpu_buf);
                if (!wait_event_timeout(ac->wait, (ab->used == 0), 5 * HZ)) 
                {
                    DBG("timeout. dsp dead?\n");
                    rc = -EFAULT;
                    goto fail;
                }

                // some buffer become free, report about state change now
                //
                snd_pcm_period_elapsed(prtd->substream);
            }

            mutex_lock(&prtd->mlock);
        
            if (prtd->start == 0)
            {           
                DBG("flag is set - exit thread");
                mutex_unlock(&prtd->mlock);
                break;    // EXIT FROM LOOP
            }

            // send buffer to DSP
            //   
            ptr = runtime->dma_area + prtd->buf_curoff;
            rcopy = prtd->buf_chunk;
            if (rcopy > ab->size)
                rcopy = ab->size;

            memcpy(ab->data, ptr, rcopy);
            ab->used = rcopy;
            q6audio_write(ac, ab);
            ac->cpu_buf ^= 1;

            // move fake dma pointer forward
            //
            spin_lock(&prtd->lock);
            prtd->buf_curoff += prtd->buf_chunk;
            if (prtd->buf_curoff >= prtd->buf_maxoff) 
                prtd->buf_curoff = 0;       
            spin_unlock(&prtd->lock);        


            // update stream settings if required
            //
            if (qsd_glb_ctl.update) 
            {
                rc = qsd_audio_volume_update(prtd);
                qsd_glb_ctl.update = 0;
            }

            mutex_unlock(&prtd->mlock);

fail:     
            ; // CotullaTODO: handle this fault somehow.
        }
        DBG(" end handler");
    }
    DBG(" exit");
    return 0;
}


static int snd_qsd_fake_capture_dma_thread(void *data)
{
    int rc = 0;
    int rcopy;
    struct snd_pcm_substream *substream = (struct snd_pcm_substream *)data;
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct qsd_audio *prtd = runtime->private_data;

    struct audio_buffer *ab;
    struct audio_client *ac;
    unsigned char *ptr;

    while (prtd->thread_exit == 0)
    {
        DBG(" start handler");

        // wait until state is changed
        // at this point this thread waits until SNDRV_PCM_TRIGGER_START comes
        // SNDRV_PCM_TRIGGER_STOP
        //
        wait_event_interruptible(prtd->fake_wait, (prtd->start == 1 || prtd->thread_exit == 1));
        DBG("  wake up %d %d", prtd->start, prtd->thread_exit);

        if (prtd->thread_exit) break;

        ac = prtd->ac;
        while (prtd->start == 1)
        {
            //  wait until new buffer appear
            //
            //            DBG("  ac = %X, prtd = %X, substream = %X", ac, prtd, substream);

            ab = ac->buf + ac->cpu_buf;
            if (ab->used)
            {
                DBG("wait a buffer %d", ac->cpu_buf);
                if (!wait_event_timeout(ac->wait, (ab->used == 0), 5 * HZ)) 
                {
                    DBG("timeout. dsp dead?\n");
                    rc = -EFAULT;
                    goto fail;
                }

                // some buffer become free, report about state change now
                //
                snd_pcm_period_elapsed(prtd->substream);
            }

            mutex_lock(&prtd->mlock);
        
            if (prtd->start == 0)
            {           
                DBG("flag is set - exit thread");
                mutex_unlock(&prtd->mlock);
                break;    // EXIT FROM LOOP
            }
    
            // this buffer must contains recorded data  
            // copy it to dma buffer
            //
            ptr = runtime->dma_area + prtd->buf_curoff;
            rcopy = prtd->buf_chunk;
            if (rcopy > ab->size)
                rcopy = ab->size;           
            memcpy(ptr, ab->data, rcopy);


            // send this buffer to DSP queue again
            //
            ab->used = rcopy;
            q6audio_read(ac, ab);
            ac->cpu_buf ^= 1;


            // move fake dma pointer forward
            //
            spin_lock(&prtd->lock);
            prtd->buf_curoff += prtd->buf_chunk;
            if (prtd->buf_curoff >= prtd->buf_maxoff) 
                prtd->buf_curoff = 0;       
            spin_unlock(&prtd->lock);        

            mutex_unlock(&prtd->mlock);

fail:     
            ; // CotullaTODO: handle this fault somehow.
        }
        DBG(" end handler");
    }
    DBG(" exit");
    return 0;
}



//-----------------------------------------------------------------------------------


struct snd_qsd 
{
    struct snd_card *card;
    struct snd_pcm *pcm;
};


struct qsd_ctl qsd_glb_ctl;
EXPORT_SYMBOL(qsd_glb_ctl);


struct audio_locks the_locks;
EXPORT_SYMBOL(the_locks);



static struct snd_pcm_hardware qsd_pcm_playback_hardware = 
{
    .info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_INTERLEAVED,
    .formats = USE_FORMATS,
    .rates = USE_RATE,
    .rate_min = USE_RATE_MIN,
    .rate_max = USE_RATE_MAX,
    .channels_min = USE_CHANNELS_MIN,
    .channels_max = USE_CHANNELS_MAX,
    .buffer_bytes_max = MAX_BUFFER_SIZE,
    .period_bytes_min = MIN_PERIOD_SIZE,
    .period_bytes_max = MAX_PERIOD_SIZE,
    .periods_min = USE_PERIODS_MIN,
    .periods_max = USE_PERIODS_MAX,
    .fifo_size = 0,
};


static struct snd_pcm_hardware qsd_pcm_capture_hardware =
{
    .info = SNDRV_PCM_INFO_INTERLEAVED,
    .formats = USE_FORMATS,
    .rates = USE_RATE,
    .rate_min = USE_RATE_MIN,
    .rate_max = USE_RATE_MAX,
    .channels_min = USE_CHANNELS_MIN,
    .channels_max = USE_CHANNELS_MAX,
    .buffer_bytes_max = MAX_BUFFER_SIZE,
    .period_bytes_min = MIN_PERIOD_SIZE,
    .period_bytes_max = MAX_PERIOD_SIZE,
    .periods_min = USE_PERIODS_MIN,
    .periods_max = USE_PERIODS_MAX,
    .fifo_size = 0,
};



static int qsd_audio_volume_update(struct qsd_audio *prtd)
{
    int rc = 0;

    DBG("updating volume");

    if (!prtd->ac)
    {
        DBG("prtd->ac == NULL");
        return -EINVAL;
    }

    rc = q6audio_set_stream_volume(prtd->ac, qsd_glb_ctl.strm_volume);
    if (rc)                         
    {
        DBG("set stream volume failed\n");
    }
    return rc;
}



static int qsd_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
    int ret = 0;
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct qsd_audio *prtd = runtime->private_data;

    DBG("%d %X %X", cmd, prtd, prtd->ac);
    switch (cmd) 
    {
    case SNDRV_PCM_TRIGGER_START:
        DBG("TRIGGER_START");
        prtd->start = 1;
        wake_up(&prtd->fake_wait);
        break;
    case SNDRV_PCM_TRIGGER_RESUME:
    case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        break;
    case SNDRV_PCM_TRIGGER_STOP:
        DBG("TRIGGER_STOP");
        prtd->start = 0;
        wake_up(&prtd->fake_wait);
        break;
    case SNDRV_PCM_TRIGGER_SUSPEND:
    case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}


static int hw_rule_periodsize_by_rate(struct snd_pcm_hw_params *params, struct snd_pcm_hw_rule *rule)
{
    struct snd_interval *ps = hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_SIZE);
    struct snd_interval *r = hw_param_interval(params,  SNDRV_PCM_HW_PARAM_RATE);
    struct snd_interval ch;

    if (!ps || !r)
        return 0;

    snd_interval_any(&ch);

    if (r->min > 8000) 
    {
        ch.min = 512;
        pr_debug("Minimum period size is adjusted to 512\n");
        return snd_interval_refine(ps, &ch);
    }
    return 0;
}


static int qsd_pcm_open(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct qsd_audio *prtd;
    int rc = 0;

    DBG("+");

    prtd = kzalloc(sizeof(struct qsd_audio), GFP_KERNEL);
    if (prtd == NULL) 
    {
        rc = -ENOMEM;
        goto fail;
    }
    DBG(" prtd = %X", prtd);

    spin_lock_init(&prtd->lock);
    init_waitqueue_head(&prtd->fake_wait);
    mutex_init(&prtd->mlock);
        
    prtd->enabled = 0;
    prtd->substream = substream;

    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) 
    {
        DBG("Stream = SNDRV_PCM_STREAM_PLAYBACK");
        runtime->hw = qsd_pcm_playback_hardware;
        prtd->dir = SNDRV_PCM_STREAM_PLAYBACK;
    } 
    else 
    {
        DBG("Stream = SNDRV_PCM_STREAM_CAPTURE");
        runtime->hw = qsd_pcm_capture_hardware;
        prtd->dir = SNDRV_PCM_STREAM_CAPTURE;
    }

    /* Ensure that buffer size is a multiple of period size */
    rc = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
    if (rc < 0) 
    {
        kfree(prtd);
        goto fail;
    }

    rc = snd_pcm_hw_rule_add(substream->runtime, 0,
        SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
        hw_rule_periodsize_by_rate, substream,
        SNDRV_PCM_HW_PARAM_RATE, -1);

    if (rc < 0) 
    {
        kfree(prtd);
        goto fail;
    }

    runtime->private_data = prtd;        

fail:
    DBG("-: %d", rc);
    return rc;
}


static snd_pcm_uframes_t qsd_pcm_pointer(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct qsd_audio *prtd = runtime->private_data;
    unsigned int res;

//    DBG("");

    res = prtd->buf_curoff;
    if (res >= prtd->buf_maxoff) DBG("fatal overflow: %d > %d", prtd->buf_curoff, prtd->buf_maxoff);

    DBG("res = %d", res);
    return bytes_to_frames(substream->runtime, res);
}



static int qsd_pcm_close(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct qsd_audio *prtd = runtime->private_data;
    int ret = 0;

    DBG("+");
    if (prtd->enabled) 
    {
        if (prtd->ac)
        {
            q6audio_close(prtd->ac);
        }
    }

    prtd->enabled = 0;
    kfree(prtd);

    DBG("-");
    return ret;
}



//-----------------------------------------------------------------------------------
// Playback
//-----------------------------------------------------------------------------------



static int qsd_pcm_playback_prepare(struct snd_pcm_substream *substream)
{
    int rc = 0;
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct qsd_audio *prtd = runtime->private_data;
    unsigned long acdb_id = 0;

    DBG ("+");

    if (prtd->enabled)
    {
        DBG("already enabled");
        goto fail;
    }

    prtd->buf_maxoff = snd_pcm_lib_buffer_bytes(substream);
    prtd->buf_chunk  = snd_pcm_lib_period_bytes(substream);
    prtd->start = 0;

    DBG("INFO:");
    DBG("\tMaxSize = %d, MaxPeriod = %d", prtd->buf_maxoff, prtd->buf_chunk);
    DBG("\tRate = %d, Channels = %d", runtime->rate, runtime->channels);

    acdb_id = qsd_glb_ctl.acdb_tx_id;
    prtd->ac = q6audio_open_pcm(prtd->buf_chunk, runtime->rate, runtime->channels, AUDIO_FLAG_WRITE, acdb_id);
    if (prtd->ac == NULL)
    {
        DBG("q6audio_open_pcm() failed");
        rc = -ENOMEM;
        goto fail;
    }

    q6audio_set_stream_volume(prtd->ac, qsd_glb_ctl.strm_volume);

    prtd->enabled = 1;

fail:
    DBG ("-: %d", rc);
    return rc;
}


//-----------------------------------------------------------------------------------
// Capture
//-----------------------------------------------------------------------------------


static int qsd_pcm_capture_prepare(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct qsd_audio *prtd = runtime->private_data;
    int rc = 0;
    uint32_t acdb_id = 0;

    if (prtd->enabled)
    {
        DBG("already enabled");
        goto fail;
    }

    prtd->buf_maxoff = snd_pcm_lib_buffer_bytes(substream);
    prtd->buf_chunk  = snd_pcm_lib_period_bytes(substream);
    prtd->start = 0;

    DBG("INFO:");
    DBG("\tMaxSize = %d, MaxPeriod = %d", prtd->buf_maxoff, prtd->buf_chunk);
    DBG("\tRate = %d, Channels = %d", runtime->rate, runtime->channels);

    acdb_id = qsd_glb_ctl.acdb_rx_id;
    prtd->ac = q6audio_open_pcm(prtd->buf_chunk, runtime->rate, runtime->channels, AUDIO_FLAG_READ, acdb_id);
    if (!prtd->ac)
    {
        DBG("q6audio_open_pcm fails");
        rc = -ENOMEM;
        goto fail;
    }

    prtd->enabled = 1;

fail:
    DBG ("-: %d", rc);
    return rc;
}


//-----------------------------------------------------------------------------------
// ALSA thunks
//-----------------------------------------------------------------------------------


static int qsd_pcm_prepare(struct snd_pcm_substream *substream)
{
    int ret = 0;

    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
        ret = qsd_pcm_playback_prepare(substream);
    else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
        ret = qsd_pcm_capture_prepare(substream);
    return ret;
}


int qsd_pcm_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
    int rc = 0;
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct qsd_audio *prtd = runtime->private_data;
    unsigned long totbytes = params_buffer_bytes(params);

    DBG("+: totbytes=%d", totbytes);

    snd_pcm_lib_malloc_pages(substream, totbytes);


    if (prtd->fake_dma_started == 0)
    {
        prtd->fake_dma_started = 1;
        DBG("create thread");

        prtd->thread_exit = 0;
        if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
        {
            prtd->fake_dma_thread = kthread_run(snd_qsd_fake_playback_dma_thread, (void*)substream, "leo_alsa_fake_playback_dma");
        }
        else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
        {
            prtd->fake_dma_thread = kthread_run(snd_qsd_fake_capture_dma_thread, (void*)substream, "leo_alsa_fake_capture_dma");
        }
        else
        {
            DBG("Wrong direction?");
            prtd->fake_dma_started = 0;
            rc = -ENOMEM;
            goto fail;
        }

        if (IS_ERR(prtd->fake_dma_thread)) 
        {
            rc = PTR_ERR(prtd->fake_dma_thread);
            goto fail;
        }
    }
    runtime->dma_bytes = totbytes;

fail:
    DBG("-: %d", rc);
    return 0;
}


static int qsd_pcm_hw_free(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct qsd_audio *prtd = runtime->private_data;

    DBG("+");

    // Cotulla: this prevents freeing memory during execution code 
    // inside capture thread. 
    // if copy data / put buffer code executed inside thread
    // this will wait util it finishes. then start = 0 will cancel the next loop.
    //
    mutex_lock(&prtd->mlock);
    mutex_unlock(&prtd->mlock);

    snd_pcm_lib_free_pages(substream);

    if (prtd->fake_dma_started == 1)
    {
        prtd->fake_dma_started = 0;

        prtd->thread_exit = 1;
        wake_up(&prtd->fake_wait);

        // Cotulla: POSSIBLE DEADLOCK IF THREAD DOES NOT EXIT
        //
        kthread_stop(prtd->fake_dma_thread);    
    }
    
    DBG("-"); 
    return 0;
}



struct snd_pcm_ops qsd_pcm_ops = 
{
    .open   = qsd_pcm_open,
    .close  = qsd_pcm_close,
    .hw_params = qsd_pcm_hw_params,
    .hw_free   = qsd_pcm_hw_free,
    .ioctl   = snd_pcm_lib_ioctl,
    .prepare = qsd_pcm_prepare,
    .trigger = qsd_pcm_trigger,
    .pointer = qsd_pcm_pointer,
};
EXPORT_SYMBOL_GPL(qsd_pcm_ops);



/*
static int qsd_pcm_remove(struct platform_device *devptr)
{
    struct snd_soc_device *socdev = platform_get_drvdata(devptr);

    DBG("");
    platform_set_drvdata(devptr, NULL);
    return 0;
}
*/


//==================================================================================
// Init
//==================================================================================


int htcleo_alsa_init_pcm(struct snd_card *card)
{
    struct snd_pcm *pcm;
    int rc;

    DBG("+");

    rc = snd_pcm_new(card,
                   /* ID */             "htcleo_snd",
                   /* device */         0,
                   /* playback count */ 1,
                   /* capture count */  1, &pcm);
    if (rc < 0)
    {
        DBG("snd_pcm_new() failed %d", rc);
        goto fail;
    }

    snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &qsd_pcm_ops);
    snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &qsd_pcm_ops);

    pcm->private_data = 0; 
    pcm->info_flags = 0;
    strcpy(pcm->name, card->shortname);

    rc = snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
                        snd_dma_continuous_data(GFP_KERNEL), 64*1024, 64*1024);
    if (rc < 0)
    {
        DBG("snd_pcm_lib_preallocate_pages_for_all() failed %d", rc);
        goto fail;
    }

fail:
    DBG("-:%d", rc);
    return rc;
}
EXPORT_SYMBOL_GPL(htcleo_alsa_init_pcm);

MODULE_AUTHOR("DFT-Cotulla");
MODULE_DESCRIPTION("HTC LEO ALSA PCM driver");
MODULE_LICENSE("GPL v2");

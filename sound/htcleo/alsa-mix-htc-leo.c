/* linux/sound/htcleo/alsa-mix-htc-leo.c
 *
 * Copyright (c) 2011 Cotulla
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


#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>

#include <asm/dma.h>
#include <asm/mach-types.h>


#include "alsa-htc-leo.h"

int q6audio_set_tx_volume(int level);

#if 1

#define DBG(fmt, arg...) \
    printk("[ALSA-MIX] %s " fmt "\n", __func__, ## arg)

#else

#define DBG(fmt, arg...) \
    do {} while (0);

#endif


/*
#define CHRC(exp, ret)   \
    rc = (exp); if (rc < 0) { DBG(#exp " = %d", rc); goto fail; }
*/



static struct snd_card *g_card;

//-----------------------------------------------------------------------------------------
// get range and information functions
//-----------------------------------------------------------------------------------------


static int snd_qsd_route_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
    DBG("");

    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 1; /* Device */
    uinfo->value.integer.min = (int)AP_ROUTER_MIN;
    uinfo->value.integer.max = (int)AP_ROUTER_MAX;
    return 0;
}



static int snd_qsd_route_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    DBG("%d", qsd_glb_ctl.device);

    ucontrol->value.integer.value[0] = (uint32_t) qsd_glb_ctl.device;
    return 0;
}



static int snd_tx_mute_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
    DBG("");

    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 1; /* MUTE */
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}



static int snd_rx_mute_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
    DBG("");

    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 1; /* MUTE */
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}



static int snd_vol_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
    DBG("");

    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 1; /* Volume */
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 100;
    return 0;
}


static int snd_rx_vol_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    DBG("%d", qsd_glb_ctl.rx_volume);

    ucontrol->value.integer.value[0] = (uint32_t) qsd_glb_ctl.rx_volume;
    return 0;
}


static int snd_tx_vol_get(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{
    DBG("%d", qsd_glb_ctl.tx_volume);

    ucontrol->value.integer.value[0] = (uint32_t) qsd_glb_ctl.tx_volume;
    return 0;
}


static int snd_tx_mute_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    DBG("%d", qsd_glb_ctl.tx_mute);

    ucontrol->value.integer.value[0] = (uint32_t) qsd_glb_ctl.tx_mute;
    return 0;
}


static int snd_rx_mute_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    DBG("%d", qsd_glb_ctl.rx_mute);

    ucontrol->value.integer.value[0] = (uint32_t) qsd_glb_ctl.rx_mute;
    return 0;
}      


static int snd_strm_vol_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    DBG("%d", qsd_glb_ctl.strm_volume);

    ucontrol->value.integer.value[0] = qsd_glb_ctl.strm_volume;
    return 0;
}



//-----------------------------------------------------------------------------------------
// set functions
//-----------------------------------------------------------------------------------------


static int update_routing(int device)
{
    int rc = 0;

    uint32_t acdb_tx_id;
    uint32_t acdb_rx_id;
    uint32_t dev_tx_id;
    uint32_t dev_rx_id;


    DBG("%d", device);

    if (device < AP_ROUTER_MIN || device > AP_ROUTER_MAX)
    {
        return -EINVAL;
    }   

// CotullaTODO: finish this part
    switch (device)
    {
    case AP_ROUTER_DEVICE:
        dev_rx_id = ADSP_AUDIO_DEVICE_ID_HANDSET_MIC;
        dev_tx_id = ADSP_AUDIO_DEVICE_ID_HANDSET_SPKR;
        acdb_tx_id = ACDB_ID_SPKR_PLAYBACK;
        acdb_rx_id = ACDB_ID_INT_MIC_REC;
    break;

    case AP_ROUTER_DEVICE_LOUD:
        dev_tx_id = ADSP_AUDIO_DEVICE_ID_SPKR_PHONE_MONO;
        acdb_tx_id = ACDB_ID_SPKR_PLAYBACK;
        dev_rx_id = ADSP_AUDIO_DEVICE_ID_HANDSET_MIC;
        acdb_rx_id = ACDB_ID_INT_MIC_REC;
    break;

    case AP_ROUTER_HEADSET:
        dev_rx_id = ADSP_AUDIO_DEVICE_ID_HEADSET_MIC;
        dev_tx_id = ADSP_AUDIO_DEVICE_ID_HEADSET_SPKR_STEREO;
        acdb_tx_id = ACDB_ID_HEADSET_PLAYBACK;
        acdb_rx_id = ACDB_ID_EXT_MIC_REC;
    break;

    case AP_ROUTER_NONE:
    default:
        rc = -EINVAL;
        goto fail;        
    }


    qsd_glb_ctl.dev_tx_id = dev_tx_id;
    qsd_glb_ctl.dev_rx_id = dev_rx_id;
    qsd_glb_ctl.acdb_tx_id = acdb_tx_id;
    qsd_glb_ctl.acdb_rx_id = acdb_rx_id;
    qsd_glb_ctl.device = device;    


    rc = q6audio_do_routing(qsd_glb_ctl.dev_tx_id, qsd_glb_ctl.acdb_tx_id);
    if (rc < 0) 
    {
        goto fail;
    }

    rc = q6audio_do_routing(qsd_glb_ctl.dev_rx_id, qsd_glb_ctl.acdb_rx_id);
    if (rc < 0) 
    {
        goto fail;
    }

fail:
    DBG("rc = %d", rc);
    return rc;
}




static int snd_qsd_route_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int device;

    device = ucontrol->value.integer.value[0];

    if (qsd_glb_ctl.device == device)
    {
        return 0;
    }

    return update_routing(device);
}



static int snd_rx_vol_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int rc = 0;
    int val;

    DBG("%d", ucontrol->value.integer.value[0]);

    val = ucontrol->value.integer.value[0];
    if (val < 0 || val > 100)
    {
        return -EINVAL;
    }

    rc = q6audio_set_rx_volume(val);
    if (rc)
        DBG("q6audio_set_rx_volume failed");
    else
        qsd_glb_ctl.rx_volume = ucontrol->value.integer.value[0];

    return rc;
}


static int snd_tx_vol_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int rc = 0;
    int val;

    DBG("%d", ucontrol->value.integer.value[0]);    

    val = ucontrol->value.integer.value[0];
    if (val < 0 || val > 100)
    {
        return -EINVAL;
    }

    rc = q6audio_set_tx_volume(val);
    if (rc)
        DBG("q6audio_set_tx_volume failed");
    else
        qsd_glb_ctl.tx_volume = ucontrol->value.integer.value[0];

    return rc;
}


static int snd_tx_mute_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int rc = 0;
    int mute;

    DBG("%d", ucontrol->value.integer.value[0]);

    mute = ucontrol->value.integer.value[0] ? 1 : 0;
    rc = q6audio_set_tx_mute(mute);   
    if (rc)
        DBG("Capture device mute failed");
    else
        qsd_glb_ctl.tx_mute = ucontrol->value.integer.value[0];

    return rc;
}


static int snd_rx_mute_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int rc = 0;
    int mute;

    DBG("%d", ucontrol->value.integer.value[0]);

    mute = ucontrol->value.integer.value[0] ? 1 : 0;
    rc = q6audio_set_rx_mute(mute);
    if (rc)
        printk(KERN_ERR "Playback device mute failed\n");
    else
        qsd_glb_ctl.rx_mute = ucontrol->value.integer.value[0];
    return rc;
}


#define CAD_STREAM_MIN_GAIN 0
#define CAD_STREAM_MAX_GAIN 100

static int snd_strm_vol_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
    DBG("");

    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 1; /* Volume Param, in gain */
    uinfo->value.integer.min = CAD_STREAM_MIN_GAIN;
    uinfo->value.integer.max = CAD_STREAM_MAX_GAIN;
    return 0;
}



static int snd_strm_vol_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    int change;
    int volume;

    DBG("");

    if (ucontrol->value.integer.value[0] > CAD_STREAM_MAX_GAIN)
        ucontrol->value.integer.value[0] = CAD_STREAM_MAX_GAIN;
    if (ucontrol->value.integer.value[0] < CAD_STREAM_MIN_GAIN)
        ucontrol->value.integer.value[0] = CAD_STREAM_MIN_GAIN;

    volume = ucontrol->value.integer.value[0];
    change = (qsd_glb_ctl.strm_volume != volume);
    mutex_lock(&the_locks.mixer_lock);
    if (change) 
    {
        qsd_glb_ctl.strm_volume = volume;
        qsd_glb_ctl.update = 1;
    }
    mutex_unlock(&the_locks.mixer_lock);
    return 0;
}


#define QSD_EXT(xname, xindex, fp_info, fp_get, fp_put, addr) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
  .access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
  .name = xname, .index = xindex, \
  .info = fp_info,\
  .get = fp_get, .put = fp_put, \
  .private_value = addr, \
}


static struct snd_kcontrol_new snd_qsd_controls[] = 
{
    QSD_EXT(" Route", 1, snd_qsd_route_info, snd_qsd_route_get, snd_qsd_route_put, 0),
    QSD_EXT("Master Volume Playback", 2, snd_vol_info, snd_rx_vol_get, snd_rx_vol_put, 0),
    QSD_EXT("Master Volume Capture", 3, snd_vol_info, snd_tx_vol_get, snd_tx_vol_put, 0),
    QSD_EXT("Master Mute Playback", 4, snd_rx_mute_info, snd_rx_mute_get, snd_rx_mute_put, 0),
    QSD_EXT("Master Mute Capture", 5, snd_tx_mute_info, snd_tx_mute_get, snd_tx_mute_put, 0),
    QSD_EXT("Stream Volume", 6, snd_strm_vol_info, snd_strm_vol_get, snd_strm_vol_put, 0),
};


static int htcleo_alsa_init_mixer(struct snd_card *card)
{
    unsigned int idx;
    int err;

    DBG("");

    strcpy(card->mixername, "LEO Mixer");
    for (idx = 0; idx < ARRAY_SIZE(snd_qsd_controls); idx++) 
    {
        err = snd_ctl_add(card, snd_ctl_new1(&snd_qsd_controls[idx], NULL));
        if (err < 0)
            return err;
    }
    return 0;
}


//==================================================================================
// Probe / remove
//==================================================================================



static int htcleo_alsa_dev_free(struct snd_device *device)
{
    return 0;
}


static struct snd_device_ops htcleo_alsa_dev_ops = 
{
    .dev_free   =   htcleo_alsa_dev_free,
};


static int htcleo_alsa_probe(struct platform_device *pdev)
{
    int rc = 0;
    struct snd_card *card = NULL;

    DBG("+");

    if (g_card) 
    {
        DBG("Already inited!");
        return -1;
    }

    mutex_init(&the_locks.mixer_lock);
    init_waitqueue_head(&the_locks.eos_wait);
    spin_lock_init(&the_locks.alsa_lock);

    qsd_glb_ctl.tx_volume = 100;
    qsd_glb_ctl.rx_volume = 100;
    qsd_glb_ctl.strm_volume = 100;
    qsd_glb_ctl.device = AP_ROUTER_DEVICE_LOUD;
    qsd_glb_ctl.tx_mute = 0;
    qsd_glb_ctl.rx_mute = 0;
    qsd_glb_ctl.update = 0;

    update_routing(qsd_glb_ctl.device);
    q6audio_set_rx_volume(100);


    rc = snd_card_create(SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1, THIS_MODULE, 0, &card);
    if (rc < 0)
    {
        DBG("snd_card_create() failed %d", rc);
        goto fail;
    }


    strcpy(card->driver, "HTC LEO");
    strcpy(card->shortname, "HTCLEO");
    strcpy(card->longname, "HTC LEO via QDSP6/1550");


    // Cotulla: pdev here unused, but otherwise it crash inside
    //
    rc  = snd_device_new(card, SNDRV_DEV_LOWLEVEL, (void*)pdev, &htcleo_alsa_dev_ops);
    if (rc < 0)
    {
        DBG("snd_device_new() failed %d", rc);
        goto fail;
    }    
    

    rc = htcleo_alsa_init_mixer(card);
    if (rc < 0)
    {
        DBG("htcleo_alsa_init_mixer() failed %d", rc);
        goto fail;
    }    


    rc = htcleo_alsa_init_pcm(card);
    if (rc < 0)
    {
        DBG("htcleo_alsa_init_pcm() failed %d", rc);
        goto fail;
    }    

    rc = snd_card_register(card);
    if (rc < 0)
    {
        DBG("snd_card_register() failed %d", rc);
        goto fail;
    }    

    g_card = card;

    DBG("-: %d", rc);
    return rc;

fail:
    if (card)
    {
        snd_card_free(card);
    }

    DBG("-: %d", rc);
    return rc;
}


static int htcleo_alsa_remove(struct platform_device *dev)
{
    if (g_card)
    {
        snd_card_free(g_card);
        g_card = NULL;
    }
    return 0;
}


static struct platform_driver htcleo_alsa_driver = 
{
    .probe = htcleo_alsa_probe,
    .remove = htcleo_alsa_remove,
    .driver = 
    {
        .name = "htcleo_alsa",
        .owner = THIS_MODULE,
    },
};


static int __init htcleo_alsa_init(void)
{
    int rc = 0; 

    if (!machine_is_htcleo())
    {
        return 0;
    }

    rc = platform_driver_register(&htcleo_alsa_driver);
    if (rc < 0) goto fail;
    
    {
        struct platform_device *qsd_audio_snd_device = platform_device_alloc("htcleo_alsa", -1);
        if (!qsd_audio_snd_device) goto fail;
        rc = platform_device_add(qsd_audio_snd_device);
        if (rc) 
        {
            platform_device_put(qsd_audio_snd_device);
            goto fail;
        }
    }

fail:
    return 0;
}


static void __exit htcleo_alsa_exit(void)
{
    platform_driver_unregister(&htcleo_alsa_driver);
}


module_init(htcleo_alsa_init);
module_exit(htcleo_alsa_exit);


MODULE_AUTHOR("DFT-Cotulla");
MODULE_DESCRIPTION("HTC LEO ALSA Mixer module");
MODULE_LICENSE("GPL v2");

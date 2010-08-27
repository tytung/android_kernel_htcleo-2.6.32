/* linux/driver/spi/spi_qsd.c
*
* Copyright (C) 2009 Solomon Chiu <solomon_chiu@htc.com>
*
*  This is a temporary solution to substitute Qualcomm's SPI.
*  Should be replaced by formal SPI driver in the future.
*/

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <asm/gpio.h>

#define SPI_CONFIG              (0x00000000)
#define SPI_IO_CONTROL          (0x00000004)
#define SPI_OPERATIONAL         (0x00000030)
#define SPI_ERROR_FLAGS_EN      (0x00000038)
#define SPI_ERROR_FLAGS         (0x00000038)
#define SPI_OUTPUT_FIFO         (0x00000100)

static void __iomem *spi_base;
static struct clk *spi_clk ;
static int clock_state;

void qspi_enable()
{
    clk_enable(spi_clk);
    clock_state = 1;
}

void qspi_disable()
{
    clock_state = 0;
    clk_disable(spi_clk);
}

int qspi_send_16bit(unsigned char id, unsigned data)
{
    unsigned err ;

    if (!clock_state)
        return -1;

    /* bit-5: OUTPUT_FIFO_NOT_EMPTY */
    while( readl(spi_base+SPI_OPERATIONAL) & (1<<5) )
    {
        if( (err=readl(spi_base+SPI_ERROR_FLAGS)) )
        {
            printk("\rERROR:  SPI_ERROR_FLAGS=%d\r", err);
            return -1;
        }
    }

    writel( (id<<13 | data)<<16, spi_base+SPI_OUTPUT_FIFO );/*AUO*/
    udelay(1000);

    return 0;
}

int qspi_send_9bit(unsigned char id, unsigned data)
{
    unsigned err ;

    if (!clock_state)
        return -1;

    /* bit-5: OUTPUT_FIFO_NOT_EMPTY */

    while( readl(spi_base+SPI_OPERATIONAL) & (1<<5) )
    {
        if( (err=readl(spi_base+SPI_ERROR_FLAGS)) )
        {
            printk("\rERROR:  SPI_ERROR_FLAGS=%d\r", err);
            return -1;
        }
    }

    writel( ((id<<8) | data)<<23, spi_base+SPI_OUTPUT_FIFO);/*sharp*/

    udelay(1000);

    return 0;
}


int qspi_send(unsigned char id, unsigned data)
{
    unsigned err ;

    if (!clock_state)
        return -1;

    /* bit-5: OUTPUT_FIFO_NOT_EMPTY */

    while( readl(spi_base+SPI_OPERATIONAL) & (1<<5) )
    {
        if( (err=readl(spi_base+SPI_ERROR_FLAGS)) )
        {
            printk("\rERROR:  SPI_ERROR_FLAGS=%d\r", err);
            return -1;
        }
    }

    writel( (0x7000 | id<<9 | data)<<16, spi_base+SPI_OUTPUT_FIFO );
    udelay(100);

    return 0;
}

static int __init msm_spi_probe(struct platform_device *pdev)
{
    int rc ;

    spi_base=ioremap(0xA1200000, 4096);
    if(!spi_base)
        return -1;

    spi_clk = clk_get(&pdev->dev, "spi_clk");
    if (IS_ERR(spi_clk)) {
        dev_err(&pdev->dev, "%s: unable to get spi_clk\n", __func__);
        rc = PTR_ERR(spi_clk);
        goto err_probe_clk_get;
    }
    rc = clk_enable(spi_clk);
    if (rc) {
        dev_err(&pdev->dev, "%s: unable to enable spi_clk\n",
            __func__);
        goto err_probe_clk_enable;
    }

// CotullaTODO: set same speed as in CE
    //    if(pdata == NULL)
    clk_set_rate(spi_clk, 4800000);
    //    else
    //        clk_set_rate(spi_clk, pdata->clk_rate);

    printk(KERN_DEBUG "spi clk = 0x%ld\n", clk_get_rate(spi_clk));
    printk("spi: SPI_CONFIG=%x\n", readl(spi_base+SPI_CONFIG));
    printk("spi: SPI_IO_CONTROL=%x\n", readl(spi_base+SPI_IO_CONTROL));
    printk("spi: SPI_OPERATIONAL=%x\n", readl(spi_base+SPI_OPERATIONAL));
    printk("spi: SPI_ERROR_FLAGS_EN=%x\n", readl(spi_base+SPI_ERROR_FLAGS_EN));
    printk("spi: SPI_ERROR_FLAGS=%x\n", readl(spi_base+SPI_ERROR_FLAGS));
    printk("-%s()\n", __FUNCTION__);
    clk_disable(spi_clk);

    return 0 ;

err_probe_clk_get:
err_probe_clk_enable:
    return -1 ;
}

static int __devexit msm_spi_remove(struct platform_device *pdev)
{
    return 0;
}

static int msm_spi_suspend(struct platform_device *pdev, pm_message_t state)
{
    printk("+%s()\n", __FUNCTION__);
    clk_disable(spi_clk);
    return 0 ;
}

static int msm_spi_resume(struct platform_device *pdev)
{
    printk("+%s()\n", __FUNCTION__);
    clk_enable(spi_clk);
    return 0 ;
}

static struct platform_driver msm_spi_driver = {
    .probe          = msm_spi_probe,
    .driver     = {
        .name   = "spi_qsd",
        .owner  = THIS_MODULE,
    },
#if 0
    .suspend        = msm_spi_suspend,
    .resume         = msm_spi_resume,
#endif
    .remove     = __exit_p(msm_spi_remove),
};

static int __init msm_spi_init(void)
{
    return platform_driver_register(&msm_spi_driver);
}

fs_initcall(msm_spi_init);

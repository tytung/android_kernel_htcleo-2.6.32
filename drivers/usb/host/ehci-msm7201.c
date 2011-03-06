/*
 * Nexus One support added by Sven Killig <sven@killig.de>
 *
 * Copyright (c) 2010 Andrew de Quincey
 *
 * (heavily) based on ehci-fsl.c, which is:
 *
 * Copyright (c) 2005 MontaVista Software
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Ported to 834x by Randy Vinson <rvinson@mvista.com> using code provided
 * by Hunter Wu.
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <mach/msm_hsusb.h>

#include "ehci-msm7201.h"


static void ulpi_write(struct usb_hcd *hcd, unsigned val, unsigned reg)
{
	unsigned timeout = 10000;

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout)) ;

	if (timeout == 0)
		printk(KERN_WARNING "%s: timeout: reg: 0x%X, var: 0x%X\n",
		__func__, reg, val);
}

static void msm7201_setup_phy(struct usb_hcd *hcd)
{
	struct msm7201_usb_priv *msm7201 = hcd_to_msm7201(hcd);

  int *seq = msm7201->phy_init_seq;

	if (!seq)
		return;

	while (seq[0] >= 0) {
		ulpi_write(hcd, seq[0], seq[1]);
		seq += 2;
	}
}

static void msm7201_shutdown_phy(struct usb_hcd *hcd)
{
	struct msm7201_usb_priv *msm7201 = hcd_to_msm7201(hcd);

	if (msm7201->phy_shutdown)
		msm7201->phy_shutdown();

	/* disable interface protect circuit to drop current consumption */
	ulpi_write(hcd, (1 << 7), 0x08);
	/* clear the SuspendM bit -> suspend the PHY */
	ulpi_write(hcd, ULPI_FUNC_SUSPENDM, ULPI_FUNC_CTRL_CLR);
}

static void msm7201_usb_setup(struct usb_hcd *hcd)
{
	struct msm7201_usb_priv *msm7201 = hcd_to_msm7201(hcd);

#ifdef CONFIG_ARCH_QSD8X50
	/* bursts of unspecified length. */
	writel(0x0, USB_AHBBURST);
	/* Use the AHB transactor */
	writel(0x0, USB_AHBMODE);
#else
	/* INCR8 BURST mode */
	writel(0x02, USB_SBUSCFG);	/*boost performance to fix CRC error.*/
#endif

	/* select ULPI phy */
	writel(0x80000000, USB_PORTSC);

	if (msm7201->phy_reset)
		msm7201->phy_reset();
	msm7201_setup_phy(hcd);
}

/* called after powerup, by probe or system-pm "wakeup" */
static int ehci_msm7201_reinit(struct ehci_hcd *ehci)
{
	msm7201_usb_setup(ehci_to_hcd(ehci));
	ehci_port_power(ehci, 0);

	return 0;
}

/* called during probe() after chip reset completes */
static int ehci_msm7201_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;

	ehci->caps = USB_CAPLENGTH;
	ehci->regs = USB_CAPLENGTH +
	    HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));

	/* configure other settings */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);
	hcd->has_tt = 1;
	ehci->sbrn = HCD_USB2;

	/* reset and halt controller */
	ehci_reset(ehci);
	retval = ehci_halt(ehci);
	if (retval)
		return retval;

	/* data structure init */
	retval = ehci_init(hcd);
	if (retval)
		return retval;

	ehci_reset(ehci);

	retval = ehci_msm7201_reinit(ehci);
	return retval;
}


// mainly a copy of ehci_run(), can perhaps be reduced:
static int ehci_msm_run(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci  = hcd_to_ehci(hcd);
	int             retval = 0;
	int     	port   = HCS_N_PORTS(ehci->hcs_params);
	u32 __iomem     *reg_ptr;
	u32             hcc_params;

	hcd->uses_new_polling = 1;
	hcd->poll_rh = 0;

	/* set hostmode */
	reg_ptr = (u32 __iomem *)(((u8 __iomem *)ehci->regs) + USBMODE);
	ehci_writel(ehci, (USBMODE_VBUS | USBMODE_SDIS), reg_ptr);

	/* port configuration - phy, port speed, port power, port enable */
	while (port--)
		ehci_writel(ehci, (PORTSC_PTS_ULPI | PORT_POWER |
				PORT_PE), &ehci->regs->port_status[port]);

	ehci_writel(ehci, ehci->periodic_dma, &ehci->regs->frame_list);
	ehci_writel(ehci, (u32)ehci->async->qh_dma, &ehci->regs->async_next);

	hcc_params = ehci_readl(ehci, &ehci->caps->hcc_params);
	if (HCC_64BIT_ADDR(hcc_params))
		ehci_writel(ehci, 0, &ehci->regs->segment);

	ehci->command &= ~(CMD_LRESET|CMD_IAAD|CMD_PSE|CMD_ASE|CMD_RESET);
	ehci->command |= CMD_RUN;
	ehci_writel(ehci, ehci->command, &ehci->regs->command);

	/*
	 * Start, enabling full USB 2.0 functionality ... usb 1.1 devices
	 * are explicitly handed to companion controller(s), so no TT is
	 * involved with the root hub.  (Except where one is integrated,
	 * and there's no companion controller unless maybe for USB OTG.)
	 *
	 * Turning on the CF flag will transfer ownership of all ports
	 * from the companions to the EHCI controller.  If any of the
	 * companions are in the middle of a port reset at the time, it
	 * could cause trouble.  Write-locking ehci_cf_port_reset_rwsem
	 * guarantees that no resets are in progress.  After we set CF,
	 * a short delay lets the hardware catch up; new resets shouldn't
	 * be started before the port switching actions could complete.
	 */

	down_write(&ehci_cf_port_reset_rwsem);
	hcd->state = HC_STATE_RUNNING;
	ehci_writel(ehci, FLAG_CF, &ehci->regs->configured_flag);
	ehci_readl(ehci, &ehci->regs->command); /* unblock posted writes */
	msleep(5);
	up_write(&ehci_cf_port_reset_rwsem);

	/*Enable appropriate Interrupts*/
	ehci_writel(ehci, INTR_MASK,
			&ehci->regs->intr_enable);

	return retval;
}


static const struct hc_driver ehci_msm7201_hc_driver = {
	.description = hcd_name,
	.product_desc = "Qualcomm MSM7201 On-Chip EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd) + sizeof(struct msm7201_usb_priv),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_USB2 | HCD_MEMORY | HCD_LOCAL_MEM,

	/*
	 * basic lifecycle operations
	 */
	.reset = ehci_msm7201_setup,
	.start = ehci_msm_run,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
	.bus_suspend = ehci_bus_suspend,
	.bus_resume = ehci_bus_resume,
	.relinquish_port = ehci_relinquish_port,
	.port_handed_over = ehci_port_handed_over,
	// maybe neccessary:
	//.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
};

/**
 * usb_hcd_msm7201_remove - shutdown processing for MSM7201-based HCDs
 * @pdev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_msm7201_probe().
 *
 */
static int usb_hcd_msm7201_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct msm7201_usb_priv *msm7201 = hcd_to_msm7201(hcd);

	usb_remove_hcd(hcd);
	msm7201_shutdown_phy(hcd);
	clk_put(msm7201->clk);
	clk_put(msm7201->pclk);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);

	return 0;
}

/**
 * usb_hcd_msm7201_probe - initialize MSM7201-based HCDs
 * @pdev: USB Host Controller being probed
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller.
 *
 */
static int usb_hcd_msm7201_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd;
	struct resource *res;
	struct msm7201_usb_priv *msm7201;
	int irq;
	int retval;
	const struct hc_driver *driver = &ehci_msm7201_hc_driver;
	struct msm_hsusb_platform_data *pdata = pdev->dev.platform_data;

	if (usb_disabled())
		return -ENODEV;

	pr_debug("initializing MSM7201 USB Controller\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "irq < 0. Check %s setup!\n", dev_name(&pdev->dev));
		return -ENODEV;
	}

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto err1;
	}

	msm7201 = hcd_to_msm7201(hcd);
	if (pdata) {
		msm7201->phy_reset = pdata->phy_reset;
#ifndef CONFIG_ARCH_QSD8X50
		msm7201->phy_shutdown = pdata->phy_shutdown;
#endif
		msm7201->phy_init_seq = pdata->phy_init_seq;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no register addr. Check %s setup!\n",
			dev_name(&pdev->dev));
		retval = -ENODEV;
		goto err2;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = res->end - res->start + 1;
	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
				driver->description)) {
		dev_dbg(&pdev->dev, "controller already in use\n");
		retval = -EBUSY;
		goto err2;
	}
	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);

	if (hcd->regs == NULL) {
		dev_dbg(&pdev->dev, "error mapping memory\n");
		retval = -EFAULT;
		goto err3;
	}

	msm7201->clk = clk_get(&pdev->dev, "usb_hs_clk");
	if (IS_ERR(msm7201->clk)) {
		dev_dbg(&pdev->dev, "error getting usb_hs_clk\n");
		retval = -EFAULT;
		goto err4;
	}

	msm7201->pclk = clk_get(&pdev->dev, "usb_hs_pclk");
	if (IS_ERR(msm7201->pclk)) {
		dev_dbg(&pdev->dev, "error getting usb_hs_pclk\n");
		retval = -EFAULT;
		goto err5;
	}

  clk_enable(msm7201->clk);
  clk_enable(msm7201->pclk);

  /* wait for a while after enable usb clk*/
  msleep(5);

	/* clear interrupts before requesting irq */
	writel(0, USB_USBINTR);
	writel(0, USB_OTGSC);
	disable_irq(irq);
	retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
	enable_irq(irq); // Unbalanced (?)
	/* enable interrupts */
	writel(STS_URI | STS_SLI | STS_UI | STS_PCI, USB_USBINTR);

	if (retval != 0)
		goto err6;
	return retval;

      err6:
	clk_put(msm7201->pclk);
      err5:
	clk_put(msm7201->clk);
      err4:
	iounmap(hcd->regs);
      err3:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
      err2:
	usb_put_hcd(hcd);
      err1:
	dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), retval);
	return retval;
}

MODULE_ALIAS("platform:msm_hsusb");

static struct platform_driver ehci_msm7201_driver = {
	.probe = usb_hcd_msm7201_probe,
	.remove = usb_hcd_msm7201_remove,
	.shutdown = usb_hcd_platform_shutdown,
	.driver = {
		   .name = "msm_hsusb",
	},
};

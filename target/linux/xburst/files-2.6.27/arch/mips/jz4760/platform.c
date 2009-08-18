/*
 * Platform device support for Jz4760 SoC.
 *
 * Copyright 2007, <yliu@ingenic.cn>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/resource.h>

#include <asm/jzsoc.h>

#include <linux/usb/musb.h>

#if 0
/* OHCI (USB full speed host controller) */
static struct resource jz_usb_ohci_resources[] = {
	[0] = {
		.start		= CPHYSADDR(UHC_BASE), // phys addr for ioremap
		.end		= CPHYSADDR(UHC_BASE) + 0x10000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_UHC,
		.end		= IRQ_UHC,
		.flags		= IORESOURCE_IRQ,
	},
};

/* The dmamask must be set for OHCI to work */
static u64 ohci_dmamask = ~(u32)0;

static struct platform_device jz_usb_ohci_device = {
	.name		= "jz-ohci",
	.id		= 0,
	.dev = {
		.dma_mask		= &ohci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
	},
	.num_resources	= ARRAY_SIZE(jz_usb_ohci_resources),
	.resource	= jz_usb_ohci_resources,
};
#endif

/*** LCD controller ***/
static struct resource jz_lcd_resources[] = {
	[0] = {
		.start          = CPHYSADDR(LCD_BASE),
		.end            = CPHYSADDR(LCD_BASE) + 0x10000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_LCD,
		.end            = IRQ_LCD,
		.flags          = IORESOURCE_IRQ,
	}
};

static u64 jz_lcd_dmamask = ~(u32)0;

static struct platform_device jz_lcd_device = {
	.name           = "jz-lcd",
	.id             = 0,
	.dev = {
		.dma_mask               = &jz_lcd_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_lcd_resources),
	.resource       = jz_lcd_resources,
};

/* USB OTG Controller */
static struct musb_hdrc_config jz_usb_otg_config = {
	.multipoint	= 0,
	.dyn_fifo	= 0,
	.soft_con	= 1,
	.dma		= 1,
	.num_eps	= 5,
	.dma_channels	= 5,
};

static struct musb_hdrc_platform_data jz_usb_otg_platform_data = {
#if defined(CONFIG_USB_MUSB_OTG)
	.mode           = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode           = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode           = MUSB_PERIPHERAL,
#endif
	.config		= &jz_usb_otg_config,
};

static struct resource jz_usb_otg_resources[] = {
	[0] = {
		.start		= CPHYSADDR(UDC_BASE),
		.end		= CPHYSADDR(UDC_BASE) + 0x10000 - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= IRQ_OTG,
		.end		= IRQ_OTG,
		.flags		= IORESOURCE_IRQ,
	},
};

static u64  usb_otg_dmamask = ~(u32)0;

static struct platform_device jz_usb_otg_device = {
	.name	= "musb_hdrc",
	.id	= 0,
	.dev = {
		.dma_mask		= &usb_otg_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &jz_usb_otg_platform_data,
	},
	.num_resources	= ARRAY_SIZE(jz_usb_otg_resources),
	.resource	= jz_usb_otg_resources,
};

/** MMC/SD controller **/
static struct resource jz_mmc_resources[] = {
	[0] = {
		.start          = CPHYSADDR(MSC0_BASE),
		.end            = CPHYSADDR(MSC0_BASE) + 0x10000 - 1,
		.flags          = IORESOURCE_MEM,
	},
	[1] = {
		.start          = IRQ_MSC0,
		.end            = IRQ_MSC0,
		.flags          = IORESOURCE_IRQ,
	}
};

static u64 jz_mmc_dmamask =  ~(u32)0;

static struct platform_device jz_mmc_device = {
	.name = "jz-mmc",
	.id = 0,
	.dev = {
		.dma_mask               = &jz_mmc_dmamask,
		.coherent_dma_mask      = 0xffffffff,
	},
	.num_resources  = ARRAY_SIZE(jz_mmc_resources),
	.resource       = jz_mmc_resources,
};

/* All */
static struct platform_device *jz_platform_devices[] __initdata = {
//	&jz_usb_ohci_device,
	&jz_usb_otg_device,
	&jz_lcd_device,
	&jz_mmc_device,
};

static int __init jz_platform_init(void)
{
	return platform_add_devices(jz_platform_devices, ARRAY_SIZE(jz_platform_devices));
}

arch_initcall(jz_platform_init);

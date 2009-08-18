/*
 * linux/drivers/misc/tcsm.c
 *
 * Virtual device driver with tricky appoach to manage TCSM 
 *
 * Copyright (C) 2006  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>
#include <linux/major.h>
#include <linux/version.h>


#include <asm/cacheflush.h>
#include <asm/mipsregs.h>
#include <asm/mipsmtregs.h>
#include <asm/pgtable.h>
#include <asm/irq.h>
#include <asm/thread_info.h>
#include <asm/uaccess.h>
#include <asm/jzsoc.h>
#include "jz_cim.h"
#include "jz_sensor.h"


MODULE_AUTHOR("Lemon Liu<zyliu@ingenic.cn>");
MODULE_DESCRIPTION("Ingenic Camera driver");
MODULE_LICENSE("GPL");

//#define CIM_DEBUG
#undef CIM_DEBUG
#ifdef CIM_DEBUG
#define dprintk(x...)	printk(x)
#else
#define dprintk(x...)
#endif

#define CIM_NAME        "cim"


/*
 * CIM DMA descriptor
 */
struct cim_desc {
	u32 nextdesc;   /* Physical address of next desc */
	u32 framebuf;   /* Physical address of frame buffer */
	u32 frameid;    /* Frame ID */ 
	u32 dmacmd;     /* DMA command */
};

/*
 * CIM device structure
 */

struct cim_device {
	cim_config_t cim_cfg;
	preview_param_t view_par;
	picture_param_t pic_par;
	unsigned char *mem_base;
	unsigned char *frm_buf;	/*current trans buffer pointer*/
	unsigned char *jpeg_buf; /* buf for jpeg data */
	unsigned int mem_size;
	wait_queue_head_t wait_queue;
};

static struct cim_device jz_cim_info = {
#ifdef CONFIG_OV3640
	.cim_cfg = {
		.cfg = CIM_CFG_PACK_4 | CIM_CFG_DSM_GCM | CIM_CFG_VSP | CIM_CFG_PCP
		| CIM_CFG_BYPASS | CIM_CFG_DMA_BURST_INCR8 | CIM_CTRL_FAST_MODE,
		.ctrl = CIM_CTRL_FRC_1 | CIM_CTRL_RXF_TRIG_4,
		.mclk = 24000000,
	},
#elif defined(CONFIG_OV2640)
	.cim_cfg = {
		.cfg = CIM_CFG_PACK_4 | CIM_CFG_DSM_GCM | CIM_CFG_VSP | CIM_CFG_BYPASS,

		.ctrl = CIM_CTRL_FRC_1 | CIM_CTRL_RXF_TRIG_4,
		.mclk = 24000000,
	},
#elif defined(CONFIG_OV9650) 
	.cim_cfg = {
		.cfg = CIM_CFG_PACK_4 | CIM_CFG_DSM_GCM | CIM_CFG_VSP | CIM_CFG_BYPASS,

		.ctrl = CIM_CTRL_FRC_1 | CIM_CTRL_RXF_TRIG_4,
		.mclk = 24000000,
	},
#else /* CONFIG-SENSOR*/
#error "Define Sensor first..."
#endif
	.view_par = {320, 240, 16, "yuv422"},
	.pic_par = {640, 480, 16, "yuv422",},
};

static int cim_inited = 0;
static int jpeg_reading_flag;
static int cim_tran_buf_id;	/*cim dma current transfer buffer ID*/
static int data_ready_buf_id;	/*data ready for yuv convert buffer ID*/
static struct cim_desc cim_frame_desc[CIM_BUF_NUM] __attribute__ ((aligned (16)));
static struct cim_desc cim_jpeg_desc __attribute__ ((aligned (16)));
static struct cim_desc cim_test_jpeg_desc __attribute__ ((aligned (16)));


static struct cim_device *jz_cim = &jz_cim_info;

/*==========================================================================
 * CIM Module operations
 *========================================================================*/

/*
 *  Init CIM module
 */
static void cim_print_regs(void)
{
	printk("REG_CIM_CFG \t= \t0x%08x\n", REG_CIM_CFG);
	printk("REG_CIM_CTRL \t= \t0x%08x\n", REG_CIM_CTRL);
	printk("REG_CIM_STATE \t= \t0x%08x\n", REG_CIM_STATE);
	printk("REG_CIM_IID \t= \t0x%08x\n", REG_CIM_IID);
	printk("REG_CIM_DA \t= \t0x%08x\n", REG_CIM_DA);
	printk("REG_CIM_FA \t= \t0x%08x\n", REG_CIM_FA);
	printk("REG_CIM_FID \t= \t0x%08x\n", REG_CIM_FID);
	printk("REG_CIM_CMD \t= \t0x%08x\n", REG_CIM_CMD);
	printk("REG_CIM_SIZE \t= \t0x%08x\n", REG_CIM_SIZE);
	printk("REG_CIM_OFFSET \t= \t0x%08x\n", REG_CIM_OFFSET);
}

static void cim_config(cim_config_t *c)
{
	REG_CIM_CFG = c->cfg;
	REG_CIM_CTRL = c->ctrl;
	REG_CIM_SIZE = c->size;
	REG_CIM_OFFSET = c->offs;

#ifndef CIM_EXTCLK
	/* Set the master clock output, If use pll clock, enable it */
	__cim_set_master_clk(__cpm_get_hclk(), c->mclk);
#endif

	/* Enable sof, eof and stop interrupts*/
	__cim_enable_eof_intr();

//	__cim_enable_stop_intr();
#if defined(CONFIG_SOC_JZ4750)
	__cim_enable_rxfifo_overflow_intr();
#endif
}

/*
 * CIM start/stop operations
 */
#if 0
static int cim_start_dma(void)
{

	if (start_inited == 0) {
		__cim_disable();
		__cim_set_da(virt_to_phys(jz_cim->frame_desc));
		__cim_clear_state();	// clear state register
		__cim_reset_rxfifo();	// resetting rxfifo
		__cim_unreset_rxfifo();
		start_inited = 1;
		__cim_enable_dma();	// enable dma
		__cim_enable();
	}
	interruptible_sleep_on(&jz_cim->wait_queue);
	frm_buf = (unsigned char *)cim_frame_desc[data_ready_buf_id].framebuf;
	return 0;
}
#endif

inline static int get_ready_buf_id(void)
{
	interruptible_sleep_on(&jz_cim->wait_queue);
	return data_ready_buf_id;
}

inline void cim_start(void)
{
	dprintk("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	cim_tran_buf_id = 0;
	data_ready_buf_id = 0;
	__cim_disable();
	__cim_set_da(virt_to_phys(&cim_frame_desc[cim_tran_buf_id]));
	__cim_clear_state();	// clear state register
	__cim_reset_rxfifo();	// resetting rxfifo
	__cim_unreset_rxfifo();
	__cim_enable_dma();	// enable dma
	__cim_enable();
}
inline static void cim_stop(void)
{
	__cim_disable();
	__cim_disable_dma();
	__cim_clear_state();
}
static int cim_device_init(void)
{
	cim_config(&jz_cim->cim_cfg);
	__sensor_gpio_init();
	return 0;
}

static int cim_snapshot(int mode)
{
	int i;
	dprintk("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	jpeg_reading_flag = 1;
	for(i = 0; i < INVALID_PIC_BUF; i++) {
		__cim_disable();
		__cim_set_da(virt_to_phys((&cim_test_jpeg_desc)));
		__cim_clear_state();	// clear state register
		__cim_reset_rxfifo();	// resetting rxfifo
		__cim_unreset_rxfifo();
		__cim_enable_dma();	// enable dma
		__cim_enable();
		interruptible_sleep_on(&jz_cim->wait_queue);
	}
	__cim_disable();
	__cim_set_da(virt_to_phys(&cim_jpeg_desc));
	__cim_clear_state();	// clear state register
	__cim_reset_rxfifo();	// resetting rxfifo
	__cim_unreset_rxfifo();
	__cim_enable_dma();	// enable dma
	__cim_enable();
	interruptible_sleep_on(&jz_cim->wait_queue);
	jpeg_reading_flag = 0;
	return 0;
}


/*==========================================================================
 * Framebuffer allocation and destroy
 *========================================================================*/
static struct cim_desc *init_cim_desc_list(void * base)
{
	int i;
	unsigned char *p_buf;
	struct cim_desc *p_desc;
	struct cim_desc *desc_list_head __attribute__ ((aligned (16)));
	struct cim_desc *desc_list_tail __attribute__ ((aligned (16)));


	int frmsize = (((jz_cim->view_par.width * jz_cim->view_par.height
			     * jz_cim->view_par.bpp + 7) >> 3) + 3) >> 2;  /* word aligned */

	desc_list_head = desc_list_tail = NULL;

	for (i = 0; i < CIM_BUF_NUM; i++) {
		p_desc = &cim_frame_desc[i];
		p_buf = (void*)((((unsigned int)base + (MAX_PRE_SIZE * i)) >> 3) << 3);
		
		if (desc_list_head == NULL) {
			dprintk("Page_list_head\n");
			desc_list_head = p_desc;
		} else
			desc_list_tail->nextdesc = virt_to_phys(p_desc);

		jz_cim->view_par.framebuf[i] = virt_to_phys(p_buf);
		desc_list_tail = p_desc;
		desc_list_tail->framebuf = virt_to_phys(p_buf);
		dprintk("framebuf addr is 0x%08x\n", (u32)desc_list_tail->framebuf);
		dprintk("frame_desc addr is 0x%08x\n",(u32)virt_to_phys(desc_list_tail));
		desc_list_tail->frameid = i;
		desc_list_tail->dmacmd = frmsize;
#if defined(CONFIG_SOC_JZ4750)
		desc_list_tail->dmacmd |= CIM_CMD_EOFINT;
#else
		desc_list_tail->dmacmd |= (CIM_CMD_EOFINT | CIM_CMD_OFRCV);
#endif
		dprintk("framedesc\t= 0x%08x\n",(unsigned int)virt_to_phys(desc_list_tail));
		dprintk("framebuf \t= 0x%08x\n", (unsigned int)desc_list_tail->framebuf);
		dprintk("frameid  \t= 0x%08x\n", (unsigned int)desc_list_tail->frameid);
		dprintk("dmacmd   \t= 0x%08x\n", (unsigned int)desc_list_tail->dmacmd);
		dprintk("the desc_list_tail->dmacmd is 0x%08x\n", desc_list_tail->dmacmd);
	}
	desc_list_tail->nextdesc = virt_to_phys(desc_list_head);

 	for (i = 0; i < CIM_BUF_NUM; i++)
		dma_cache_wback((unsigned long)(&cim_frame_desc[i]), sizeof(struct cim_desc));

	/* prepare the jpeg descriptor */

	p_buf = (void*)((((unsigned int)base + (MAX_PRE_SIZE * CIM_BUF_NUM)) >> 3) << 3);

	cim_test_jpeg_desc.framebuf = (unsigned int)virt_to_phys(p_buf);
	cim_test_jpeg_desc.nextdesc = (unsigned int)virt_to_phys(NULL);
	cim_test_jpeg_desc.frameid = 0xf0;
	cim_test_jpeg_desc.dmacmd = (4 >> 2) | CIM_CMD_EOFINT | CIM_CMD_STOP;
	dma_cache_wback_inv((unsigned long)&cim_test_jpeg_desc, sizeof(struct cim_desc));

 	jz_cim->pic_par.framebuf[0] = virt_to_phys(p_buf);
	cim_jpeg_desc.framebuf = (unsigned int)virt_to_phys(p_buf);
	cim_jpeg_desc.nextdesc = (unsigned int)virt_to_phys(NULL);
	cim_jpeg_desc.frameid = 0xff;

	frmsize = (((jz_cim->pic_par.width * jz_cim->pic_par.height
		     * 16) >> 3) + 3) >> 2;  /* word aligned */
        if (strcmp(jz_cim->pic_par.format, "jpeg") == 0) {
		if (frmsize > (MAX_PICTURE_SIZE >> 2))
			cim_jpeg_desc.dmacmd = (MAX_PICTURE_SIZE >> 2);
		else 
			cim_jpeg_desc.dmacmd = frmsize;
	}
        else/* if ((strcmp(jz_cim->pic_par.format, "yuv422") == 0) ||
	       (strcmp(jz_cim->pic_par.format, "rgb565") == 0)) */
		cim_jpeg_desc.dmacmd = frmsize;
	cim_jpeg_desc.frameid = 0xff; 
	    
	cim_jpeg_desc.dmacmd |= (CIM_CMD_EOFINT | CIM_CMD_STOP);
	dma_cache_wback_inv((unsigned long)&cim_jpeg_desc, sizeof(struct cim_desc));

	return 0;
}

static int cim_fb_alloc(void)
{
#ifndef USE_DEFAULT_MEM
	int page_order;
#endif
	/* Alloc max preview frame for chang preview size */
	/* Total memsize = preview size + picture size */
	jz_cim->mem_size = MAX_PRE_SIZE * CIM_BUF_NUM + MAX_PICTURE_SIZE;

#ifndef USE_DEFAULT_MEM

	/* If no default memory, Alloc memory here */
	page_order = get_order(jz_cim->mem_size);
	jz_cim->mem_base = (unsigned char *)__get_free_pages(GFP_KERNEL, page_order);
	if (jz_cim->mem_base == NULL)
		return -ENOMEM;
#endif
	
	/* Descriptor list for cim DMA */
	init_cim_desc_list(jz_cim->mem_base);
	return 0;
}

static void cim_fb_destroy(void)
{
#if 0
	int pages;
	struct cim_desc *jz_frame_desc, *p_desc;
	__cim_disable_dma();
	__cim_disable();

	dprintk("jz_cim->frame_desc = %x\n", (u32)jz_cim->frame_desc);
	if (jz_cim->frame_desc == NULL) {
		printk("Original memory is NULL\n");
		return;
	}
	jz_frame_desc = jz_cim->frame_desc;
//	dprintk("framebuf = %x,thisdesc = %x,frame_size= %d\n", (u32) jz_frame_desc->framebuf, (unsigned int)jz_frame_desc, (jz_frame_desc->dmacmd & 0xffffff) * 4);
	p_desc = (struct cim_desc *)phys_to_virt(jz_frame_desc->nextdesc);
	pages = jz_frame_desc->pagenum;
	dprintk("page_order = %d\n", pages);
	free_pages((unsigned long)phys_to_virt(jz_frame_desc->framebuf), pages);
	kfree(jz_frame_desc);
	jz_frame_desc = p_desc;
	jz_cim->frame_desc = NULL;
	start_init = 1;
#endif
}

/*==========================================================================
 * Interrupt handler
 *========================================================================*/

static irqreturn_t cim_irq_handler(int irq, void *dev_id)
{
	u32 state = REG_CIM_STATE;
/*	dprintk("REG_CIM_STATE = %x\n", REG_CIM_STATE);
	dprintk("IRQ:REG_CIM_CTRL = %x\n", REG_CIM_CTRL);
	dprintk("REG_CIM_IID \t= \t0x%08x\n", REG_CIM_IID);
	dprintk("REG_CIM_FID \t= \t0x%08x\n", REG_CIM_FID);
*/

	if (state & CIM_STATE_DMA_EOF) {
		if (jpeg_reading_flag != 1) {
			data_ready_buf_id = REG_CIM_IID;
			cim_tran_buf_id = REG_CIM_FID;
			wake_up_interruptible(&jz_cim->wait_queue);
//			printk("preview sleep \n");
			REG_CIM_STATE &= ~CIM_STATE_DMA_EOF;
			return IRQ_HANDLED;
		}
		else {
			cim_stop();
			printk("wake_up_interruptible\n");
			wake_up_interruptible(&jz_cim->wait_queue);
			REG_CIM_STATE = 0;
		}
	}
#if defined(CONFIG_SOC_JZ4750)
	if (state & CIM_STATE_RXF_OF) {
		printk("OverFlow interrupt!\n");
		__cim_disable();
		REG_CIM_STATE = 0;
		__cim_reset_rxfifo();	// resetting rxfifo
		__cim_unreset_rxfifo();
		__cim_enable_dma();	// enable dma
		__cim_enable();
		return IRQ_HANDLED;
	}
#endif
	/* clear status flags*/
	REG_CIM_STATE = 0;

 	return IRQ_HANDLED;
}

/*==========================================================================
 * File operations
 *========================================================================*/

static int cim_open(struct inode *inode, struct file *filp);
static int cim_release(struct inode *inode, struct file *filp);
static ssize_t cim_read(struct file *filp, char *buf, size_t size, loff_t *l);
static ssize_t cim_write(struct file *filp, const char *buf, size_t size, loff_t *l);
static int cim_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int cim_mmap(struct file *file, struct vm_area_struct *vma);

static struct file_operations cim_fops = 
{
	open:		cim_open,
	release:	cim_release,
	read:		cim_read,
	write:		cim_write,
	ioctl:		cim_ioctl,
	mmap:		cim_mmap,

};

static int cim_open(struct inode *inode, struct file *filp)
{
	if (cim_inited == 0) {
		cim_device_init();
	}
	/* allocate frame buffers */
	cim_inited = 1;
	dprintk("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
 	try_module_get(THIS_MODULE);
	return 0;
}

static int cim_release(struct inode *inode, struct file *filp)
{
	dprintk("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	cim_fb_destroy();
	cim_stop();

 	module_put(THIS_MODULE);
	return 0;
}

static ssize_t cim_read(struct file *filp, char *buf, size_t size, loff_t *l)
{
	unsigned long off = *l; 
	if ((size + off) > jz_cim->mem_size)
		size = jz_cim->mem_size;
	memcpy(buf, jz_cim->mem_base + off, size);
	return size;
}

static ssize_t cim_write(struct file *filp, const char *buf, size_t size, loff_t *l)
{
	printk("cim error: write is not implemented\n");
	return -1;
}

/**************************
 *     IOCTL Handlers     *
 **************************/

/* 
 * If use default mem, app need trans a mem_base to cim though "IOCTL_SET_MEM".(only once)
 * Else driver will alloc memory by itself. See cim_fb_alloc() for detail.
 *
 * Then "IOCTL_SET_CIM_CONFIG" and "IOCTL_SET_PREVIEW_PARAM" will be call to set preview parametes.
 * Now, call IOCTL_START_CIM to start data tranfer.
 * When Take a picture, 
 *
 */
static int cim_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	switch (cmd) {
	case IOCTL_SET_I2C_ADDR:
		if (copy_from_user(&i2c_addr, (void *)arg, 4))
			return -EFAULT;
		break;
	case IOCTL_SET_I2C_CLK:
		if (copy_from_user(&i2c_clk, (void *)arg, 4))
			return -EFAULT;
		break;
	case IOCTL_WRITE_I2C_REG:
	{
		unsigned char regval[2];

		if (copy_from_user(regval, (void *)arg, 2))
			return -EFAULT;

		sensor_write_reg(regval[0], regval[1]);
		break;
	}
	case IOCTL_READ_I2C_REG:
	{
		unsigned char reg, val;

		if (copy_from_user(&reg, (void *)arg, 1))
			return -EFAULT;

		val = sensor_read_reg(reg);

		if (copy_to_user((void *)(arg + 1), &val, 1))
			return -EFAULT;
		break;
	}
	case IOCTL_WRITE_I2C_REG16:
	{
		unsigned short regval[2];

		if (copy_from_user(regval, (void *)arg, 4))
			return -EFAULT;

		sensor_write_reg16(regval[0], (unsigned char)regval[1]);
		break;
	}
	case IOCTL_READ_I2C_REG16:
	{
		unsigned short reg, val;

		if (copy_from_user(&reg, (void *)arg, 2))
			return -EFAULT;

		val = sensor_read_reg16(reg);

		if (copy_to_user((void *)(arg + 1), &val, 2))
			return -EFAULT;
		break;
	}
#ifdef USE_DEFAULT_MEM
	case IOCTL_SET_MEM:
		jz_cim->mem_base = (unsigned char *)arg;
		cim_fb_alloc();
		break;
#endif
	case IOCTL_START_CIM:
		cim_start();
		break;
	case IOCTL_STOP_CIM:
		cim_stop();
		return 0;
	case IOCTL_GET_CIM_CONFIG:
		dprintk("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
		return copy_to_user((void *)argp, (void *)&jz_cim->cim_cfg,
				    sizeof(cim_config_t)) ? -EFAULT : 0;
		break;

	case IOCTL_SET_CIM_CONFIG:
		if (copy_from_user((void *)&jz_cim->cim_cfg, (void *)arg,
				   sizeof(cim_config_t)))
			return -EFAULT;
		cim_config(&jz_cim->cim_cfg);
		break;
	case IOCTL_GET_PREVIEW_PARAM:
		dprintk("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
		return copy_to_user(argp, &jz_cim->view_par, sizeof(preview_param_t)) ? -EFAULT : 0;
		break;
	case IOCTL_SET_PREVIEW_PARAM:
	{
		int i, framesize, wpf; /* words per frame */
		preview_param_t p;


		if (copy_from_user((void *)&p, (void *)arg, sizeof(preview_param_t)))
			return -EFAULT;

		framesize = (p.width * p.height * p.bpp + 7) >> 3;
		if (framesize > MAX_PRE_SIZE){
			printk("ERROR! Preview size is too large!\n");
			return -EINVAL;
		}
		jz_cim->view_par.width = p.width;
		jz_cim->view_par.height = p.height;
		jz_cim->view_par.bpp = p.bpp;
		wpf = (framesize +3) >> 2 ;
		for (i = 0; i < CIM_BUF_NUM; i++) {
			cim_frame_desc[i].dmacmd &= ~CIM_CMD_LEN_MASK;
			cim_frame_desc[i].dmacmd |= wpf;
			dma_cache_wback((unsigned long)(&cim_frame_desc[i]), sizeof(struct cim_desc));
		}
		break;
	}
	case IOCTL_GET_PICTURE_PARAM:
		return copy_to_user(argp, &jz_cim->pic_par, sizeof(picture_param_t)) ? -EFAULT : 0;
		break;
	case IOCTL_SET_PICTURE_PARAM:
	{
		int framesize, wpf; /* words per frame */
		picture_param_t p;

		if (copy_from_user((void *)&p, (void *)arg, sizeof(picture_param_t)))
			return -EFAULT;
		framesize = (p.width * p.height * 16 + 7) >> 3;
		jz_cim->pic_par.width = p.width;
		jz_cim->pic_par.height = p.height;
		wpf = (framesize + 3) >> 2 ;
		cim_jpeg_desc.dmacmd &= ~CIM_CMD_LEN_MASK;
		cim_jpeg_desc.dmacmd |= wpf;
		dma_cache_wback((unsigned long)(&cim_jpeg_desc), sizeof(struct cim_desc));
		break;
	}
	case IOCTL_PRINT_REGS:
		cim_print_regs();
		break;
		
	case IOCTL_TAKE_PICTURE:
		cim_snapshot(0);
		break;
	case IOCTL_GET_CURRENT_BUF_ID:
	{
		int id;
		id = get_ready_buf_id();
		return copy_to_user(argp, &id, 4) ? -EFAULT : 0;
	}
		break;
	default:
		printk("Not supported command: 0x%x\n", cmd);
		return -EINVAL;
		break;
	}
	return 0;
}

/* Use mmap /dev/fb can only get a non-cacheable Virtual Address. */
static int cim_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long start;
	unsigned long off;
	u32 len;

	dprintk("%s, %s, %d\n", __FILE__, __FUNCTION__, __LINE__);
	off = vma->vm_pgoff << PAGE_SHIFT;

	/* frame buffer memory */
	start = virt_to_phys(jz_cim->mem_base);
	len = PAGE_ALIGN((start & ~PAGE_MASK) + ((unsigned long)jz_cim->mem_size));
	start &= PAGE_MASK;

	if ((vma->vm_end - vma->vm_start + off) > len) {
		printk("Error: vma is larger than memory length\n");
		return -EINVAL;
	}
	off += start;

	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);	/* Uncacheable */

#if  defined(CONFIG_MIPS32)
 	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
 	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;		/* Uncacheable */
#endif

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot)) {
		return -EAGAIN;
	}
	return 0;
}
static struct miscdevice cim_dev = {
	CIM_MINOR,
	"cim",
	&cim_fops
};

/*
 * Module init and exit
 */

static int __init cim_init(void)
{
	int ret;

	/* GPIO init for cim pins and i2c SDA & SCL */
	__gpio_as_cim();
	__gpio_as_i2c();

	/* waitqueue */
	init_waitqueue_head(&jz_cim->wait_queue);

#ifndef USE_DEFAULT_MEM
	/* Alloc memory for cim DMA*/

	printk("Alloc memory for cim DMA\n");
	ret = cim_fb_alloc();
	if (ret) {
		printk("No mem: Alloc memory for cim DMA\n");
		return ret;
	}
#endif
	/* request interrupt for cim */
	if ((ret = request_irq(IRQ_CIM, cim_irq_handler, IRQF_DISABLED, CIM_NAME, jz_cim))) {
		printk(KERN_ERR "request_irq return error, ret=%d\n", ret);
		cim_fb_destroy();
		printk(KERN_ERR "CIM could not get IRQ\n");
		return ret;
	}

	/* Register as a misc device */
	ret = misc_register(&cim_dev);
	if (ret < 0) {
		return ret;
	}

	printk("Virtual Driver of JZ CIM registered\n");
	return 0;
}

static void __exit cim_exit(void)
{
	misc_deregister(&cim_dev);
}

module_init(cim_init);
module_exit(cim_exit);

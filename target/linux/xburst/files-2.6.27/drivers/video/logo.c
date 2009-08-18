/* drivers/video/logo.c
 *
 * Show Logo in RLE 565 or RGB 565 format
 *
 * Copyright (C) 2008 Google Incorporated
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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fb.h>
#include <linux/vt_kern.h>
#include <linux/unistd.h>
#include <linux/syscalls.h>

#include <linux/irq.h>
#include <asm/system.h>

#define fb_width(fb)	((fb)->var.xres)
#define fb_height(fb)	((fb)->var.yres)
#define fb_size(fb)	((fb)->var.xres * (fb)->var.yres * 2)

void jzfb_get_panel_size(unsigned int *w, unsigned *h);

#ifdef CONFIG_FB_565RLE_LOGO
static void memset16(void *_ptr, unsigned short val, unsigned count)
{
	unsigned short *ptr = _ptr;
	count >>= 1;
	while (count--)
		*ptr++ = val;
}
/* 565RLE image format: [count(2 bytes), rle(2 bytes)] */
int display_fb_rle565(unsigned short *buf, unsigned count) {
	struct fb_info *info;
	unsigned max;
	int vm_width, vm_height, stride, ppl;
	unsigned short *bits, *ptr;
	info = registered_fb[0];
	if (!info) {
		printk(KERN_WARNING "%s: Can not access framebuffer\n",
			__func__);
		return -ENODEV;
	}

	jzfb_get_panel_size(&vm_width, &vm_height);

	max = vm_width * vm_height;
	stride = fb_width(info) - vm_width;
	ppl = fb_width(info); /*pixel per line*/
	ptr = (unsigned short *)buf;
	bits = (unsigned short *)(info->screen_base);
	if (vm_width < ppl) {
		while (count > 3) {
			unsigned n = ptr[0];
			unsigned h; 
			unsigned l;
			unsigned o;
			h = n < max%vm_width ? n : max%vm_width;
			l = (n-h) / vm_width;
			o = (n-h) % vm_width;
			if (n > max)
				break;
			/* data before total line */
			if (h) {
				memset16(bits, ptr[1], h<<1);
				bits += h;
				max -= h;
				if (max%vm_width == 0)
					bits += stride;
			}
			/* total lines */
			while(l--){
				memset16(bits, ptr[1], vm_width<<1);
				max -= vm_width;
				bits += ppl;
			}
			if (o) {
				/* data after total line */
				memset16(bits, ptr[1], o<<1);
				bits += o;
				max -= o;
				if (max%vm_width == 0)
					bits += stride;
			}
			ptr += 2;
			count -= 4;
		}
	}
	else {
		while (count > 3) {
			unsigned n = ptr[0];
			if (n > max)
				break;
			memset16(bits, ptr[1], n << 1);
			bits += n;
			max -= n;
			ptr += 2;
			count -= 4;
			}
	}
	return 0;
}
#endif

/* 565RGB image format: rgb565 */
#ifdef CONFIG_FB_565RGB_LOGO
int display_fb_rgb565(unsigned short *buf, unsigned count) {
	struct fb_info *info;
	unsigned max;
	int vm_width, vm_height, stride, ppl;
	unsigned short *bits, *ptr;
	info = registered_fb[0];
	if (!info) {
		printk(KERN_WARNING "%s: Can not access framebuffer\n",
			__func__);
		return -ENODEV;
	}

	jzfb_get_panel_size(&vm_width, &vm_height);

	max = vm_width * vm_height;
	stride = fb_width(info) - vm_width;
	ppl = fb_width(info); /*pixel per line*/
	ptr = (unsigned short *)buf;
	bits = (unsigned short *)(info->screen_base);
	if (vm_width < ppl) {
		while (vm_height--) {
			memcpy((void *)bits, (void *)ptr, vm_width<<1);
			bits += ppl;
			ptr += vm_width;
		}
	}
	else {
		memcpy((void *)bits, (void *)ptr, max<<1);
	}
	return 0;
}
#endif

int load_565_image(char *filename)
{
	int fd, err = 0;
	unsigned count;
	unsigned short *data;

	fd = sys_open(filename, O_RDONLY, 0);
	if (fd < 0) {
		printk(KERN_WARNING "%s: Can not open %s\n",
			__func__, filename);
		return -ENOENT;
	}
	count = (unsigned)sys_lseek(fd, (off_t)0, 2);
	if (count == 0) {
		sys_close(fd);
		err = -EIO;
		goto err_logo_close_file;
	}
	sys_lseek(fd, (off_t)0, 0);
	data = kmalloc(count, GFP_KERNEL);
	if (!data) {
		printk(KERN_WARNING "%s: Can not alloc data\n", __func__);
		err = -ENOMEM;
		goto err_logo_close_file;
	}
	if ((unsigned)sys_read(fd, (char *)data, count) != count) {
		err = -EIO;
		goto err_logo_free_data;
	}
#ifdef CONFIG_FB_565RLE_LOGO
	/* 565RLE image format: [count(2 bytes), rle(2 bytes)] */
	display_fb_rle565(data, count);
#endif
#ifdef CONFIG_FB_565RGB_LOGO
	/* 565RGB image format: rgb565 */
	display_fb_rgb565(data, count);
#endif


err_logo_free_data:
	kfree(data);
err_logo_close_file:
	sys_close(fd);
	return err;
}
EXPORT_SYMBOL(load_565_image);

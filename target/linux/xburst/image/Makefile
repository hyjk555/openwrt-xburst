#
# Copyright (C) 2009 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

JFFS2_BLOCKSIZE=256k 512k

ifneq ($(CONFIG_XBURST_UBOOT),)
  define Build/Clean
	$(MAKE) -C u-boot clean
  endef

  define Build/Compile
	$(MAKE) -C u-boot compile
  endef
endif

define Image/BuildKernel
	cp $(KDIR)/vmlinux.elf $(BIN_DIR)/openwrt-$(BOARD)-vmlinux.elf
	gzip -9 -c $(KDIR)/vmlinux > $(KDIR)/vmlinux.bin.gz
	$(STAGING_DIR_HOST)/bin/lzma e $(KDIR)/vmlinux $(KDIR)/vmlinux.bin.l7
	dd if=$(KDIR)/vmlinux.bin.l7 of=$(BIN_DIR)/openwrt-$(BOARD)-vmlinux.lzma bs=65536 conv=sync
	dd if=$(KDIR)/vmlinux.bin.gz of=$(BIN_DIR)/openwrt-$(BOARD)-vmlinux.gz bs=65536 conv=sync

	$(TARGET_CROSS)objcopy -O binary  -R .reginfo -R .mdebug -R .comment -R .note -R .pdr -R .options -R .MIPS.options -S $(LINUX_DIR)/vmlinux $(KDIR)/linux.bin
	rm -f $(KDIR)/linux.bin.gz
	gzip -9 $(KDIR)/linux.bin
	mkimage -A mips -O linux -T kernel -C gzip -a 0x80010000 -e 0x80010000 -n "Qi Kernel Image Ben Nanonote" -d $(KDIR)/linux.bin.gz uImage

	cp uImage $(BIN_DIR)/openwrt-$(BOARD)-uImage
endef

define Image/Build/squashfs
    $(call prepare_generic_squashfs,$(KDIR)/root.squashfs)
endef

define Image/Build
	$(call Image/Build/$(1))
	dd if=$(KDIR)/root.$(1) of=$(BIN_DIR)/openwrt-$(BOARD)-root.$(1) bs=128k conv=sync
endef

$(eval $(call BuildImage))

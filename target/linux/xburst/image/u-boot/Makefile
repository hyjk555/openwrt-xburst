#
# Copyright (C) 2009 Qi Hardware, Inc.
#
# This is free software, licensed under the GNU General Public License v3 
# or any later.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=u-boot
PKG_VERSION:=2009.08

PKG_BUILD_DIR:=$(KERNEL_BUILD_DIR)/$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.bz2
PKG_SOURCE_URL:=ftp://ftp.denx.de/pub/u-boot
PKG_CAT:=bzcat

include $(INCLUDE_DIR)/package.mk

UBOOT_CONFIG=$(strip $(subst ",, $(CONFIG_XBURST_UBOOT_TARGET)))

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) $(UBOOT_CONFIG)_config
	$(MAKE) -C $(PKG_BUILD_DIR) CROSS_COMPILE=$(TARGET_CROSS)
endef

define Build/InstallDev
	mkdir -p $(BIN_DIR)
	cp $(PKG_BUILD_DIR)/u-boot-nand.bin $(BIN_DIR)/openwrt-$(BOARD)-u-boot.bin
endef

$(eval $(call Build/DefaultTargets))

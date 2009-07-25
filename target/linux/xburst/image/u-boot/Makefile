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
PKG_VERSION:=20090707
PKG_REV:=59869ca72df8bc4e4ffa9dd17cb6673bbe010272
PKG_RELEASE:=1

PKG_BUILD_DIR:=$(KERNEL_BUILD_DIR)/$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION).tar.bz2
PKG_SOURCE_URL:=git://git.denx.de/u-boot.git
PKG_SOURCE_SUBDIR:=$(PKG_NAME)-$(PKG_VERSION)
PKG_SOURCE_VERSION:=$(PKG_REV)

include $(INCLUDE_DIR)/package.mk

UBOOT_CONFIG=$(strip $(subst ",, $(CONFIG_XBURST_UBOOT_TARGET)))

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) $(UBOOT_CONFIG)_config
	$(MAKE) -C $(PKG_BUILD_DIR) CROSS_COMPILE=$(TARGET_CROSS)
endef

define Build/InstallDev
	mkdir -p $(BIN_DIR)
	dd if=$(PKG_BUILD_DIR)/u-boot-nand.bin of=$(BIN_DIR)/u-boot-$(UBOOT_CONFIG).bin
endef

$(eval $(call Build/DefaultTargets))

#################################################################################
 # Ralink Technology, Inc.	                                         	#
 # 4F, No. 2 Technology 5th Rd.                                          	#
 # Science-based Industrial Park                                         	#
 # Hsin-chu, Taiwan, R.O.C.                                              	#
 #                                                                       	#
 # (c) Copyright 2005, Ralink Technology, Inc.                           	#
 #                                                                       	#
 # All rights reserved. Ralink's source code is an unpublished work and the	#
 # use of a copyright notice does not imply otherwise. This source code		#
 # contains confidential trade secret material of Ralink Tech. Any attempt	#
 # or participation in deciphering, decoding, reverse engineering or in any	#
 # way altering the source code is stricitly prohibited, unless the prior	#
 # written consent of Ralink Technology, Inc. is obtained.			#
#################################################################################


#
# Please specify CHIP & INTERFACE Type first
#
# CONFIG_CHIP_NAME = 2880 (PCI, MII)  
#                    3052 (USB, MII)
#                    3662 (USB, MII, PCI, PCIE)
#                    3883 (USB, MII, PCI, PCIE)                   
#                    3352 (USB, MII)
#                    5350 (USB, MII)
#
# CONFIG_INF_TYPE = PCI
#                   MII
#                   USB
#                   PCIE
#                   
#
CONFIG_CHIP_NAME = 3883
CONFIG_INF_TYPE=MII

#
# Please enable CONFIG_EXTRA_CFLAG=y on 2.6.25 or above 
#
CONFIG_EXTRA_CFLAG=y

#
# Please enable CONFIG_RALINK_SRC=y on 2.6.0 or above to configure host driver source code path
#
CONFIG_RALINK_SRC=y

#
# Feature Support
# Aggregation_Enable(USB only), PhaseLoadCode_Enable, RetryPktSend_Enable(MII only)
#
Aggregation_Enable=
ifeq ($(CONFIG_INF_TYPE), MII)
RetryPktSend_Enable=y
PhaseLoadCode_Enable=y
endif



CONFIG_NM_SUPPORT=y
CONFIG_MC_SUPPORT=
CONFIG_MESH_SUPPORT=
CONFIG_WIDI_SUPPORT=

#
# Chipset Related Feature Support
# CONFIG_CONCURRENT_INIC_SUPPORT (RT3883/RT3662 only)
# CONFIG_NEW_MBSS_SUPPORT (RT3883/RT3662 only)
#


ifeq ($(CONFIG_CHIP_NAME), $(filter 3662 3883, $(CONFIG_CHIP_NAME)))
CONFIG_CONCURRENT_INIC_SUPPORT=y
CONFIG_NEW_MBSS_SUPPORT=y
endif


#CONFIG_WOWLAN_SUPPORT
CONFIG_WOWLAN_SUPPORT=

# some platform not support 32-bit DMA addressing, need to turn this flag on
CONFIG_PCI_FORCE_DMA=


CC := $(CROSS_COMPILE)gcc
LD := $(CROSS_COMPILE)ld
AS := $(CROSS_COMPILE)as

_CFLAGS += -I$(RALINK_SRC)/comm
_CFLAGS += -D__KERNEL__ 
_CFLAGS += -O0 -g -Wall -Wstrict-prototypes -Wno-trigraphs 
_CFLAGS += -DDBG -DFIX_POTENTIAL_BUG #-DINBAND_DEBUG
_CFLAGS += -Wno-error=unused-variable
_CFLAGS += $(WFLAGS)

PCI_OBJS := pci/rt_pci_dev.o
MII_OBJS := mii/rt_mii_dev.o
USB_OBJS := usb/rt_usb_dev.o usb/rt_usb_fwupload.o
COMM_OBJS:= comm/rt_profile.o comm/raconfig.o comm/iwhandler.o \
            comm/ioctl.o comm/iwreq_stub.o comm/mbss.o comm/wds.o \
            comm/apcli.o comm/crc32.o 

obj-m := rt$(CONFIG_CHIP_NAME)_iNIC.o
rt$(CONFIG_CHIP_NAME)_iNIC-objs := $(COMM_OBJS)

ifeq ($(CONFIG_NM_SUPPORT), y)
_CFLAGS += -DNM_SUPPORT
endif

ifeq ($(CONFIG_MC_SUPPORT), y)
_CFLAGS += -DMULTIPLE_CARD_SUPPORT
endif

ifeq ($(CONFIG_CONCURRENT_INIC_SUPPORT), y)
_CFLAGS += -DCONFIG_CONCURRENT_INIC_SUPPORT
endif

ifeq ($(CONFIG_NEW_MBSS_SUPPORT), y)
_CFLAGS += -DNEW_MBSS_SUPPORT
endif

ifeq ($(CONFIG_WOWLAN_SUPPORT), y)
_CFLAGS += -DWOWLAN_SUPPORT
endif

_CFLAGS += -DCONFIG_CHIP_NAME=$(CONFIG_CHIP_NAME)

ifeq ($(CONFIG_INF_TYPE), PCIE)
rt$(CONFIG_CHIP_NAME)_iNIC-objs += $(PCI_OBJS)
_CFLAGS += -I$(RALINK_SRC)/pci -DCONFIG_INF_TYPE=INIC_INF_TYPE_PCI -DPCIE_RESET
endif

ifeq ($(CONFIG_INF_TYPE), PCI)
rt$(CONFIG_CHIP_NAME)_iNIC-objs += $(PCI_OBJS)
_CFLAGS += -I$(RALINK_SRC)/pci -DCONFIG_INF_TYPE=INIC_INF_TYPE_PCI
ifeq ($(CONFIG_CHIP_NAME), $(filter 2883, $(CONFIG_CHIP_NAME)))
_CFLAGS += -DPCI_NONE_RESET
endif
endif
 
ifeq ($(CONFIG_PCI_FORCE_DMA), y)
_CFLAGS += -DPCI_FORCE_DMA
endif

ifeq ($(CONFIG_INF_TYPE), MII)
rt$(CONFIG_CHIP_NAME)_iNIC-objs += $(MII_OBJS)
_CFLAGS += -I$(RALINK_SRC)/mii -DCONFIG_INF_TYPE=INIC_INF_TYPE_MII
ifeq ($(RetryPktSend_Enable), y)
_CFLAGS += -DRETRY_PKT_SEND
endif
#_CFLAGS += -DMII_SLAVE_STANDALONE
endif
 
ifeq ($(CONFIG_INF_TYPE), USB)
rt$(CONFIG_CHIP_NAME)_iNIC-objs += $(USB_OBJS)
_CFLAGS+= -I$(RALINK_SRC)/usb -DCONFIG_INF_TYPE=INIC_INF_TYPE_USB
ifeq ($(CONFIG_CHIP_NAME), $(filter 3662 3883 3352 5350, $(CONFIG_CHIP_NAME)))
_CFLAGS += -DRLK_INIC_USBDEV_GEN2
endif
ifeq ($(Aggregation_Enable), y)
ifeq ($(CONFIG_CHIP_NAME), $(filter 3662 3883 3352 5350, $(CONFIG_CHIP_NAME)))
_CFLAGS += -DRLK_INIC_TX_AGGREATION_ONLY
else
_CFLAGS += -DRLK_INIC_SOFTWARE_AGGREATION
endif
endif
else
ifeq ($(PhaseLoadCode_Enable), y)
_CFLAGS += -DPHASE_LOAD_CODE
endif
endif 

ifneq ($(PLATFORM_OBJ), )
_CFLAGS += -I$(RALINK_SRC)/platform
endif

ifeq ($(CONFIG_EXTRA_CFLAG), y)
EXTRA_CFLAGS += $(_CFLAGS)
else
CFLAGS += $(_CFLAGS)
endif


all: 
	$(MAKE) -C $(STAGING_DIR) ARCH=$(ARCH) SUBDIRS=$(PWD) modules

clean:
	rm -f $(COMM_OBJS) $(MII_OBJS) $(PCI_OBJS) $(PLATFORM_OBJS) $(USB_OBJS) $(MESH_OBJS) $(WIDI_OBJS) Module.symvers *~ .*.cmd *.map *.o *.ko *.mod.c comm/.*.cmd pci/.*.cmd mii/.*.cmd widi/.*.cmd usb/.*.cmd platform/.*.cmd *.order
	for d in $(subdir-m); \
    do                       \
      $(MAKE) --directory=$$d $@; \
    done



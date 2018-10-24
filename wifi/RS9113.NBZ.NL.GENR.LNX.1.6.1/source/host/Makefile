#/*
#* Copyright (c) 2017 Redpine Signals Inc. All rights reserved.
#*
#* Redistribution and use in source and binary forms, with or without
#* modification, are permitted provided that the following conditions are met:
#*
#* 	1. Redistributions of source code must retain the above copyright
#* 	   notice, this list of conditions and the following disclaimer.
#*
#* 	2. Redistributions in binary form must reproduce the above copyright
#* 	   notice, this list of conditions and the following disclaimer in the
#* 	   documentation and/or other materials provided with the distribution.
#*
#* 	3. Neither the name of the copyright holder nor the names of its
#* 	   contributors may be used to endorse or promote products derived from
#* 	   this software without specific prior written permission.
#*
#* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#* INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#* POSSIBILITY OF SUCH DAMAGE.
#*/


ROOT_DIR:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

include $(ROOT_DIR)/.config
include $(ROOT_DIR)/config/make.config

COMMON_HAL_DRV:=common_hal
COMMON_HAL_OSI:=$(COMMON_HAL_DRV)/osi/
COMMON_HAL_OSD:=$(COMMON_HAL_DRV)/osd/

SDIO_HOST_INTF_OSD_DIR:=$(COMMON_HAL_DRV)/intfdep/sdio/osd_sdio/linux
SDIO_HOST_INTF_OSI_DIR:=$(COMMON_HAL_DRV)/intfdep/sdio/osi_sdio
USB_HOST_INTF_OSD_DIR:=$(COMMON_HAL_DRV)/intfdep/usb/osd_usb/linux
USB_HOST_INTF_OSI_DIR:=$(COMMON_HAL_DRV)/intfdep/usb/osi_usb


WLAN_DRV:=$(ROOT_DIR)/wlan
IEEE80211_DIR:=wlan/net80211/linux
WLAN_HAL_DIR:=wlan/wlan_hal
WLAN_DEVDEP_DIR:=wlan/wlan_hal/osi_wlan/devdep/rsi_9113
WLAN_CORE_DIR:=wlan/wlan_hal/osi_wlan/core
WLAN_OSD_DIR:=wlan/wlan_hal/osd_wlan/linux
WLAN_IEEE80211_OSI:=wlan/net80211/linux/osi_net80211/src
WLAN_IEEE80211_OSD_DIR:=wlan/net80211/linux/osd_linux/src

BT_DRV:=$(ROOT_DIR)/bt
BT_OSI_DIR:=bt/osi_bt
BT_OSD_DIR:=bt/osd_bt/linux

ZIGB_DRV:=$(ROOT_DIR)/zigbee
ZIGB_OSI_DIR := zigbee/osi_zigb
ZIGB_OSD_DIR := zigbee/osd_zigb/linux

WLAN_SUPP_DIR:=$(ROOT_DIR)/wlan/supplicant/linux
WLAN_HOSTAPD_DIR:=$(ROOT_DIR)/wlan/hostapd-2.4/hostapd

UTILS_DIR:=$(ROOT_DIR)/utils/

DEST_DIR=$(ROOT_DIR)/release
MOD_DIR=$(ROOT_DIR)/mod_files/

CC=$(CROSS_COMPILE)gcc

DEF_KERNEL_DIR := $(ROOT_DIR)/kernel_modules_headers

ifeq ($(KERNELDIR),)
	KERNELDIR := $(DEF_KERNEL_DIR)
endif

EXTRA_CFLAGS += -DLINUX -Wimplicit -Wstrict-prototypes

EXTRA_CFLAGS += -DFLASH_BURNING
EXTRA_CFLAGS += -DRSI_SDIO_MULTI_BLOCK_SUPPORT
EXTRA_CFLAGS += -DDYNAMIC_VARIABLES
EXTRA_CFLAGS += -DRF_8111

EXTRA_CFLAGS += -I$(ROOT_DIR)/$(COMMON_HAL_DRV)/include/linux/
EXTRA_CFLAGS += -I$(ROOT_DIR)/$(COMMON_HAL_DRV)/include/common/

################WLAN COMPILE FLAGS ##################################
EXTRA_CFLAGS += -DINET6
EXTRA_CFLAGS += -Dnet80211_s
EXTRA_CFLAGS += -DBGSCAN_SUPPORT
EXTRA_CFLAGS += -DPWR_SAVE_SUPPORT
EXTRA_CFLAGS += -DACM_NO_TSPEC_CNFM
EXTRA_CFLAGS += -DBYPASS_TX_DATA_PATH
EXTRA_CFLAGS += -DBYPASS_RX_DATA_PATH
EXTRA_CFLAGS += -DIEEE80211_AMPDU_AGE
EXTRA_CFLAGS += -DSECURITY_SUPPORT
EXTRA_CFLAGS += -DAMPDU_AGGR_SUPPORT

# Enable or disable below option in make.config
ifeq ($(ENABLE_P2P_SUPPORT), y)
	EXTRA_CFLAGS += -DENABLE_P2P_SUPPORT
endif

# Enable or disable below option in make.config
ifeq ($(RSI_CCX), y)
EXTRA_CFLAGS += -DRSI_CCX
endif

EXTRA_CFLAGS += -DUSE_WORKQUEUES
#EXTRA_CFLAGS += -DUSE_INTCONTEXT
EXTRA_CFLAGS += -DCHANNEL_UTILIZATION
#EXTRA_CFLAGS += -DDISABLE_DFS_CHANNELS
#EXTRA_CFLAGS += -DCHIP_9116
#EXTRA_CFLAGS += -DMODE_11AH
#EXTRA_CFLAGS += -DIEEE80211K
#EXTRA_CFLAGS += -DRADAR_AUTO
#EXTRA_CFLAGS += -DCONFIG_11W
#EXTRA_CFLAGS += -DCONFIG_11R
#EXTRA_CFLAGS += -DSDIO_CMD52_RESET
#EXTRA_CFLAGS += -D SEPERATE_BROADCAST_QUEUE
#EXTRA_CFLAGS += -DCONFIG_FPGA_40MHZ
#EXTRA_CFLAGS += -DUART_DEBUG
#EXTRA_CFLAGS += -DDUTY_CYCLE_IOCTL
#EXTRA_CFLAGS += -DCONFIG_ACS
EXTRA_CFLAGS += -DONEBOX_CONFIG_WOWLAN
#EXTRA_CFLAGS += -DONEBOX_CONFIG_GTK_OFFLOAD
#EXTRA_CFLAGS += -DONEBOX_CONFIG_PUF
EXTRA_CFLAGS += -I$(ROOT_DIR)/$(IEEE80211_DIR)/osd_linux/include/
EXTRA_CFLAGS += -I$(ROOT_DIR)/$(IEEE80211_DIR)/osi_net80211/net80211/
EXTRA_CFLAGS += -I$(ROOT_DIR)/$(IEEE80211_DIR)/osi_net80211/
EXTRA_CFLAGS += -I$(ROOT_DIR)/wlan/wlan_hal/include/linux
#####################END OF WLAN FLAGS ###############################

################ BT COMPILE FLAGS ##################################
EXTRA_CFLAGS += -DUSE_BLUEZ_BT_STACK
EXTRA_CFLAGS += -I$(ROOT_DIR)/bt/include/linux
#####################END OF BT FLAGS ###############################

################ ZIGBEE COMPILE FLAGS ##################################
EXTRA_CFLAGS += -I$(ROOT_DIR)/zigbee/include/linux
#####################END OF ZIGB FLAGS ###############################

EXTRA_CFLAGS += -g

DRV_DIR=$(ROOT_DIR)/


ifeq ($(USE_USB),"Y")
EXTRA_CFLAGS += -DUSE_USB_INTF
HOST_INTF_OSI_OBJ := $(USB_HOST_INTF_OSI_DIR)/onebox_usb_main_osi.o        \
										 $(USB_HOST_INTF_OSI_DIR)/onebox_host_intf_osi_callbacks.o 
HOST_INTF_OSD_OBJ := $(USB_HOST_INTF_OSD_DIR)/onebox_usb_main_osd.o        \
											 $(USB_HOST_INTF_OSD_DIR)/onebox_host_intf_osd_callbacks.o
endif

ifeq ($(USE_SDIO),"Y")
EXTRA_CFLAGS += -DUSE_SDIO_INTF
EXTRA_CFLAGS += -DSDIO_COMMANDS
HOST_INTF_OSI_OBJ += $(SDIO_HOST_INTF_OSI_DIR)/onebox_sdio_main_osi.o      \
										 $(SDIO_HOST_INTF_OSI_DIR)/onebox_host_intf_osi_callbacks.o 

HOST_INTF_OSD_OBJ +=  $(SDIO_HOST_INTF_OSD_DIR)/onebox_sdio_main_osd.o      \
										  $(SDIO_HOST_INTF_OSD_DIR)/onebox_host_intf_osd_callbacks.o 
endif

COMMON_NON_GPL_OBJ := $(COMMON_HAL_OSI)onebox_common_tx.o \
											$(COMMON_HAL_OSI)onebox_common_rx.o        \
											$(COMMON_HAL_OSI)onebox_common_init.o      \
											$(COMMON_HAL_OSI)onebox_flashing.o         \
											$(COMMON_HAL_OSI)checksum_onescompliment.o \
											$(COMMON_HAL_OSI)onebox_common_devops.o    \
											$(COMMON_HAL_OSI)onebox_common_callbacks.o \
											$(COMMON_HAL_OSI)onebox_common_pwr_mngr.o \
											$(COMMON_HAL_OSI)onebox_common_dev_config_params.o

COMMON_NON_GPL_OBJ += $(COMMON_HAL_OSI)onebox_sdio_devops.o
COMMON_NON_GPL_OBJ += $(COMMON_HAL_OSI)onebox_usb_devops.o


COMMON_GPL_OBJ := $(COMMON_HAL_OSD)linux/onebox_osd_ops.o       \
									$(COMMON_HAL_OSD)linux/onebox_osd_callbacks.o \
									$(COMMON_HAL_OSD)linux/onebox_file_ops.o      \
									$(COMMON_HAL_OSD)linux/onebox_thread.o        \
									$(COMMON_HAL_OSD)linux/onebox_private.o       \
									$(COMMON_HAL_OSD)linux/onebox_proc.o          \
									$(COMMON_HAL_OSD)linux/onebox_osd_callbacks.o \
	
ONEBOX_GPL_OBJ :=	$(COMMON_HAL_OSD)linux/onebox_modules_init.o  



wlan_wep-objs   := $(WLAN_IEEE80211_OSI)/ieee80211_crypto_wep.o
wlan_tkip-objs   := $(WLAN_IEEE80211_OSI)/ieee80211_crypto_tkip.o
wlan_scan_sta-objs   := $(WLAN_IEEE80211_OSI)/ieee80211_scan_sta.o
wlan_ccmp-objs   := $(WLAN_IEEE80211_OSI)/ieee80211_crypto_ccmp.o
wlan_xauth-objs   := $(WLAN_IEEE80211_OSI)/ieee80211_xauth.o
wlan_aes_cmac-objs := $(WLAN_IEEE80211_OSI)/ieee80211_crypto_bip.o

wlan_acl-objs   := $(WLAN_IEEE80211_OSI)/ieee80211_acl.o
wlan-objs := $(WLAN_IEEE80211_OSI)/ieee80211.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_linux.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_ioctl.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_node.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_action.o       \
						 $(WLAN_IEEE80211_OSI)/ieee80211_adhoc.o        \
						 $(WLAN_IEEE80211_OSI)/ieee80211_ageq.o        \
						 $(WLAN_IEEE80211_OSI)/ieee80211_crypto.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_crypto_none.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_dfs.o  \
						 $(WLAN_IEEE80211_OSI)/ieee80211_quiet.o  \
						 $(WLAN_IEEE80211_OSI)/ieee80211_ht.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_mfp.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_input.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_mesh.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_monitor.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_phy.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_proto.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_ratectl.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_regdomain.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_wds.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_scan.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_sta.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_superg.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_hostap.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_radiotap.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_power.o \
						 $(WLAN_IEEE80211_OSI)/ieee80211_output.o\
						 $(WLAN_IEEE80211_OSI)/ieee80211_11k.o\
						 $(WLAN_IEEE80211_OSD_DIR)/mbuf.o \
						 $(WLAN_IEEE80211_OSD_DIR)/sysctl.o \
						 $(WLAN_IEEE80211_OSD_DIR)/ndiface.o \
						 $(WLAN_IEEE80211_OSD_DIR)/ioctl.o \
						 $(WLAN_IEEE80211_OSD_DIR)/linux.o \
						 $(WLAN_IEEE80211_OSD_DIR)/cfg80211_ioctl.o \
						 $(WLAN_IEEE80211_OSD_DIR)/cfg80211_wrapper.o \

ifeq ($(ENABLE_P2P_SUPPORT), y)
	wlan-objs += $(WLAN_IEEE80211_OSI)/ieee80211_p2p.o
endif

WLAN_DEVDEP_OBJ := $(WLAN_DEVDEP_DIR)/onebox_wlan_osi_init.o \
									 $(WLAN_DEVDEP_DIR)/onebox_devdep_wlan_callbacks.o \
									 $(WLAN_DEVDEP_DIR)/onebox_dev_ops.o \
									 $(WLAN_DEVDEP_DIR)/onebox_ps.o \
									 $(WLAN_DEVDEP_DIR)/onebox_devdep_mgmt.o \
									 $(WLAN_DEVDEP_DIR)/onebox_pktpro.o    \
									 $(WLAN_DEVDEP_DIR)/onebox_debug.o    \
									 $(WLAN_DEVDEP_DIR)/onebox_reorder.o \
									 $(WLAN_DEVDEP_DIR)/onebox_radar.o	

WLAN_CORE_OBJ := $(WLAN_CORE_DIR)/onebox_core_wlan.o  \
								 $(WLAN_CORE_DIR)/onebox_wlan_per.o  \
								 $(WLAN_CORE_DIR)/onebox_core_wlan_callbacks.o  \
								 $(WLAN_CORE_DIR)/onebox_core_hal_intf.o   \
								 $(WLAN_CORE_DIR)/onebox_core_os_intf.o \
								 $(WLAN_CORE_DIR)/onebox_core_autorate.o  \
								 $(WLAN_CORE_DIR)/onebox_core_vap.o \
								 $(WLAN_CORE_DIR)/onebox_core_wmm.o \
								 $(WLAN_CORE_DIR)/onebox_net80211_callbacks.o

WLAN_OSD_OBJ := $(WLAN_OSD_DIR)/onebox_wlan_ioctl.o  \
								$(WLAN_OSD_DIR)/onebox_wlan_osd_init.o \
								$(WLAN_OSD_DIR)/onebox_wlan_osd_ops.o \
								$(WLAN_OSD_DIR)/onebox_wlan_osd_callbacks.o \
								$(WLAN_OSD_DIR)/onebox_wlan_proc.o 

BT_OSI_OBJ := $(BT_OSI_DIR)/onebox_bt_osi_init.o \
							$(BT_OSI_DIR)/onebox_bt_rx.o \
							$(BT_OSI_DIR)/onebox_bt_tx.o \
							$(BT_OSI_DIR)/onebox_bt.o \
							$(BT_OSI_DIR)/onebox_bt_osi_callbacks.o 

BT_OSD_OBJ := $(BT_OSD_DIR)/onebox_osd_bluez.o \
							$(BT_OSD_DIR)/onebox_bt_nl.o \
							$(BT_OSD_DIR)/onebox_bt_osd_init.o \
							$(BT_OSD_DIR)/onebox_bt_osd_callbacks.o \
							$(BT_OSD_DIR)/onebox_bt_proc.o  

ZIGB_OSI_OBJ := $(ZIGB_OSI_DIR)/onebox_zigb_osi_init.o \
								$(ZIGB_OSI_DIR)/onebox_zigb_rx.o \
								$(ZIGB_OSI_DIR)/onebox_zigb_tx.o \
								$(ZIGB_OSI_DIR)/onebox_zigb.o \
								$(ZIGB_OSI_DIR)/onebox_zigb_osi_callbacks.o \

ZIGB_OSD_OBJ := $(ZIGB_OSD_DIR)/onebox_zigb_ioctl.o \
								$(ZIGB_OSD_DIR)/onebox_zigb_nl.o \
								$(ZIGB_OSD_DIR)/onebox_zigb_osd_init.o \
								$(ZIGB_OSD_DIR)/onebox_zigb_osd_callbacks.o \
								$(ZIGB_OSD_DIR)/onebox_zigb_proc.o 

ifeq ($(RSI_CONFIG_ANDROID), y)
EXTRA_CFLAGS += -DRSI_CONFIG_ANDROID
BT_OSD_OBJ += $(BT_OSD_DIR)/onebox_osd_bluedroid.o
endif

BT_NONGPL_OBJS :=  $(BT_OSI_OBJ)
BT_GPL_OBJS := $(BT_OSD_OBJ)

obj-m := onebox_bt_gpl.o
onebox_bt_gpl-objs := $(BT_GPL_OBJS)

obj-m += onebox_bt_nongpl.o
onebox_bt_nongpl-objs := $(BT_NONGPL_OBJS)

ZIGB_GPL_OBJS := $(ZIGB_OSD_OBJ)
obj-m := onebox_zigb_gpl.o
onebox_zigb_gpl-objs := $(ZIGB_GPL_OBJS)

ZIGB_NONGPL_OBJS :=  $(ZIGB_OSI_OBJ)
obj-m += onebox_zigb_nongpl.o
onebox_zigb_nongpl-objs := $(ZIGB_NONGPL_OBJS)

GPL_OBJS := $(COMMON_GPL_OBJ)   
obj-m += onebox_common_gpl.o
onebox_common_gpl-objs := $(GPL_OBJS)

obj-m += onebox_gpl.o
onebox_gpl-objs := $(ONEBOX_GPL_OBJ) $(HOST_INTF_OSD_OBJ)

NONGPL_OBJS :=  $(COMMON_NON_GPL_OBJ) $(HOST_INTF_OSI_OBJ) 
obj-m += onebox_nongpl.o
onebox_nongpl-objs := $(NONGPL_OBJS) 

WLAN_GPL_OBJS := $(WLAN_OSD_OBJ)
obj-m := onebox_wlan_gpl.o
onebox_wlan_gpl-objs := $(WLAN_GPL_OBJS)

WLAN_NONGPL_OBJS :=  $(WLAN_DEVDEP_OBJ) $(WLAN_CORE_OBJ)
obj-m = onebox_wlan_nongpl.o
onebox_wlan_nongpl-objs := $(WLAN_NONGPL_OBJS)

COMMON_MOD_INSTALL := 	onebox_common_gpl.o \
												onebox_gpl.o \
												onebox_nongpl.o  

WLAN_MOD_INSTALL  :=		wlan.o \
				wlan_wep.o \
				wlan_tkip.o \
				wlan_ccmp.o  \
				wlan_xauth.o \
				wlan_scan_sta.o \
				wlan_acl.o \
				wlan_aes_cmac.o \
				onebox_wlan_nongpl.o  \
				onebox_wlan_gpl.o  

BT_MOD_INSTALL := onebox_bt_gpl.o \
									onebox_bt_nongpl.o

ZIGB_MOD_INSTALL := onebox_zigb_gpl.o \
									onebox_zigb_nongpl.o

MODULES_TO_INSTALL := $(COMMON_MOD_INSTALL)

ifeq ($(COMPILE_WLAN), "Y")
MODULES_TO_INSTALL += $(WLAN_MOD_INSTALL)
endif

ifneq ($(RSI_CONFIG_ANDROID), y)
ifeq ($(COMPILE_WLAN), "Y")
ifeq ($(COMPILE_HOSTAPD), "y")
COMPILE_HOSTAPD=hostapd
endif
WLAN_COMPILE_FLAGS=wpa_supplicant $(COMPILE_HOSTAPD)
endif
UTIL_COMPILE_FLAGS=onebox_utils
endif


ifeq ($(COMPILE_BT), "Y")
MODULES_TO_INSTALL += $(BT_MOD_INSTALL)
endif

ifeq ($(COMPILE_ZIGB), "Y")
MODULES_TO_INSTALL += $(ZIGB_MOD_INSTALL)
endif

obj-m := $(MODULES_TO_INSTALL) 

all: compile_driver $(WLAN_COMPILE_FLAGS) $(UTIL_COMPILE_FLAGS)

compile_driver: 
	@echo -e "\033[32mCompiling Onebox Mobile Driver...\033[0m"
	make EXTRA_CFLAGS="$(EXTRA_CFLAGS)" -C $(KERNELDIR) SUBDIRS=$(ROOT_DIR) modules 
	cp *.ko $(DEST_DIR)        
	mv *.ko *.o *.mod.c *.order *.symvers $(MOD_DIR)
	$(CROSS_COMPILE)strip --strip-unneeded $(DEST_DIR)/*.ko 
	@echo -e "\033[32mCompilation done SUCCESSFULLY\033[0m"

onebox_utils:
	@echo -e "\033[32mCompiling onebox utils...\033[0m"
	make CC="$(CC)" ROOT_DIR=$(ROOT_DIR) -C $(UTILS_DIR)/

wpa_supplicant:
	@echo -e "\033[32mCompiling wpa supplicant...\033[0m"
	make CC="$(CC)" ROOT_DIR=$(ROOT_DIR) -C $(WLAN_SUPP_DIR)/wpa_supplicant
	cp $(WLAN_SUPP_DIR)/wpa_supplicant/wpa_supplicant $(DEST_DIR)
	cp $(WLAN_SUPP_DIR)/wpa_supplicant/wpa_cli $(DEST_DIR)

hostapd:
	@echo -e "\033[32mCompiling hostapd...\033[0m"
	make CC="$(CC)" -C $(WLAN_HOSTAPD_DIR)
	cp $(WLAN_HOSTAPD_DIR)/hostapd $(DEST_DIR)
	cp $(WLAN_HOSTAPD_DIR)/hostapd_cli $(DEST_DIR)

menuconfig: 
	$(MAKE) -C $(ROOT_DIR)/config/lxdialog lxdialog
	$(CONFIG_SHELL) $(ROOT_DIR)/config/Menuconfig config/config.in

compile_clean:
	@echo "- Cleaning All Objects and Intermediate Files"
	@find . -name '*.o' | xargs rm -f
	@find . -name '.*.ko.cmd' | xargs rm -rf
	@find . -name '.*.ko.unsigned.cmd' | xargs rm -rf
	@find . -name '*.ko.*' | xargs rm -rf
	@find . -name '.*.o.cmd' | xargs rm -rf
	@find . -name '*.mod.c' | xargs rm -rf
	@find . -name '.tmp_versions' | xargs rm -rf
	@find . -name '*.markers' | xargs rm -rf
	@find . -name '*.symvers' | xargs rm -rf
	@find . -name '*.order' | xargs rm -rf
	@find . -name '*.d' | xargs rm -rf

clean:
	@echo "- Cleaning All Object and Intermediate Files"
	@find . -name '*.ko' | xargs rm -rf
	@find . -name '*.o' | xargs rm -f
	@find . -name '.*.ko.cmd' | xargs rm -rf
	@find . -name '.*.ko.unsigned.cmd' | xargs rm -rf
	@find . -name '*.ko.*' | xargs rm -rf
	@find . -name '.*.o.cmd' | xargs rm -rf
	@find . -name '*.mod.c' | xargs rm -rf
	@find . -name '.tmp_versions' | xargs rm -rf
	@find . -name '*.markers' | xargs rm -rf
	@find . -name '*.symvers' | xargs rm -rf
	@find . -name '*.order' | xargs rm -rf
	@find . -name '*.d' | xargs rm -rf
	@find . -name 'wpa_priv' | xargs rm -rf
	@find . -name 'onebox_util' | xargs rm -rf
	@find . -name 'onebox_util' | xargs rm -rf
	@find . -name 'nl80211_util' | xargs rm -rf
	@find . -name 'bbp_util' | xargs rm -rf
	@find . -name 'transmit' | xargs rm -rf
	@find . -name 'transmit_packet' | xargs rm -rf
	@find . -name 'receive' | xargs rm -rf
	@find . -name 'sniffer_app' | xargs rm -rf
	@find . -name 'CVS' | xargs rm -rf
	@find . -name '*.swp' | xargs rm -rf
	@find . -name 'bt_receive' | xargs rm -rf
	@find . -name 'bt_util' | xargs rm -rf
	@find . -name 'bt_freq_hop' | xargs rm -rf
	@find . -name 'bt_ber' | xargs rm -rf
	@find . -name 'bt_stats' | xargs rm -rf
	@find . -name 'bt_transmit' | xargs rm -rf
	@find . -name 'bt_bbp_utils' | xargs rm -rf
	@find . -name 'ble_aoa_aod_receive' | xargs rm -rf
	@find . -name 'ble_aoa_aod_transmit' | xargs rm -rf
	@find . -name 'ble_receive' | xargs rm -rf
	@find . -name 'ble_transmit' | xargs rm -rf
	@find . -name 'bt_br_edr_receive' | xargs rm -rf
	@find . -name 'bt_br_edr_transmit' | xargs rm -rf
	@find . -name 'doth_over_ride' | xargs rm -rf
	@find . -name 'wlan_iqs' | xargs rm -rf
	@find . -name 'zb_transmit' | xargs rm -rf
	@find . -name 'zb_util' | xargs rm -rf
	@find . -name 'zb_bbp_util' | xargs rm -rf
	@find . -name 'append_pwr_values' | xargs rm -rf
	@find . -name 'START_RS9116_NBZ_D0N' | xargs rm -rf
	@rm -rf $(WLAN_SUPP_DIR)/core *~ *.o eap_*.so $(ALL) $(WINALL) eapol_test preauth_test
	@rm -rf $(WLAN_SUPP_DIR)/wpa_supplicant/wpa_supplicant 
	@rm -rf $(WLAN_SUPP_DIR)/wpa_supplicant/wpa_cli 
	@rm -rf $(WLAN_SUPP_DIR)/wpa_supplicant/wpa_gui 
	@rm -rf $(DEST_DIR)/wpa_supplicant 
	@rm -rf $(WLAN_SUPP_DIR)/wpa_supplicant/wpa_passphrase
	@rm -rf $(DEST_DIR)/wpa_cli 
	@rm -rf $(DEST_DIR)/wpa_gui
	@rm -rf $(WLAN_HOSTAPD_DIR)/core *~ *.o eap_*.so $(ALL) $(WINALL) eapol_test preauth_test
	@rm -rf $(WLAN_HOSTAPD_DIR)/hostapd 
	@rm -rf $(WLAN_HOSTAPD_DIR)/hostapd_cli 
	@rm -rf $(WLAN_HOSTAPD_DIR)/hostapd_passphrase
	@rm -rf $(DEST_DIR)/hostapd
	@rm -rf $(DEST_DIR)/hostapd_cli 
	@echo "- Done"

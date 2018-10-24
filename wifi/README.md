# Redpine Driver

This folder contains Redpine's proprietary driver for the RS9113 wireless modules. Listed below are all the steps Maidbot took to obtain, modify and tweak this driver to work with our system.

1. Log in to www.redpinenetworks.us/OpenKM with the following credentials:  
   User ID: maidbot.us  
   Password: thinkoutsidethebot

1. Navigate to `RS9113 Combo Modules > n-Link > Software > Linux` and download the latest `RS9113.NBZ.NL.GENR.LNX.x.x.x.tgz`.

1. Unzip `RS9113.NBZ.NL.GENR.LNX.x.x.x.tgz` into this folder.

1. Modify the `../Dockerfile` (line 23 as of this writing) to reflect the name of the unzipped folder.

1. Modify the `../install/launch.bash` (line 6 as of this writing) to reflect the name of the unzipped folder.

1. Open `Makefile` in `RS9113.NBZ.NL.GENR.LNX.x.x.x/source/host` and:  
   Replace: `DEF_KERNEL_DIR := /lib/modules/$(shell uname -r)/build`  
   With: `DEF_KERNEL_DIR := $(ROOT_DIR)/kernel_modules_headers`

1. Open `.config` in `RS9113.NBZ.NL.GENR.LNX.x.x.x/source/host` and replace the text with:
   ```
   ENABLE_USB=y
   ENABLE_SDIO=y
   RSI_CONFIG_LINUX=y
   # RSI_CONFIG_ANDROID is not set
   ONEBOX_CONFIG_NL80211=y
   # HOSTAPD_SUPPORT is not set
   ENABLE_WLAN=y
   ENABLE_BT=y
   ENABLE_ZIGB=y
   ```

1. Open `.config` in `RS9113.NBZ.NL.GENR.LNX.x.x.x/source/host/wlan/supplicant/linux/wpa_supplicant` and:  
   Uncomment line 32 (`CONFIG_DRIVER_NL80211=y`)  
   Uncomment line 44 (`CONFIG_LIBNL32=y`)  
   Replace line 37 with: `CFLAGS += -I/usr/include/libnl3`

1. Open `start_sta.sh` in `RS9113.NBZ.NL.GENR.LNX.x.x.x/source/host/release` and comment out the last line (the one that launches `wpa_supplicant`)

1. Add an empty file `empty.txt` in `RS9113.NBZ.NL.GENR.LNX.x.x.x/source/host/mod_files`

1. Open `common_insert.sh` in `RS9113.NBZ.NL.GENR.LNX.x.x.x/source/host/release` and set `COEX_MODE` to `1`

1. Open `common_insert.sh` in `RS9113.NBZ.NL.GENR.LNX.x.x.x/source/host/release` and set `SDIO_CLOCK_SPEED` to `5000`  

1. Open `onebox_sdio_main_osd.c` in `RS9113.NBZ.NL.GENR.LNX.x.x.x/source/host/common_hal/intfdep/sdio/osd_sdio/linux` and modify line 718 to `				clock = 5000000;`

1. Open `common_insert.sh` in `RS9113.NBZ.NL.GENR.LNX.x.x.x/source/host/release` and set `firmware_path` to `/mnt/data/docker/volumes/${RESIN_APP_ID}_resin-data/_data/firmware/`

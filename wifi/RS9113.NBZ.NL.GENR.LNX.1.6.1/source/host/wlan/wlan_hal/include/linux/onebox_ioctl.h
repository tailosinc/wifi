/*
 * Copyright (c) 2017 Redpine Signals Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	1. Redistributions of source code must retain the above copyright
 * 	   notice, this list of conditions and the following disclaimer.
 *
 * 	2. Redistributions in binary form must reproduce the above copyright
 * 	   notice, this list of conditions and the following disclaimer in the
 * 	   documentation and/or other materials provided with the distribution.
 *
 * 	3. Neither the name of the copyright holder nor the names of its
 * 	   contributors may be used to endorse or promote products derived from
 * 	   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ONEBOX_AP_IOCTL_H__
#define __ONEBOX_AP_IOCTL_H__

#define ONEBOX_VAP_CREATE        SIOCIWLASTPRIV - 0xf  //: check for exact value b4 execution
#define ONEBOX_VAP_DELETE        SIOCIWLASTPRIV - 0x10
#define ONEBOX_HOST_IOCTL        SIOCIWLASTPRIV - 0x0B
#define RSI_WATCH_IOCTL          SIOCIWLASTPRIV - 0xd 
#define WLAN_I_Q_STATS           SIOCIWLASTPRIV - 0xc 
#define SET_BEACON_INVL          18
#define SET_BGSCAN_PARAMS        19
#define DO_BGSCAN                20
#define BGSCAN_SSID              21
#define EEPROM_READ_IOCTL        22
#define EEPROM_WRITE_IOCTL       23
#define PS_REQUEST               25
#define UAPSD_REQ                26
#define RESET_ADAPTER            32
#define RESET_PER_Q_STATS        33 
#define AGGR_LIMIT               34
#define RX_FILTER		35
#define MASTER_READ		36
#define MASTER_WRITE		37
#define TEST_MODE		38
#define RF_PWR_MODE		39
#define SET_COUNTRY 40
#define GET_INFO 41
#define SET_SCAN_TYPE 42
#define SET_WOWLAN_CONFIG 43
#define SET_EXT_ANT_GAIN 44
#define ADD_APPLE_IE 46
#define IAP_INIT	47
#define IAP_MFI_CHALLENGE	48
#define IAP_READ_CERTIFICATE	49
#define DRV_PARAMS		50
#define CONF_BEACON_RECV		51
#define GET_TXPOWER		52
#define SET_HOST_SCAN		53
#define CHECK_STA_STATE	54
#define ENABLE_MAX_POWER	55
#define SPECTRAL_MASK   56
#define RSI_USEONLY_RATES      57
#ifdef IEEE80211K
#define SET_MSRMNT_PARAMS 58
#endif
#ifdef ONEBOX_CONFIG_GTK_OFFLOAD
#define GTK_OFFLOAD   59
#endif
#define PUF_REQUEST	  60
#define GPIO_R_W                       63
#define COEX_SLOT_PARAMS 67              
#define WLAN_9116_FEATURE 68 
#define LOG_STRUCT_PRGMG 69 
#define PROTOCOL_RF_WRITE	             70
#define PROTOCOL_RF_READ	             71
#define DISABLE_PROGRAMMING            72
#define PROG_STRUCTURE                 73
#define IPMU_REG                      74

#define ONEBOX_SET_BB_RF         SIOCIWLASTPRIV - 0x08 
#define ONEBOX_SET_CW_MODE       SIOCIWLASTPRIV - 0x05 

/* To reset the per queue traffic stats */
//: Free ioctl num 0 , can be used.
//#define xxx      SIOCIWFIRSTPRIV + 0

#define SIOCGPROTOCOL                (SIOCIWFIRSTPRIV+21)

struct mfi_challenge {
  uint8 challenge_data[20];
};

bool check_valid_bgchannel(uint16 *data_ptr, uint8_t supported_band);
int ieee80211_ioctl_delete_vap(struct ieee80211com *ic, struct ifreq *ifr, struct net_device *mdev);
int ieee80211_ioctl_create_vap(struct ieee80211com *ic, struct ifreq *ifr, struct net_device *mdev);
int onebox_ioctl(struct net_device *dev,struct ifreq *ifr, int cmd);
typedef int (*ioctl_handler_t)(WLAN_ADAPTER w_adapter, struct iwreq *wrq);
#endif

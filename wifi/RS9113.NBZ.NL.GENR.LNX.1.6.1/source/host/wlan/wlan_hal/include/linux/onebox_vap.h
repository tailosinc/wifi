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

#ifndef __CORE_VAP_HEADER_H__
#define __CORE_VAP_HEADER_H__

#include <net80211/ieee80211_var.h>
#include "wlan_common.h"

typedef struct 
{
	uint8        kv_type;                /* one of HAL_CIPHER */
	uint8        kv_pad;
	uint16       kv_len;                 /* length in bits */
	uint8        kv_val[16];             /* enough for 128-bit keys */
	uint8        kv_mic[8];              /* TKIP MIC key */
	uint8        kv_txmic[8];            /* TKIP TX MIC key (optional) */
} HAL_KEYVAL;

typedef enum 
{
	HAL_CIPHER_WEP          = 0,
	HAL_CIPHER_AES_OCB      = 1,
	HAL_CIPHER_AES_CCM      = 2,
	HAL_CIPHER_CKIP         = 3,
	HAL_CIPHER_TKIP         = 4,
	HAL_CIPHER_CLR          = 5,            /* no encryption */

	HAL_CIPHER_MIC          = 127           /* TKIP-MIC, not a cipher */
} HAL_CIPHER;

#define ONEBOX_VAPS_DEFAULT        4    /* default no of beacon buffs */

#define FCC    0x00
#define ETSI   0x01
#define TELEC  0x02
#define WORLD   0x03
#define KCC    0x04
#define SRRC   0x05
#define TAIWAN 0x06

/*
 * Define the scheme that we select MAC address for multiple BSS on the same radio.
 * The very first VAP will just use the MAC address from the EEPROM.
 * For the next 3 VAPs, we set the U/L bit (bit 1) in MAC address,
 * and use the higher bits as the index of the VAP.
 */
#define ONEBOX_GET_VAP_ID(bssid)       ((bssid)[5] & 0x3)

/* driver-specific vap state */
struct core_vap 
{
	int32 (*av_newstate)(struct ieee80211vap *, enum ieee80211_state, int);
	
	netbuf_ctrl_block_m_t *rv_bcbuf;      /* beacon buffer */
	
	struct ieee80211_beacon_offsets rv_boff;/* dynamic update state */
	int32 av_bslot;                   /* beacon slot index */
	onebox_netbuf_head_t rv_mcastq;        /* multicast transmit queue */
	uint32 av_beacon_alloc;
	uint32 curstate; /* current state of the vap, gets updated in newstate */
};

/********************************************************************
 *                      Function Prototypes
 *******************************************************************/

struct ieee80211vap *onebox_vap_create(struct ieee80211com *ic, 
                                       const char *name, int unit, 
                                       enum ieee80211_opmode opmode, int flags, 
                                       const uint8 bssid[IEEE80211_ADDR_LEN],
                                       uint8_t macaddr[IEEE80211_ADDR_LEN]);
void onebox_scan_end(struct ieee80211com *ic);
void onebox_scan_start(struct ieee80211com *ic);
ONEBOX_STATUS onebox_send_rx_filter_frame(WLAN_ADAPTER, uint16_t rx_filter_word);
void onebox_vap_delete(struct ieee80211vap *vap, uint8 );
void onebox_node_cleanup(struct ieee80211_node *ni);
void core_set_channel(struct ieee80211com *ic, struct ieee80211vap *vap);
void core_radio_params_update(struct ieee80211com *ic);
int core_set_params(struct ieee80211com *ic, int cmd, int data);

int onebox_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate, int arg);
int core_wme_update(struct ieee80211com *ic);
void core_updateslot(struct ifnet *ife);
void onebox_newassoc(struct ieee80211_node *ni, int isnew);
int core_rate_setup(WLAN_ADAPTER w_adapter, uint8 mode);
ONEBOX_STATUS onebox_send_auto_rate_request (WLAN_ADAPTER, struct ieee80211_node *);
#ifdef IEEE80211K
ONEBOX_STATUS onebox_send_meas_info(uint8 *meas_struct, uint8_t msrmnt_type);
uint8 rm_prepare_probe_req_body(uint8 *buf,struct radio_meas_info *meas_info,WLAN_ADAPTER w_adapter );
#endif

/*************** Security related prototypes ********************/

int onebox_keyset(WLAN_ADAPTER w_adapter, 
                  const struct ieee80211_key *k,
                  const uint8 mac0[IEEE80211_ADDR_LEN], 
                  struct ieee80211_node *bss);
int  onebox_key_alloc(struct ieee80211vap *,struct ieee80211_key *,
                      ieee80211_keyix * keyix, ieee80211_keyix *rxkeyix);
int onebox_key_delete(struct ieee80211vap *vap, const struct ieee80211_key *k);
int onebox_key_set(struct ieee80211vap *vap, const struct ieee80211_key *k, const uint8 mac[IEEE80211_ADDR_LEN]);
void onebox_key_update_begin(struct ieee80211vap *vap);
void onebox_key_update_end(struct ieee80211vap *vap);
void core_node_free(struct ieee80211_node *ni);
void core_get_default_channels(struct ieee80211com *ic);
void vap_load_beacon(WLAN_ADAPTER w_adapter, int32 vap_id);
void core_net80211_attach(WLAN_ADAPTER w_adapter);
void core_net80211_detach(WLAN_ADAPTER w_adapter);
int core_send_station_info(struct ieee80211_node *sta, int notify_event);
struct ieee80211_node *core_node_alloc(struct ieee80211vap *vap, const uint8 *mac_addr);
void ieee80211_radiotap_attach(struct ieee80211com *ic,
                               struct ieee80211_radiotap_header *th,
                               int tlen, 
                               uint32_t tx_radiotap,
                               struct ieee80211_radiotap_header *rh,
                               int rlen, 
                               uint32_t rx_radiotap);
void ieee80211_radiotap_tx(struct ieee80211vap *vap0, struct mbuf *m);
void onebox_set_contention_vals(struct ieee80211com *ic, WLAN_ADAPTER w_adapter);
#ifdef ONEBOX_CONFIG_WOWLAN
ONEBOX_STATUS onebox_vap_config_wowlan(struct ieee80211vap *vap, void *priv);
#endif

#endif

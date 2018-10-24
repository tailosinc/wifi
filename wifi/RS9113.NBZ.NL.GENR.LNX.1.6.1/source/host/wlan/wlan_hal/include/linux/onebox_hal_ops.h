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

#include "wlan_common.h"
#include "onebox_thread.h"
#include "onebox_linux.h"
#include "onebox_hal.h"

/* Net80211 OPERATIONS */
struct onebox_net80211_operations 
{
	struct ieee80211_node *(*onebox_find_rxnode)(struct ieee80211com *ic,const struct ieee80211_frame_min *wh);
	/* To send a packet to Net80211 when the sender's node is NULL */
	int (*onebox_input_all)(struct ieee80211com *ic,netbuf_ctrl_block_m_t *netbuf_cb, int rssi, int rtsf, int chno);
	void (*onebox_ifattach)(struct ieee80211com *ic,const uint8_t macaddr[IEEE80211_ADDR_LEN]);
	void (*onebox_ifdetach)(struct ieee80211com *ic);
	struct ieee80211vap *  (*onebox_vap_setup)(struct ieee80211com *ic, struct ieee80211vap *vap, const char name[IFNAMSIZ], 
	                                           int32 unit, enum ieee80211_opmode opmode, int32 flags,const uint8 bssid[IEEE80211_ADDR_LEN], 
	                                           const uint8 macaddr[IEEE80211_ADDR_LEN], int dont_create_ifp);
	int (*onebox_vap_attach)(struct ieee80211vap *vap,ifm_change_cb_t media_change, ifm_stat_cb_t media_stat, int dont_attach_ifp);
	void (*onebox_vap_detach)(struct ieee80211vap *vap, int dont_touch_ifp, uint8_t wait_for_lock);
#ifndef BYPASS_TX_DATA_PATH
	/* To encapsulate a frame with the 802.11 Mac header */
	netbuf_ctrl_block_m_t *(*onebox_encap)(struct ieee80211vap *vap, 
	                                       struct ieee80211_node *ni,netbuf_ctrl_block_m_t *netbuf_cb);
#endif
	void (*onebox_iterate_nodes)(struct ieee80211_node_table *nt,ieee80211_iter_func *f, void *arg);
	netbuf_ctrl_block_m_t *(*onebox_beacon_alloc)(struct ieee80211_node *ni,struct ieee80211_beacon_offsets *bo);
	void     (*onebox_free_node)(struct ieee80211_node *);
	struct ieee80211_node *(*onebox_find_node_locked)(struct ieee80211_node_table *,
	                                                  const uint8_t macaddr[IEEE80211_ADDR_LEN]);
	int (*onebox_beacon_update)(struct ieee80211_node *ni,struct ieee80211_beacon_offsets *bo,
	                            netbuf_ctrl_block_m_t *netbuf_cb, int mcast);
	int (*onebox_media_change)(struct ifnet *ifp);
	void (*onebox_media_status)(struct ifnet *ifp, struct ifmediareq *imr);
	void (*onebox_radiotap_attach)(struct ieee80211com *ic,
	                               struct ieee80211_radiotap_header *th,
	                               int tlen, 
	                               uint32_t tx_radiotap,
	                               struct ieee80211_radiotap_header *rh,
	                               int rlen, 
	                               uint32_t rx_radiotap);
	void (*onebox_radiotap_tx)(struct ieee80211vap *vap0, struct mbuf *m);
#ifdef ENABLE_P2P_SUPPORT
	struct ieee80211_p2p * (*onebox_p2p_init)(struct ieee80211vap *vap);
#endif /* ENABLE_P2P_SUPPORT */

	struct ieee80211_node *(*onebox_find_node)(struct ieee80211_node_table *nt,
	                                           const uint8_t macaddr[IEEE80211_ADDR_LEN]);
	void (*onebox_rxmic_failure)(struct ieee80211vap *, const struct ieee80211_frame *, u_int);
	void (*onebox_send_scan_done)(struct ieee80211vap *, bool );
	void (*onebox_send_probereq_confirm)(struct ieee80211vap *);
	/*vap creation */
	struct ieee80211vap*  (*ic_vap_create)(struct ieee80211com *,
	                                       const char name[IFNAMSIZ],
	                                       int unit,
	                                       int opmode,
	                                       int flags,
	                                       const uint8_t bssid[IEEE80211_ADDR_LEN],
	                                       uint8_t macaddr[IEEE80211_ADDR_LEN]);
	/*vap deletion */
	void   (*ic_vap_delete)(struct ieee80211vap *, uint8_t);
	/* Process a received management frame */
	void   (*iv_recv_mgmt) (struct ieee80211_node *, struct mbuf *, int subtype, int rssi , int nf);
	/* new station association callback/notification */
	void   (*ic_newassoc)(struct ieee80211_node *, int);
	/* node state management */
	void   (*ic_node_free)(struct ieee80211_node *);
	void   (*ic_node_cleanup)(struct ieee80211_node *);
	void   (*ic_node_age)(struct ieee80211_node *);
	void   (*ic_node_drain)(struct ieee80211_node *);
	int8_t (*ic_node_getrssi)(const struct ieee80211_node*);
	void   (*ic_node_getsignal)(const struct ieee80211_node*,int8_t *, int8_t *);
	void   (*ic_node_getmimoinfo)(const struct ieee80211_node*,struct ieee80211_mimo_info *);

	void   (*ic_set_channel)(struct ieee80211com *, struct ieee80211vap *);


	/* To send management frames */
	int (*ic_send_mgmt) (struct ieee80211_node *ni, int type, int arg);
	/* [schedule] beacon frame update */
	void    (*iv_update_beacon) (struct ieee80211vap *,int);
	/* 802.3 output method for raw frame xmit */
	//int  (*iv_output) (struct ifnet *, struct mbuf *,
	//struct sockaddr *, struct route *);
	/* received packet  processing at Net80211 */
	int (*iv_input)(struct ieee80211_node *,struct mbuf *, int rssi, int nf);
	/*  Control frames processing at Net80211 */
	void (*iv_recv_ctl)(struct ieee80211_node *,
	                    struct mbuf *, int);
	/* To deliver data to the operating system */
	void   (*iv_deliver_data)  (struct ieee80211vap *,
                                    struct ieee80211_node *, struct mbuf *);

	/*
	 * * 802.11n ADDBA support.  A simple/generic implementation
	 * * of A-MPDU tx aggregation is provided; the driver may
	 * * override these methods to provide their own support.
	 * * A-MPDU rx re-ordering happens automatically if the
	 * * driver passes out-of-order frames to ieee80211_input
	 * * from an assocated HT station.
	 * */
	int (*ic_recv_action)(struct ieee80211_node *,
                             const struct ieee80211_frame *,
                             const uint8_t *frm, const uint8_t *efrm);
	int (*ic_send_action)(struct ieee80211_node *,
	                      int category, int action, void *);
	/* check if A-MPDU should be enabled this station+ac */
	int (*ic_ampdu_enable)(struct ieee80211_node *,
	          struct ieee80211_tx_ampdu *);
	/* start/stop doing A-MPDU tx aggregation for a station */
	int (*ic_addba_request)(struct ieee80211_node *,
	                        struct ieee80211_tx_ampdu *,
	                        int dialogtoken, int baparamset,
	                        int batimeout);
	int (*ic_addba_response)(struct ieee80211_node *,
	                         struct ieee80211_tx_ampdu *,
	                         int status, int baparamset, int batimeout);
	void (*ic_addba_stop)(struct ieee80211_node *,
	                      struct ieee80211_tx_ampdu *);

	/* BAR response received */
	void (*ic_bar_response)(struct ieee80211_node *,
	                        struct ieee80211_tx_ampdu *, int status);

	/* start/stop doing A-MPDU rx processing for a station */
	int (*ic_ampdu_rx_start)(struct ieee80211_node *,
	                         struct ieee80211_rx_ampdu *, int baparamset,
	                         int batimeout, int baseqctl);
	void (*ic_ampdu_rx_stop)(struct ieee80211_node *,
	                         struct ieee80211_rx_ampdu *);

	/*  Key Management callbacks */
	int (*iv_key_alloc)  (struct ieee80211vap *,struct ieee80211_key *,
                              ieee80211_keyix * keyix, ieee80211_keyix *rxkeyix);
	int (*iv_key_delete) (struct ieee80211vap *, 
	                      const struct ieee80211_key *);
	int (*iv_key_set)    (struct ieee80211vap *,
	                      const struct ieee80211_key *,
	                      const uint8_t  mac[IEEE80211_ADDR_LEN]);
	void (*iv_key_update_begin)(struct ieee80211vap * );
	void (*iv_key_update_end)(struct ieee80211vap *);
	/* operate-mode detach hook */
	void  (*iv_opdetach)      (struct ieee80211vap *);
	/* beacon miss processing */ 
	void (*iv_bmiss)         (struct ieee80211vap *);
	/* reset device state after 802.11 parameter/state change */
	int  (*iv_reset)         (struct ieee80211vap *, u_long);
	/* power save handling */
	void (*iv_update_ps)     (struct ieee80211vap *, int);
	int  (*iv_set_tim)       (struct ieee80211_node *, int);
	/* state machine processing */
	int  (*iv_newstate)      (struct ieee80211vap *,
	                               enum ieee80211_state, int);

	/* ieee80211_ratectl structure call backs */
	int      (*ir_attach)        (const struct ieee80211vap *);
	void     (*ir_detach)        (const struct ieee80211vap *);
	void     (*ir_init)          (struct ieee80211vap *);
	void     (*ir_deinit)        (struct ieee80211vap *);
	void     (*ir_node_init)     (struct ieee80211_node *);
	void     (*ir_node_deinit)   (struct ieee80211_node *);
	int      (*ir_rate)          (struct ieee80211_node *, void *, uint32_t);
	void     (*ir_tx_complete)   (const struct ieee80211vap *,
	                              const struct ieee80211_node *, int,
	                              void *, void *);
	void     (*ir_tx_update)     (const struct ieee80211vap *,
	                              const struct ieee80211_node *,
	                              void *, void *, void *);
	void     (*ir_setinterval)   (const struct ieee80211vap *, int);
#ifdef PWR_SAVE_SUPPORT
	void (*onebox_ieee80211_sta_leave)(struct ieee80211_node *ni);
#endif
#ifdef ONEBOX_CONFIG_CFG80211
	struct cfg80211_priv* 	 (*onebox_cfg80211_attach) (struct device *dev, uint8 *, struct ieee80211com *ic );
	struct wireless_dev* (*onebox_cfg80211_register_wireless_dev)(struct net_device *ndev, struct cfg80211_priv *cfg_priv, int opmode);
#endif
	void (*ieee80211_beacon_miss)(struct ieee80211com *ic);
	void (*onebox_node_pwrsave)(struct ieee80211_node *, int );
	void (*onebox_setbasicrates)(struct ieee80211_rateset *, enum ieee80211_phymode );
	void   (*ic_stop_initial_timer)(struct ieee80211com *, struct ieee80211vap *);
	void (*onebox_media_init)(struct ieee80211com *ic);
	void (*onebox_scanreq_signal)(struct ieee80211com *ic);
	void (*onebox_radar_notify)(struct ieee80211com *ic, struct ieee80211_channel *chan);
	void (*onebox_cfg80211_callbacks)(struct ieee80211com *ic);
	struct ieee80211_channel *(*onebox_find_channel)(struct ieee80211com *ic, int, int);
	void (*ic_send_bgscan_params_default)(struct ieee80211com *, uint8 );
#ifdef IEEE80211K
	int (*onebox_meas_rpt)(struct ieee80211vap *vap, uint8 *msg, uint8 msg_len);
#endif

};

/* CORE OPERATIONS */
struct onebox_core_operations
{
	uint32 (*onebox_core_init)(WLAN_ADAPTER w_adapter);
	uint32 (*onebox_core_deinit)(WLAN_ADAPTER w_adapter);
	int (*onebox_core_xmit)(WLAN_ADAPTER w_adapter,netbuf_ctrl_block_t  *netbuf_cb);
	uint32 (*onebox_update_tx_status)(WLAN_ADAPTER w_adapter,struct tx_stat_s *tx_stat);
	uint32 (*onebox_tx_data_done)(WLAN_ADAPTER w_adapter,netbuf_ctrl_block_t *netbuf_cb);
	uint32 (*onebox_indicate_pkt_to_net80211)(WLAN_ADAPTER w_adapter,netbuf_ctrl_block_t *netbuf_cb, int8 rssi, int8 chno);
	uint32 (*onebox_indicate_pkt_to_core)(WLAN_ADAPTER w_adapter,netbuf_ctrl_block_t *netbuf_cb);
#ifdef BYPASS_RX_DATA_PATH
	uint8 (*onebox_bypass_rx_data_pkt)(WLAN_ADAPTER w_adapter,netbuf_ctrl_block_t *netbuf_cb);
#endif
	void (*onebox_core_qos_processor)(WLAN_ADAPTER w_adapter);
        void (*onebox_vap_load_beacon)(WLAN_ADAPTER w_adapter,int32 vap_id);
	struct ieee80211vap*  (*onebox_create_vap)(struct ieee80211com *,
	                       const char name[IFNAMSIZ],
	                       int unit,
	                       enum ieee80211_opmode opmode,
	                       int flags,
	                       const uint8_t bssid[IEEE80211_ADDR_LEN],
	                       uint8_t macaddr[IEEE80211_ADDR_LEN]);

	void (*onebox_core_radiotap_tx)(WLAN_ADAPTER w_adapter, struct ieee80211vap *vap, netbuf_ctrl_block_t *netbuf_cb);
	void (*onebox_dump)(int32 dbg_zone_l, PUCHAR msg_to_print_p,int32 len);
	void (*onebox_mic_failure)(WLAN_ADAPTER w_adapter,uint8 *msg);
	ONEBOX_STATUS (*onebox_stats_frame)(WLAN_ADAPTER w_adapter);
	PWR_STATUS (*onebo_load_deep_sleep) (WLAN_ADAPTER w_adapter, uint16 cmd);
	int (*onebox_start_per_tx)(WLAN_ADAPTER w_adapter);
	ONEBOX_STATUS (*onebox_start_per_burst)(WLAN_ADAPTER w_adapter);
	int  (*onebox_core_bt_xmit) (WLAN_ADAPTER w_adapter, netbuf_ctrl_block_t *netbuf_cb);
  void (*onebox_net80211_attach)(WLAN_ADAPTER w_adapter);
  
};  

/* DEVICE_DEPENDENT OPERATIONS */
struct onebox_devdep_operations
{
	/* In case if u want to change the order of these members, you should change
	 * the order in the definitions also */
	void (*onebox_handle_interrupt)(WLAN_ADAPTER w_adapter);
	ONEBOX_STATUS (*onebox_load_ta_instructions)(WLAN_ADAPTER w_adapter);
	ONEBOX_STATUS (*onebox_snd_cmd_pkt)(WLAN_ADAPTER  w_adapter, netbuf_ctrl_block_t *netbuf_cb);
	//ONEBOX_STATUS (*onebox_snd_mgmt_pkt)(WLAN_ADAPTER w_adapter, 
   //                                          netbuf_ctrl_block_t *netbuf_cb);
	ONEBOX_STATUS (*onebox_send_beacon)(WLAN_ADAPTER w_adapter, 
                                            netbuf_ctrl_block_t *netbuf_cb,
	                                    struct core_vap *core_vp,
                                            int8 dtim_beacon);
	ONEBOX_STATUS (*onebox_send_data_pkt)(WLAN_ADAPTER w_adapter,
                                              netbuf_ctrl_block_t *netbuf_cb,
                                              int8 q_num);
	ONEBOX_STATUS (*onebox_send_pkt)(WLAN_ADAPTER w_adapter,
                                   netbuf_ctrl_block_t *netbuf_cb);
	ONEBOX_STATUS (*onebox_device_init)(WLAN_ADAPTER w_adapter);
	ONEBOX_STATUS (*onebox_device_deinit)(WLAN_ADAPTER w_adapter);
	ONEBOX_STATUS (*onebox_set_channel)(WLAN_ADAPTER w_adapter, uint16 ch_no);
	ONEBOX_STATUS (*onebox_mgmt_send_bb_reset_req)(WLAN_ADAPTER w_adapter);
	int (*onebox_hal_set_sec_wpa_key)(WLAN_ADAPTER w_adapter,const struct ieee80211_node *ni_sta, uint8 key_type);
	uint32 (*onebox_hal_send_sta_notify_frame)(WLAN_ADAPTER w_adapter, struct ieee80211_node *sta_addr, int32 sta_id);
	void (*onebox_schedule_pkt_for_tx)(WLAN_ADAPTER w_adapter);
	void (*onebox_sdio_scheduler_thread)(void *data);
//	int (*onebox_send_qos_conf_frame)(WLAN_ADAPTER w_adapter,struct chanAccParams wme);
	int (*onebox_program_bb_rf) (WLAN_ADAPTER w_adapter);
	ONEBOX_STATUS (*onebox_update_device_op_params) (WLAN_ADAPTER w_adapter);
//	ONEBOX_STATUS (*onebox_start_tx_rx) (WLAN_ADAPTER w_adapter);
	ONEBOX_STATUS (*onebox_start_autorate_stats) (WLAN_ADAPTER w_adapter);
	ONEBOX_STATUS (*onebox_hal_load_key)(WLAN_ADAPTER w_adapter, uint8 *data, uint16 key_len, 
	                                     uint16 offset, uint8 key_type, uint8 key_id, uint32 cipher, struct ieee80211vap *vap);
//	ONEBOX_STATUS (*onebox_set_vap_capabilities)(WLAN_ADAPTER w_adapter, int unit, 
//	                    enum ieee80211_opmode opmode, uint8 *bssid);
	ONEBOX_STATUS (*onebox_set_vap_capabilities)(struct ieee80211vap *vap, uint8 status);
	//uint16* (*onebox_hw_queue_status)(WLAN_ADAPTER w_adapter);
	ONEBOX_STATUS (*onebox_band_check)(WLAN_ADAPTER w_adapter);
	ONEBOX_STATUS (*onebox_set_bb_rf_values)(WLAN_ADAPTER w_adapter, struct iwreq *wrq);
	ONEBOX_STATUS (*onebox_cw_mode)(WLAN_ADAPTER w_adapter, uint8 mode);
	ONEBOX_STATUS (*onebox_send_internal_mgmt_frame)(WLAN_ADAPTER w_adapter, uint16 *addr, uint16 len);
	ONEBOX_STATUS (*onebox_eeprom_rd)(WLAN_ADAPTER w_adapter);
	ONEBOX_STATUS (*onebox_flash_write)(WLAN_ADAPTER w_adapter);
#ifdef BYPASS_TX_DATA_PATH
	int (*onebox_send_block_unblock_data) (struct ieee80211vap *vap, uint8 notify_event, uint8 quiet_enable);
#endif
	ONEBOX_STATUS (*onebox_program_ant_sel)(WLAN_ADAPTER w_adapter, uint8 value, uint8 frame_type);
	ONEBOX_STATUS (*onebox_do_master_ops)(WLAN_ADAPTER w_adapter, struct master_params_s *master, uint16 type);
	ONEBOX_STATUS (*onebox_send_debug_frame)(WLAN_ADAPTER w_adapter, struct test_mode *test);
	ONEBOX_STATUS (*conf_beacon_recv)(WLAN_ADAPTER w_adapter, uint8 value);
	ONEBOX_STATUS (*onebox_get_txpower)(WLAN_ADAPTER w_adapter);
#ifdef RADAR_AUTO
  	ONEBOX_STATUS (*onebox_radar_req_frame)( WLAN_ADAPTER w_adapter, uint8 radar_req, uint8 intr_clr);
#endif
	ONEBOX_STATUS (*onebox_check_scan)(WLAN_ADAPTER w_adapter);
	ONEBOX_STATUS (*onebox_process_eeprom_write)(WLAN_ADAPTER w_adapter);

};

//ONEBOX_STATUS send_deep_sleep(struct ieee80211vap *vap, uint8 enable);

struct onebox_wlan_osd_operations 
{
  int (*onebox_init_wlan_thread)(onebox_thread_handle_t *handle, uint8 *name,
   			    uint32 priority, thread_function function_ptr,
			    void *context);
#if KERNEL_VERSION_BTWN_2_6_(18, 26)
  int (*onebox_kill_wlan_thread)(onebox_thread_handle_t *handle);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(27)
  int (*onebox_kill_wlan_thread)(WLAN_ADAPTER w_adapter);
#endif
  netbuf_ctrl_block_t* (*onebox_get_skb_cb)(netbuf_ctrl_block_m_t *netbuf_cb_m);
  void (*onebox_per_thread) (void *pContext);
  int (*onebox_kill_per_thread)(WLAN_ADAPTER w_adapter);
};

#if KERNEL_VERSION_BTWN_2_6_(18, 26)
int kill_wlan_thread(onebox_thread_handle_t *handle);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(27)
int kill_wlan_thread(WLAN_ADAPTER w_adapter);
#endif
int kill_per_thread(WLAN_ADAPTER w_adapter);

/* EOF */

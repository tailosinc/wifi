/*-
 * Copyright (c) 2001 Atsushi Onoe
 * Copyright (c) 2002-2009 Sam Leffler, Errno Consulting
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD: src/sys/net80211/ieee80211_var.h,v 1.107 2011/12/17 10:23:17 bschmidt Exp $
 */
#ifndef _NET80211_IEEE80211_VAR_H_
#define _NET80211_IEEE80211_VAR_H_

/*
 * Definitions for IEEE 802.11 drivers.
 */
/* NB: portability glue must go first */
#if defined(__NetBSD__)
#include <net80211/ieee80211_netbsd.h>
#elif defined(__FreeBSD__)
#include <net80211/ieee80211_freebsd.h>
#elif defined(__linux__)
#include <ieee80211_linux.h>
#else
#ifdef RSI_CONFIG_ANDROID
#include <ieee80211_linux.h>
#else
#error	"No support for your operating system!"
#endif
#endif

#include <net80211/_ieee80211.h>
#include <net80211/ieee80211.h>
#include <net80211/ieee80211_ageq.h>
#include <net80211/ieee80211_crypto.h>
#include <net80211/ieee80211_dfs.h>
#include <net80211/ieee80211_ioctl.h>		/* for ieee80211_stats */
#include <net80211/ieee80211_phy.h>
#include <net80211/ieee80211_power.h>
#include <net80211/ieee80211_node.h>
#include <net80211/ieee80211_proto.h>
#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_scan.h>
#include <net80211/ieee80211_p2p.h>

#define	IEEE80211_TXPOWER_MAX	100	/* .5 dbM (XXX units?) */
#define	IEEE80211_TXPOWER_MIN	0 	/* in dbM*/

#define	IEEE80211_DTIM_DEFAULT	1	/* default DTIM period */
#define	IEEE80211_BINTVAL_DEFAULT 100	/* default beacon interval (TU's) */

#define	IEEE80211_BMISS_MAX	2	/* maximum consecutive bmiss allowed */
#define	IEEE80211_HWBMISS_DEFAULT 20	/* consecutive beacon miss to initiate disconnection */

#define	IEEE80211_BGSCAN_INTVAL_MIN	15	/* min bg scan intvl (secs) */
#define	IEEE80211_BGSCAN_INTVAL_DEFAULT	(5*60)	/* default bg scan intvl */

#define	IEEE80211_BGSCAN_IDLE_MIN	100	/* min idle time (ms) */
#define	IEEE80211_BGSCAN_IDLE_DEFAULT	250	/* default idle time (ms) */

#define	IEEE80211_SCAN_VALID_MIN	10	/* min scan valid time (secs) */
#define	IEEE80211_SCAN_VALID_DEFAULT	10	/* default scan valid time */

#define	IEEE80211_PS_SLEEP	0x1	/* STA is in power saving mode */
#define	IEEE80211_PS_MAX_QUEUE	50	/* maximum saved packets */

#define	IEEE80211_FIXED_RATE_NONE	0xff
#define	IEEE80211_TXMAX_DEFAULT		6	/* default ucast max retries */

#define	IEEE80211_RTS_DEFAULT		IEEE80211_RTS_MAX
#define	IEEE80211_FRAG_DEFAULT		IEEE80211_FRAG_MAX

#define	IEEE80211_MS_TO_TU(x)	(((x) * 1000) / 1024)
#define	IEEE80211_TU_TO_MS(x)	(((x) * 1024) / 1000)
#define	IEEE80211_TU_TO_TICKS(x)(((x) * 1024 * hz) / (1000 * 1000))

#define MAX_NUM_CHANS 39
/*
 * 802.11 control state is split into a common portion that maps
 * 1-1 to a physical device and one or more "Virtual AP's" (VAP)
 * that are bound to an ieee80211com instance and share a single
 * underlying device.  Each VAP has a corresponding OS device
 * entity through which traffic flows and that applications use
 * for issuing ioctls, etc.
 */

/*
 * Data common to one or more virtual AP's.  State shared by
 * the underlying device and the net80211 layer is exposed here;
 * e.g. device-specific callbacks.
 */
struct ieee80211vap;
typedef void (*ieee80211vap_attach)(struct ieee80211vap *);

struct ieee80211_appie {
	uint16_t		ie_len;		/* size of ie_data */
	uint8_t			ie_data[];	/* user-specified IE's */
};

struct ieee80211_apple_ie {
	bool        	add_apple_ie;	
	uint8_t		*ie_data;	/* user-specified IE's */
	uint16_t	ie_len;		/* size of ie_data */
	unsigned long	jiffies;
};

struct ieee80211_tdma_param;
struct ieee80211_rate_table;
struct ieee80211_tx_ampdu;
struct ieee80211_rx_ampdu;
struct ieee80211_superg;
struct ieee80211_frame;

#ifdef IEEE80211K
struct rm_multicast_dig {
  uint16_t first_seq, highest_mcast_rate,multicast_frame_count;
  uint8_t  first_seq_num,last_seq_num,mcast_var_set;
  uint16_t mcast_meas_duration;
};
#endif

#ifdef ONEBOX_CONFIG_CFG80211
struct cfg80211_priv {
  struct wireless_dev *wdev[4]; //PRAVEEN: change to MAX_VAP macro
  struct net_device *ndev[4]; 
  struct wiphy *wiphy;
  struct ieee80211com *ic;
};

#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
struct cfg80211_ft_events {
        const u8 *ies;
        size_t ies_len;
        const u8 *target_ap;
        const u8 *ric_ies;
        size_t ric_ies_len;
};
#endif // CONFIG_11R

struct cfg80211_events {

 int (*cfg80211_scan_done)(void *scan_req,bool scan_failed);
 void (*cfg_sta_disconnect)(struct wireless_dev *wdev);
#if !defined(CONFIG_11R ) || ((LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)))
 void (*cfg_sta_connection_confirm)(struct wireless_dev *wdev, uint8_t mac[6]) ;
#else 
  void (*cfg_sta_connection_confirm)(struct wireless_dev *wdev, uint8_t mac[6],uint8_t *resp_ie, size_t resp_ie_len) ;
#endif
#if(LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0))
 void (*cfg_node_leave)(struct net_device *ndev, uint8_t *mac_address);
 void (*cfg_node_join)(struct net_device *ndev, uint8_t *mac_address, uint8_t *assoc_req_ie, uint8_t assoc_ie_len);
#endif
#if((LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37)) && (LINUX_VERSION_CODE <= KERNEL_VERSION(3, 5, 7)))
 void (*inform_recvd_mgmt_to_cfg80211)(struct net_device *dev, uint8_t *buf, uint16_t len, uint16_t recv_freq, int32_t sig_dbm);
#else
 void (*inform_recvd_mgmt_to_cfg80211)(struct wireless_dev *wdev, uint8_t *buf, uint16_t len, uint16_t recv_freq, int32_t sig_dbm);
#endif
 void (*cfg80211_radar_status_event)(struct net_device *dev, enum ieee80211_notify_cac_event cmd);
 void (*inform_mic_failure_to_cfg80211)(struct net_device *ndev, uint8_t *mac_addr, uint8_t key_type, int32_t key_id);
#ifdef ENABLE_P2P_SUPPORT
 void (*cfg80211_ready_on_channel)(struct wireless_dev *wdev, u64 cookie, unsigned int freq, unsigned int duration);
 void (*cfg80211_remain_on_channel_expired)(struct wireless_dev *wdev, u64 cookie, unsigned int freq);
#endif
#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
  int (*cfg80211_notify_ft_event) (struct wireless_dev *wdev, uint8_t *ies, size_t len, uint8_t *mac);
#endif //CONFIG_11R
};
#endif
struct ieee80211_channel *addchan_based_cie(struct ieee80211com *ic, int ieee, int flags);

struct ieee80211_useonly_rates {
	uint8_t ur_enable;
	enum ieee80211_phymode ur_mode;
	uint8_t ur_nrates;
	uint8_t ur_rates[IEEE80211_RATE_MAXSIZE];
};

#ifndef __RSI_DEV_MODEL__
#define __RSI_DEV_MODEL__
typedef enum rsi_dev_m {
	RSI_DEV_9110=0,
	RSI_DEV_9113,
	RSI_DEV_9116
} rsi_dev_t;
#endif

struct ieee80211com {
	struct ifnet		*ic_ifp;	/* associated device */
	ieee80211_com_lock_t	ic_comlock;	/* state update lock */
	TAILQ_HEAD(, ieee80211vap) ic_vaps;	/* list of vap instances */
	int			ic_headroom;	/* driver tx headroom needs */
	enum ieee80211_phytype	ic_phytype;	/* XXX wrong for multi-mode */
	enum ieee80211_opmode	ic_opmode;	/* operation mode */
	struct ifmedia		ic_media;	/* interface media config */
	struct callout		ic_inact;	/* inactivity processing */
#ifdef CONFIG_11W
	struct callout		ic_assoc_retry;
	struct callout		ic_sa_procedure;
#endif
	struct taskqueue	*ic_tq;		/* deferred state thread */
	struct work_struct	ic_parent_task;	/* deferred parent processing */
	struct work_struct	ic_promisc_task;/* deferred promisc update */
	struct work_struct	ic_mcast_task;	/* deferred mcast update */
	//struct work_struct	ic_chan_task;	/* deferred channel change */
	struct work_struct	ic_bmiss_task;	/* deferred beacon miss hndlr */

	uint32_t		ic_flags;	/* state flags */
	uint32_t		ic_flags_ext;	/* extended state flags */
	uint32_t		ic_flags_ht;	/* HT state flags */
	uint32_t		ic_flags_ven;	/* vendor state flags */
	uint32_t		ic_caps;	/* capabilities */
	uint32_t		ic_htcaps;	/* HT capabilities */
	uint32_t		ic_htextcaps;	/* HT extended capabilities */
	uint32_t		ic_cryptocaps;	/* crypto capabilities */
	uint8_t			ic_modecaps[2];	/* set of mode capabilities */
	uint8_t			ic_promisc;	/* vap's needing promisc mode */
	uint8_t			ic_allmulti;	/* vap's needing all multicast*/
	uint8_t			ic_nrunning;	/* vap's marked running */
	uint8_t			ic_curmode;	/* current mode */
	uint16_t		ic_bintval;	/* beacon interval */
	uint16_t		ic_lintval;	/* listen interval */
	uint16_t		ic_holdover;	/* PM hold over duration */
	int16_t		ic_txpowlimit;	/* global tx power limit */
	struct ieee80211_rateset ic_sup_rates[IEEE80211_MODE_MAX];

	/*
	 * Channel state:
	 *
	 * ic_channels is the set of available channels for the device;
	 *    it is setup by the driver
	 * ic_nchans is the number of valid entries in ic_channels
	 * ic_chan_avail is a bit vector of these channels used to check
	 *    whether a channel is available w/o searching the channel table.
	 * ic_chan_active is a (potentially) constrained subset of
	 *    ic_chan_avail that reflects any mode setting or user-specified
	 *    limit on the set of channels to use/scan
	 * ic_curchan is the current channel the device is set to; it may
	 *    be different from ic_bsschan when we are off-channel scanning
	 *    or otherwise doing background work
	 * ic_bsschan is the channel selected for operation; it may
	 *    be undefined (IEEE80211_CHAN_ANYC)
	 * ic_prevchan is a cached ``previous channel'' used to optimize
	 *    lookups when switching back+forth between two channels
	 *    (e.g. for dynamic turbo)
	 */
	int			ic_nchans;	/* # entries in ic_channels */
	struct ieee80211_channel ic_channels[IEEE80211_CHAN_MAX];
	uint8_t			ic_chan_avail[IEEE80211_CHAN_BYTES];
	uint8_t			ic_chan_active[IEEE80211_CHAN_BYTES];
	uint8_t			ic_chan_scan[IEEE80211_CHAN_BYTES];
	struct ieee80211_channel *ic_curchan;	/* current channel */
	const struct ieee80211_rate_table *ic_rt; /* table for ic_curchan */
	struct ieee80211_channel *ic_bsschan;	/* bss channel */
	struct ieee80211_channel *ic_prevchan;	/* previous channel */
	struct ieee80211_regdomain ic_regdomain;/* regulatory data */
	struct ieee80211_appie	*ic_countryie;	/* calculated country ie */
	struct ieee80211_channel *ic_countryie_chan;
	struct ieee80211_appie	*ic_supported_chan;	/* calculated country ie */
	struct ieee80211_apple_ie  ic_apple_ie;	/* APPLE ie */

	/* 802.11h/DFS state */
	struct ieee80211_channel *ic_csa_newchan;/* channel for doing CSA */
	short			ic_csa_mode;	/* mode for doing CSA */
	short			ic_csa_count;	/* count for doing CSA */
	struct ieee80211_dfs_state ic_dfs;	/* DFS state */

	struct ieee80211_scan_state *ic_scan;	/* scan state */
	int			ic_lastdata;	/* time of last data frame */
	int			ic_lastscan;	/* time last scan completed */

	/* NB: this is the union of all vap stations/neighbors */
	int			ic_max_keyix;	/* max h/w key index */
	struct ieee80211_node_table ic_sta;	/* stations/neighbors */
	struct ieee80211_ageq	ic_stageq;	/* frame staging queue */
	uint32_t		ic_hash_key;	/* random key for mac hash */

	/* XXX multi-bss: split out common/vap parts */
	struct ieee80211_wme_state ic_wme;	/* WME/WMM state */
	//struct ieee80211_wme_state ic_wme_sta;	/* WME/WMM state */

	/* XXX multi-bss: can per-vap be done/make sense? */
	enum ieee80211_protmode	ic_protmode;	/* 802.11g protection mode */
	uint16_t		ic_nonerpsta;	/* # non-ERP stations */
	uint16_t		ic_longslotsta;	/* # long slot time stations */
	uint16_t		ic_sta_assoc;	/* stations associated */
	uint16_t		ic_ht_sta_assoc;/* HT stations associated */
	uint16_t		ic_ht40_sta_assoc;/* HT40 stations associated */
	uint8_t			ic_curhtprotmode;/* HTINFO bss state */
	enum ieee80211_protmode	ic_htprotmode;	/* HT protection mode */
	int			ic_lastnonerp;	/* last time non-ERP sta noted*/
	int			ic_lastnonht;	/* last time non-HT sta noted */
	uint8_t			ic_rxstream;    /* # RX streams */
	uint8_t			ic_txstream;    /* # TX streams */

	/* optional state for Atheros SuperG protocol extensions */
	struct ieee80211_superg	*ic_superg;

	/* radiotap handling */
	struct ieee80211_radiotap_header *ic_th;/* tx radiotap headers */
	void			*ic_txchan;	/* channel state in ic_th */
	struct ieee80211_radiotap_header *ic_rh;/* rx radiotap headers */
	void			*ic_rxchan;	/* channel state in ic_rh */
	int			ic_montaps;	/* active monitor mode taps */

	/* virtual ap create/delete */
	struct ieee80211vap*	(*ic_vap_create)(struct ieee80211com *,
				    const char [IFNAMSIZ], int,
				    enum ieee80211_opmode, int,
				    const uint8_t [IEEE80211_ADDR_LEN],
				    uint8_t [IEEE80211_ADDR_LEN]);
	void			(*ic_vap_delete)(struct ieee80211vap *, uint8_t );
	/* operating mode attachment */
	ieee80211vap_attach	ic_vattach[IEEE80211_OPMODE_MAX];
	/* return hardware/radio capabilities */
	void			(*ic_getradiocaps)(struct ieee80211com *,
				    int, int *, struct ieee80211_channel []);
	/* check and/or prepare regdomain state change */
	int			(*ic_setregdomain)(struct ieee80211com *,
				    struct ieee80211_regdomain *,
				    int, struct ieee80211_channel []);

	/* send/recv 802.11 management frame */
	int			(*ic_send_mgmt)(struct ieee80211_node *,
				     int, int);
	/* send raw 802.11 frame */
	int			(*ic_raw_xmit)(struct ieee80211_node *,
				    struct mbuf *,
				    const struct ieee80211_bpf_params *);
	/* update device state for 802.11 slot time change */
	void			(*ic_updateslot)(struct ifnet *);
	/* handle multicast state changes */
	void			(*ic_update_mcast)(struct ifnet *);
	/* handle promiscuous mode changes */
	void			(*ic_update_promisc)(struct ifnet *);
	/* new station association callback/notification */
	void			(*ic_newassoc)(struct ieee80211_node *, int);
	/* TDMA update notification */
	void			(*ic_tdma_update)(struct ieee80211_node *,
				    const struct ieee80211_tdma_param *, int);
	/* node state management */
	struct ieee80211_node*	(*ic_node_alloc)(struct ieee80211vap *,
				    const uint8_t [IEEE80211_ADDR_LEN]);
	void			(*ic_node_free)(struct ieee80211_node *);
	void			(*ic_node_cleanup)(struct ieee80211_node *);
	void			(*ic_node_age)(struct ieee80211_node *);
	void			(*ic_node_drain)(struct ieee80211_node *);
	int8_t			(*ic_node_getrssi)(const struct ieee80211_node*);
	void			(*ic_node_getsignal)(const struct ieee80211_node*,
				    int8_t *, int8_t *);
	void			(*ic_node_getmimoinfo)(
				    const struct ieee80211_node*,
				    struct ieee80211_mimo_info *);
	/* scanning support */
	void			(*ic_scan_start)(struct ieee80211com *);
	void			(*ic_scan_end)(struct ieee80211com *);
	void			(*ic_set_channel)(struct ieee80211com *, struct ieee80211vap *);
	void			(*ic_scan_curchan)(struct ieee80211_scan_state *,
				    unsigned long);
	void			(*ic_scan_mindwell)(struct ieee80211_scan_state *);
	void			(*ic_pwr_save)(struct ieee80211vap *, uint32_t, uint32_t);

	/*
	 * 802.11n ADDBA support.  A simple/generic implementation
	 * of A-MPDU tx aggregation is provided; the driver may
	 * override these methods to provide their own support.
	 * A-MPDU rx re-ordering happens automatically if the
	 * driver passes out-of-order frames to ieee80211_input
	 * from an assocated HT station.
	 */
	int			(*ic_recv_action)(struct ieee80211_node *,
				    const struct ieee80211_frame *,
				    const uint8_t *frm, const uint8_t *efrm);
	int			(*ic_send_action)(struct ieee80211_node *,
				    int category, int action, void *);
	/* check if A-MPDU should be enabled this station+ac */
	int			(*ic_ampdu_enable)(struct ieee80211_node *,
				    struct ieee80211_tx_ampdu *);
	/* start/stop doing A-MPDU tx aggregation for a station */
	int			(*ic_addba_request)(struct ieee80211_node *,
				    struct ieee80211_tx_ampdu *,
				    int dialogtoken, int baparamset,
				    int batimeout);
	int			(*ic_addba_response)(struct ieee80211_node *,
				    struct ieee80211_tx_ampdu *,
				    int status, int baparamset, int batimeout);
	void			(*ic_addba_stop)(struct ieee80211_node *,
				    struct ieee80211_tx_ampdu *);
	void			(*ic_addba_response_timeout)(struct ieee80211_node *,
				    struct ieee80211_tx_ampdu *);
	/* BAR response received */
	void			(*ic_bar_response)(struct ieee80211_node *,
				    struct ieee80211_tx_ampdu *, int status);
	/* start/stop doing A-MPDU rx processing for a station */
	int			(*ic_ampdu_rx_start)(struct ieee80211_node *,
				    struct ieee80211_rx_ampdu *, int baparamset,
				    int batimeout, int baseqctl);
	void			(*ic_ampdu_rx_stop)(struct ieee80211_node *,
				    struct ieee80211_rx_ampdu *);
	uint64_t		ic_spare[7];
#ifdef __LINUX__
	int 		(*ic_send_station_info)(struct ieee80211_node *ni,
					int notify_event);
	int			    (*ic_set_params)(struct ieee80211com *ic, int cmd,
					int info);
	unsigned long   ilock_flags;
#endif
	void			(*ic_stop_initial_timer)(struct ieee80211com *, struct ieee80211vap *);
	//int	(*send_deep_sleep)(struct ieee80211vap *, uint8_t );
	uint32_t band_flags;
	uint32_t band_change;
 	uint8_t hasCountryIE[2];
	uint8_t firstScanCmp;
  //uint8_t band[2];
	void	(*ic_update_radio_params)(struct ieee80211com *ic);
	uint8_t band_to_scan;
  //atomic_t scanreq_confirm_recvd;
#ifdef ONEBOX_CONFIG_CFG80211
	struct cfg80211_priv *cfg_priv;	
#ifdef CONFIG_ACS              
	struct survey_dump obm_survey[MAX_NUM_CHANS];
	uint8_t idx;
#endif
#endif
	uint8_t coex_mode;
	uint8_t driver_mode;
#define MODULE_NON_HPM		0
#define MODULE_HPM				1
#define MODULE_WISE_MCU		2
	uint8_t mod_mode;
#ifdef ONEBOX_CONFIG_CFG80211
 struct cfg80211_events cfg80211_call_backs;
#endif
	atomic_t scanreq_flags;
	int scan_cv_flags;
	int scanreq1_flags;
	int probereq_flags;

	int     max_linkhdr;
	int     max_protohdr;
	int     max_hdr;
	int     max_datalen;
	int module_model_type;
	uint8_t mode_11j;
	void (*ic_send_bgscan_params_default)(struct ieee80211com *, uint8_t );
	uint8_t bgscan_host_triggered;
	uint8_t bgscan_2ghz_sent;
	uint8_t bgscan_5ghz_sent;
	rsi_dev_t device_model;
	struct ieee80211_useonly_rates ic_ur;
#ifdef IEEE80211K
	int  (*ic_send_meas_info)( uint8_t *meas_info, uint8_t msrmnt_type);
	struct radio_meas_info meas_info[MAX_MEASUREMENT_REQ];
	struct sm_meas_info sm_meas_info[MAX_MEASUREMENT_REQ];
#endif
};


struct ieee80211_aclator;
struct ieee80211_tdma_state;
struct ieee80211_mesh_state;
struct ieee80211_hwmp_state;

struct dynamic_s{
		uint16_t desc_word[8];
		struct framebody{
			uint16_t data_rate;
			uint16_t mgmt_rate;
			uint16_t keep_alive_period;
		}frame_body;
};

#define MAX_QUIET_INFO_POOL_SIZE 8
#define QUIET_LENGTH 6


typedef struct quiet_info_s
{
  uint8_t  element_id;         //Element id of Quiet frame
  uint8_t  length;             //Length of the element id
  uint8_t  quiet_cnt;          //quiet count in TBTT's after which quiet is to be started
  uint8_t  quiet_period;       //period in number of beacon intervals, after which quiet is to be repeated(not being used in the present architecture)
  uint16_t quiet_duration;     //duration for which Quiet is to be maintained
  uint16_t quiet_offset;       //Offset value in TU's which is to be added to Quiet count to get the start time of Quiet
  uint32_t quiet_tsf_tout;
  struct quiet_info_s *next; //Pointer to the next Quiet element node
}quiet_info_t;

#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
typedef struct 
{  
	atomic_t        eventCondition;         
	wait_queue_head_t   eventQueue;            
}ONEBOX_EVENT_FT;
#endif //CONFIG_11R

struct ieee80211vap {
	uint32_t cur_rate;
	struct iw_statistics    stats;
	struct ifmedia		iv_media;	/* interface media config */
	struct ifnet		*iv_ifp;	/* associated device */
	struct bpf_if		*iv_rawbpf;	/* packet filter structure */
	struct sysctl_ctx_list	*iv_sysctl;	/* dynamic sysctl context */
	struct sysctl_oid	*iv_oid;	/* net.wlan.X sysctl oid */
	uint8_t 		 block_beacon_interrupt;
	TAILQ_ENTRY(ieee80211vap) iv_next;	/* list of vap instances */
	struct ieee80211com	*iv_ic;		/* back ptr to common state */
	uint32_t		iv_debug;	/* debug msg flags */
	struct ieee80211_stats	iv_stats;	/* statistics */

	uint8_t			iv_myaddr[IEEE80211_ADDR_LEN];
	uint32_t		iv_flags;	/* state flags */
	uint32_t		iv_flags_ext;	/* extended state flags */
	uint32_t		iv_flags_ht;	/* HT state flags */
	uint32_t		iv_flags_ven;	/* vendor state flags */
	uint32_t		iv_caps;	/* capabilities */
	uint32_t		iv_htcaps;	/* HT capabilities */
	uint32_t		iv_htextcaps;	/* HT extended capabilities */
	enum ieee80211_opmode	iv_opmode;	/* operation mode */
	enum ieee80211_state	iv_state;	/* state machine state */
	enum ieee80211_state	iv_nstate;	/* pending state */
	int			iv_nstate_arg;	/* pending state arg */
	struct work_struct	iv_nstate_task;	/* deferred state processing */
	struct work_struct	iv_swbmiss_task;/* deferred iv_bmiss call */
	struct callout		iv_mgtsend;	/* mgmt frame response timer */
						/* inactivity timer settings */
	int			iv_inact_init;	/* setting for new station */
	int			iv_inact_auth;	/* auth but not assoc setting */
	int			iv_inact_run;	/* authorized setting */
	int			iv_inact_probe;	/* inactive probe time */

	int			iv_des_nssid;	/* # desired ssids */
	struct ieee80211_scan_ssid iv_des_ssid[1];/* desired ssid table */
	uint8_t			iv_des_bssid[IEEE80211_ADDR_LEN];
	struct ieee80211_channel *iv_des_chan;	/* desired channel */
	uint16_t		iv_des_mode;	/* desired mode */
	int			iv_nicknamelen;	/* XXX junk */
	uint8_t			iv_nickname[IEEE80211_NWID_LEN];
	u_int			iv_bgscanidle;	/* bg scan idle threshold */
	u_int			iv_bgscanintvl;	/* bg scan min interval */
	u_int			iv_scanvalid;	/* scan cache valid threshold */
	u_int			iv_scanreq_duration;
	u_int			iv_scanreq_mindwell;
	u_int			iv_scanreq_maxdwell;
	uint16_t		iv_scanreq_flags;/* held scan request params */
	uint8_t			iv_scanreq_nssid;
	struct ieee80211_scan_ssid iv_scanreq_ssid[IEEE80211_SCAN_MAX_SSID];
	/* sta-mode roaming state */
	enum ieee80211_roamingmode iv_roaming;	/* roaming mode */
	struct ieee80211_roamparam iv_roamparms[IEEE80211_MODE_MAX];

	uint16_t		iv_bmissthreshold;
	uint16_t		iv_keep_alive_period;
	uint8_t			iv_bmiss_count;	/* current beacon miss count */
	int			iv_bmiss_max;	/* max bmiss before scan */
	uint16_t		iv_swbmiss_count; /* beacons in last period */
	uint32_t		iv_swbmiss_period; /* s/w bmiss period */
	struct callout		iv_swbmiss;	/* s/w beacon miss timer */

	int			iv_ampdu_rxmax;	/* A-MPDU rx limit (bytes) */
	int			iv_ampdu_density;/* A-MPDU density */
	int			iv_ampdu_limit;	/* A-MPDU tx limit (bytes) */
	int			iv_amsdu_limit;	/* A-MSDU tx limit (bytes) */
	u_int			iv_ampdu_mintraffic[WME_NUM_AC];

	uint32_t		*iv_aid_bitmap;	/* association id map */
	uint16_t		iv_max_aid;
	uint16_t		iv_sta_assoc;	/* stations associated */
	uint16_t		iv_ps_sta;	/* stations in power save */
	uint16_t		iv_ps_pending;	/* ps sta's w/ pending frames */
	uint16_t		iv_txseq;	/* mcast xmit seq# space */
	uint16_t		iv_tim_len;	/* ic_tim_bitmap size (bytes) */
	uint8_t			*iv_tim_bitmap;	/* power-save stations w/ data*/
	uint8_t			iv_dtim_period;	/* DTIM period */
	uint8_t			iv_dtim_count;	/* DTIM count from last bcn */
						/* set/unset aid pwrsav state */
	uint8_t			iv_quiet;	/* Quiet Element */
	uint8_t			iv_quiet_count;	/* constant count for Quiet Element */
	uint8_t			iv_quiet_count_value;	/* variable count for Quiet Element */
	uint8_t			iv_quiet_period;	/* period for Quiet Element */
	uint16_t		iv_quiet_duration;	/* duration for Quiet Element */
	uint16_t		iv_quiet_offset;	/* offset for Quiet Element */
	int			iv_csa_count;	/* count for doing CSA */
	struct callout		iv_csa;		/* channel switch timer */

	struct ieee80211_node	*iv_bss;	/* information for this node */
	struct ieee80211_txparam iv_txparms[IEEE80211_MODE_MAX];
	uint16_t		iv_rtsthreshold;
	uint16_t		iv_fragthreshold;
	int			iv_inact_timer;	/* inactivity timer wait */
	/* application-specified IE's to attach to mgt frames */
	struct ieee80211_appie	*iv_appie_beacon;
	struct ieee80211_appie	*iv_appie_probereq;
	struct ieee80211_appie	*iv_appie_proberesp;
	struct ieee80211_appie	*iv_appie_assocreq;
	struct ieee80211_appie	*iv_appie_assocresp;
#if defined(CONFIG_11R ) && ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)))
	uint8_t			ft_event_bit;
	uint8_t                  iv_ap_mac[6];
	struct ieee80211_appie	*iv_appie_auth;
	struct ieee80211_appie	*iv_appie_mdft;
	uint8_t			target_ap[IEEE80211_ADDR_LEN];
	uint8_t			*mdie;
	ONEBOX_EVENT_FT	ft_event;
#endif //CONFIG_11R
	struct ieee80211_appie	*iv_appie_wpa;
	uint8_t			*iv_wpa_ie;
	uint8_t			*iv_rsn_ie;
	uint16_t		iv_max_keyix;	/* max h/w key index */
	ieee80211_keyix		iv_def_txkey;	/* default/group tx key index */
#ifdef CONFIG_11W
	ieee80211_keyix		iv_igtk_keyix;
#endif
	struct ieee80211_key	iv_nw_keys[IEEE80211_WEP_NKID];
#ifdef CONFIG_11W
	struct ieee80211_key    iv_igtk_keys[IEEE80211_MAX_NKID];
#endif
	int			(*iv_key_alloc)(struct ieee80211vap *,
				    struct ieee80211_key *,
				    ieee80211_keyix *, ieee80211_keyix *);
	int			(*iv_key_delete)(struct ieee80211vap *, 
				    const struct ieee80211_key *);
	int			(*iv_key_set)(struct ieee80211vap *,
				    const struct ieee80211_key *,
				    const uint8_t mac[IEEE80211_ADDR_LEN]);
	void			(*iv_key_update_begin)(struct ieee80211vap *);
	void			(*iv_key_update_end)(struct ieee80211vap *);

	const struct ieee80211_authenticator *iv_auth; /* authenticator glue */
	void			*iv_ec;		/* private auth state */

	const struct ieee80211_aclator *iv_acl;	/* acl glue */
	void			*iv_as;		/* private aclator state */

	const struct ieee80211_ratectl *iv_rate;
	void			*iv_rs;		/* private ratectl state */

	struct ieee80211_tdma_state *iv_tdma;	/* tdma state */
	struct ieee80211_mesh_state *iv_mesh;	/* MBSS state */
	struct ieee80211_hwmp_state *iv_hwmp;	/* HWMP state */

	/* operate-mode detach hook */
	void			(*iv_opdetach)(struct ieee80211vap *);
	/* receive processing */
	int			(*iv_input)(struct ieee80211_node *,
				    struct mbuf *, int, int);
	void			(*iv_recv_mgmt)(struct ieee80211_node *,
				    struct mbuf *, int, int, int);
	void			(*iv_recv_ctl)(struct ieee80211_node *,
				    struct mbuf *, int);
	void			(*iv_deliver_data)(struct ieee80211vap *,
				    struct ieee80211_node *, struct mbuf *);
#if 0
	/* send processing */
	int			(*iv_send_mgmt)(struct ieee80211_node *,
				     int, int);
#endif
	/* beacon miss processing */
	void			(*iv_bmiss)(struct ieee80211vap *);
	/* reset device state after 802.11 parameter/state change */
	int			(*iv_reset)(struct ieee80211vap *, u_long);
	/* [schedule] beacon frame update */
	void			(*iv_update_beacon)(struct ieee80211vap *, int);
	/* power save handling */
	void			(*iv_update_ps)(struct ieee80211vap *, int);
	int			(*iv_set_tim)(struct ieee80211_node *, int);
	/* state machine processing */
	int			(*iv_newstate)(struct ieee80211vap *,
				    enum ieee80211_state, int);
	/* 802.3 output method for raw frame xmit */
	int			(*iv_output)(struct ifnet *, struct mbuf *,
				    struct sockaddr *, struct route *);
	uint64_t		iv_spare[6];
#ifdef __LINUX__
	struct hal_priv_ieee80211vap *hal_priv_vap;
#endif
	int32_t (*iv_block) (struct ieee80211vap *, uint8_t notify_event, uint8_t quiet_valid);
	int	(*vap_caps)(struct ieee80211vap *, uint8_t );
	int	(*beacon_ssid_notification)(struct ieee80211vap *, uint8_t );
	int	(*channel_command)(struct ieee80211vap *, uint8_t );
	int	(*vap_dynamic_update)(struct ieee80211vap *);
	int	(*iv_mod_bgscan_params)(struct ieee80211vap *, uint16_t *data, uint8_t default_val);
#ifdef ENABLE_P2P_SUPPORT
	uint8_t p2p_enable;
	uint8_t p2p_mode;
#define IEEE80211_P2P_DEVICE 0
#define IEEE80211_P2P_GO     1
#define IEEE80211_P2P_CLIENT 2
	struct ieee80211_p2p *p2p;
#endif
	void  (*check_traffic)(struct ieee80211vap *, uint8_t , uint32_t);
#ifdef ONEBOX_CONFIG_CFG80211
	struct cfg80211_scan_request *scan_request;
	struct wireless_dev *wdev;
//	int temp;
#endif
	struct scan_channel_s
	{
		uint16_t num_freqs;
		uint16_t freqs[IEEE80211_MAX_FREQS_ALLOWED];
	}scan_req;
	
	void	(*onebox_send_bgscan_host_triggered)(struct ieee80211vap *vap);
  struct quiet_info_s  quiet_info_pool[MAX_QUIET_INFO_POOL_SIZE];
  struct quiet_info_s  *quiet_list;
#ifdef ONEBOX_CONFIG_CFG80211
	uint32_t cfg80211_ap_set_channel;
	struct obm_cfg80211_ap_params ap_params;
    uint32_t rssi_hyst;  
    int rssi_thold;
#endif
#if defined(RSI_CCX) || defined(CONFIG_11R)
	struct ieee80211_iapp_pkt aironet_iapp_pkt;
#endif
#ifdef RSI_CCX 
	uint8_t ccx_capable;
	struct ieee80211_aironet_cckm_ie cckm_ie ;
#endif
	uint8_t iv_host_scan;
	uint8_t iv_block_scan;
	uint32_t iv_jointime;
	uint8_t in_wep;        //To check current Security type WEP or not.
#ifdef ONEBOX_CONFIG_WOWLAN
	int (*config_wowlan)(struct ieee80211vap *vap, void *priv);
#endif
	uint64_t iv_mcast_pn;
#ifdef IEEE80211K
	struct rm_multicast_dig rm_mcast_data;
#endif
#ifdef ONEBOX_CONFIG_GTK_OFFLOAD
  void (*send_gtk_rekey_data)(struct ieee80211vap *vap, struct ieee80211_gtk_rekey_data *data);
#endif
};
MALLOC_DECLARE(M_80211_VAP);

#ifdef CONFIG_11W
#define IEEE80211_ADDR_MBCAST(a)        ((a[0] & BIT(0))? 1: 0)               
#endif
#define	IEEE80211_ADDR_EQ(a1,a2)	(memcmp(a1,a2,IEEE80211_ADDR_LEN) == 0)
#define	IEEE80211_ADDR_COPY(dst,src)	memcpy(dst,src,IEEE80211_ADDR_LEN)

/* ic_flags/iv_flags */
#define	IEEE80211_F_TURBOP	0x00000001	/* CONF: ATH Turbo enabled*/
#define	IEEE80211_F_COMP	0x00000002	/* CONF: ATH comp enabled */
#define	IEEE80211_F_FF		0x00000004	/* CONF: ATH FF enabled */
#define	IEEE80211_F_BURST	0x00000008	/* CONF: bursting enabled */
/* NB: this is intentionally setup to be IEEE80211_CAPINFO_PRIVACY */
#define	IEEE80211_F_PRIVACY	0x00000010	/* CONF: privacy enabled */
#define	IEEE80211_F_PUREG	0x00000020	/* CONF: 11g w/o 11b sta's */
#define	IEEE80211_F_SCAN	0x00000080	/* STATUS: scanning */
#define	IEEE80211_F_ASCAN	0x00000100	/* STATUS: active scan */
#define	IEEE80211_F_SIBSS	0x00000200	/* STATUS: start IBSS */
/* NB: this is intentionally setup to be IEEE80211_CAPINFO_SHORT_SLOTTIME */
#define	IEEE80211_F_SHSLOT	0x00000400	/* STATUS: use short slot time*/
#define	IEEE80211_F_PMGTON	0x00000800	/* CONF: Power mgmt enable */
#define	IEEE80211_F_DESBSSID	0x00001000	/* CONF: des_bssid is set */
#define	IEEE80211_F_WME		0x00002000	/* CONF: enable WME use */
#define	IEEE80211_F_BGSCAN	0x00004000	/* CONF: bg scan enabled (???)*/
#define	IEEE80211_F_SWRETRY	0x00008000	/* CONF: sw tx retry enabled */
#define IEEE80211_F_TXPOW_FIXED	0x00010000	/* TX Power: fixed rate */
#define	IEEE80211_F_IBSSON	0x00020000	/* CONF: IBSS creation enable */
#define	IEEE80211_F_SHPREAMBLE	0x00040000	/* STATUS: use short preamble */
#define	IEEE80211_F_DATAPAD	0x00080000	/* CONF: do alignment pad */
#define	IEEE80211_F_USEPROT	0x00100000	/* STATUS: protection enabled */
#define	IEEE80211_F_USEBARKER	0x00200000	/* STATUS: use barker preamble*/
#define	IEEE80211_F_CSAPENDING	0x00400000	/* STATUS: chan switch pending*/
#define	IEEE80211_F_WPA1	0x00800000	/* CONF: WPA enabled */
#define	IEEE80211_F_WPA2	0x01000000	/* CONF: WPA2 enabled */
#define	IEEE80211_F_WPA		0x01800000	/* CONF: WPA/WPA2 enabled */
#define	IEEE80211_F_DROPUNENC	0x02000000	/* CONF: drop unencrypted */
#define	IEEE80211_F_COUNTERM	0x04000000	/* CONF: TKIP countermeasures */
#define	IEEE80211_F_HIDESSID	0x08000000	/* CONF: hide SSID in beacon */
#define	IEEE80211_F_NOBRIDGE	0x10000000	/* CONF: dis. internal bridge */
#define	IEEE80211_F_PCF		0x20000000	/* CONF: PCF enabled */
#define	IEEE80211_F_DOTH	0x40000000	/* CONF: 11h enabled */
#define	IEEE80211_F_DWDS	0x80000000	/* CONF: Dynamic WDS enabled */

#define	IEEE80211_F_BITS \
	"\20\1TURBOP\2COMP\3FF\4BURST\5PRIVACY\6PUREG\10SCAN\11ASCAN\12SIBSS" \
	"\13SHSLOT\14PMGTON\15DESBSSID\16WME\17BGSCAN\20SWRETRY\21TXPOW_FIXED" \
	"\22IBSSON\23SHPREAMBLE\24DATAPAD\25USEPROT\26USERBARKER\27CSAPENDING" \
	"\30WPA1\31WPA2\32DROPUNENC\33COUNTERM\34HIDESSID\35NOBRIDG\36PCF" \
	"\37DOTH\40DWDS"

/* Atheros protocol-specific flags */
#define	IEEE80211_F_ATHEROS \
	(IEEE80211_F_FF | IEEE80211_F_COMP | IEEE80211_F_TURBOP)
/* Check if an Atheros capability was negotiated for use */
#define	IEEE80211_ATH_CAP(vap, ni, bit) \
	((vap)->iv_flags & (ni)->ni_ath_flags & (bit))

/* ic_flags_ext/iv_flags_ext */
#define	IEEE80211_FEXT_INACT	 0x00000002	/* CONF: sta inact handling */
#define	IEEE80211_FEXT_SCANWAIT	 0x00000004	/* STATUS: awaiting scan */
/* 0x00000006 reserved */
#define	IEEE80211_FEXT_BGSCAN	 0x00000008	/* STATUS: complete bgscan */
#define	IEEE80211_FEXT_WPS	 0x00000010	/* CONF: WPS enabled */
#define	IEEE80211_FEXT_TSN 	 0x00000020	/* CONF: TSN enabled */
#define	IEEE80211_FEXT_SCANREQ	 0x00000040	/* STATUS: scan req params */
#define	IEEE80211_FEXT_RESUME	 0x00000080	/* STATUS: start on resume */
#define	IEEE80211_FEXT_4ADDR	 0x00000100	/* CONF: apply 4-addr encap */
#define	IEEE80211_FEXT_NONERP_PR 0x00000200	/* STATUS: non-ERP sta present*/
#define	IEEE80211_FEXT_SWBMISS	 0x00000400	/* CONF: do bmiss in s/w */
#define	IEEE80211_FEXT_DFS	 0x00000800	/* CONF: DFS enabled */
#define	IEEE80211_FEXT_DOTD	 0x00001000	/* CONF: 11d enabled */
#define	IEEE80211_FEXT_STATEWAIT 0x00002000	/* STATUS: awaiting state chg */
#define	IEEE80211_FEXT_REINIT	 0x00004000	/* STATUS: INIT state first */
#define	IEEE80211_FEXT_BPF	 0x00008000	/* STATUS: BPF tap present */
/* NB: immutable: should be set only when creating a vap */
#define	IEEE80211_FEXT_WDSLEGACY 0x00010000	/* CONF: legacy WDS operation */
#define	IEEE80211_FEXT_PROBECHAN 0x00020000	/* CONF: probe passive channel*/
#define	IEEE80211_FEXT_UNIQMAC	 0x00040000	/* CONF: user or computed mac */

#define	IEEE80211_FEXT_BITS \
	"\20\2INACT\3SCANWAIT\4BGSCAN\5WPS\6TSN\7SCANREQ\10RESUME" \
	"\0114ADDR\12NONEPR_PR\13SWBMISS\14DFS\15DOTD\16STATEWAIT\17REINIT" \
	"\20BPF\21WDSLEGACY\22PROBECHAN\23UNIQMAC"

/* ic_flags_ht/iv_flags_ht */
#define	IEEE80211_FHT_NONHT_PR	 0x00000001	/* STATUS: non-HT sta present */
#define	IEEE80211_FHT_GF  	 0x00040000	/* CONF: Greenfield enabled */
#define	IEEE80211_FHT_HT	 0x00080000	/* CONF: HT supported */
#define	IEEE80211_FHT_AMPDU_TX	 0x00100000	/* CONF: A-MPDU tx supported */
#define	IEEE80211_FHT_AMPDU_RX	 0x00200000	/* CONF: A-MPDU rx supported */
#define	IEEE80211_FHT_AMSDU_TX	 0x00400000	/* CONF: A-MSDU tx supported */
#define	IEEE80211_FHT_AMSDU_RX	 0x00800000	/* CONF: A-MSDU rx supported */
#define	IEEE80211_FHT_USEHT40	 0x01000000	/* CONF: 20/40 use enabled */
#define	IEEE80211_FHT_PUREN	 0x02000000	/* CONF: 11n w/o legacy sta's */
#define	IEEE80211_FHT_SHORTGI20	 0x04000000	/* CONF: short GI in HT20 */
#define	IEEE80211_FHT_SHORTGI40	 0x08000000	/* CONF: short GI in HT40 */
#define	IEEE80211_FHT_HTCOMPAT 	 0x10000000	/* CONF: HT vendor OUI's */
#define	IEEE80211_FHT_RIFS  	 0x20000000	/* CONF: RIFS enabled */
#define	IEEE80211_FHT_STBC_TX 	 0x40000000	/* CONF: STBC tx enabled */
#define	IEEE80211_FHT_STBC_RX 	 0x80000000	/* CONF: STBC rx enabled */

#define	IEEE80211_FHT_BITS \
	"\20\1NONHT_PR" \
	"\23GF\24HT\25AMPDU_TX\26AMPDU_TX" \
	"\27AMSDU_TX\30AMSDU_RX\31USEHT40\32PUREN\33SHORTGI20\34SHORTGI40" \
	"\35HTCOMPAT\36RIFS\37STBC_TX\40STBC_RX"

#define	IEEE80211_FVEN_BITS	"\20"

/* ic_caps/iv_caps: device driver capabilities */
/* 0x2e available */
#define	IEEE80211_C_STA		0x00000001	/* CAPABILITY: STA available */
#define	IEEE80211_C_8023ENCAP	0x00000002	/* CAPABILITY: 802.3 encap */
#define	IEEE80211_C_FF		0x00000040	/* CAPABILITY: ATH FF avail */
#define	IEEE80211_C_TURBOP	0x00000080	/* CAPABILITY: ATH Turbo avail*/
#define	IEEE80211_C_IBSS	0x00000100	/* CAPABILITY: IBSS available */
#define	IEEE80211_C_PMGT	0x00000200	/* CAPABILITY: Power mgmt */
#define	IEEE80211_C_HOSTAP	0x00000400	/* CAPABILITY: HOSTAP avail */
#define	IEEE80211_C_AHDEMO	0x00000800	/* CAPABILITY: Old Adhoc Demo */
#define	IEEE80211_C_SWRETRY	0x00001000	/* CAPABILITY: sw tx retry */
#define	IEEE80211_C_TXPMGT	0x00002000	/* CAPABILITY: tx power mgmt */
#define	IEEE80211_C_SHSLOT	0x00004000	/* CAPABILITY: short slottime */
#define	IEEE80211_C_SHPREAMBLE	0x00008000	/* CAPABILITY: short preamble */
#define	IEEE80211_C_MONITOR	0x00010000	/* CAPABILITY: monitor mode */
#define	IEEE80211_C_DFS		0x00020000	/* CAPABILITY: DFS/radar avail*/
#define	IEEE80211_C_MBSS	0x00040000	/* CAPABILITY: MBSS available */
/* 0x7c0000 available */
#define	IEEE80211_C_WPA1	0x00800000	/* CAPABILITY: WPA1 avail */
#define	IEEE80211_C_WPA2	0x01000000	/* CAPABILITY: WPA2 avail */
#define	IEEE80211_C_WPA		0x01800000	/* CAPABILITY: WPA1+WPA2 avail*/
#define	IEEE80211_C_BURST	0x02000000	/* CAPABILITY: frame bursting */
#define	IEEE80211_C_WME		0x04000000	/* CAPABILITY: WME avail */
#define	IEEE80211_C_WDS		0x08000000	/* CAPABILITY: 4-addr support */
#define IEEE80211_C_P2P     0x10000000  /* CAPABILITY: Using p2p available */
#define	IEEE80211_C_BGSCAN	0x20000000	/* CAPABILITY: bg scanning */
#define	IEEE80211_C_TXFRAG	0x40000000	/* CAPABILITY: tx fragments */
#define	IEEE80211_C_TDMA	0x80000000	/* CAPABILITY: TDMA avail */
/* XXX protection/barker? */

#define	IEEE80211_C_OPMODE \
	(IEEE80211_C_STA | IEEE80211_C_IBSS | IEEE80211_C_HOSTAP | \
	 IEEE80211_C_AHDEMO | IEEE80211_C_MONITOR | IEEE80211_C_WDS | \
	 IEEE80211_C_TDMA | IEEE80211_C_MBSS)

#define	IEEE80211_C_BITS \
	"\20\1STA\002803ENCAP\7FF\10TURBOP\11IBSS\12PMGT" \
	"\13HOSTAP\14AHDEMO\15SWRETRY\16TXPMGT\17SHSLOT\20SHPREAMBLE" \
	"\21MONITOR\22DFS\23MBSS\30WPA1\31WPA2\32BURST\33WME\34WDS\36BGSCAN" \
	"\37TXFRAG\40TDMA"

/*
 * ic_htcaps/iv_htcaps: HT-specific device/driver capabilities
 *
 * NB: the low 16-bits are the 802.11 definitions, the upper
 *     16-bits are used to define s/w/driver capabilities.
 */
#define	IEEE80211_HTC_AMPDU	0x00010000	/* CAPABILITY: A-MPDU tx */
#define	IEEE80211_HTC_AMSDU	0x00020000	/* CAPABILITY: A-MSDU tx */
/* NB: HT40 is implied by IEEE80211_HTCAP_CHWIDTH40 */
#define	IEEE80211_HTC_HT	0x00040000	/* CAPABILITY: HT operation */
#define	IEEE80211_HTC_SMPS	0x00080000	/* CAPABILITY: MIMO power save*/
#define	IEEE80211_HTC_RIFS	0x00100000	/* CAPABILITY: RIFS support */
#define	IEEE80211_HTC_RXUNEQUAL	0x00200000	/* CAPABILITY: RX unequal MCS */
#define	IEEE80211_HTC_RXMCS32	0x00400000	/* CAPABILITY: MCS32 support */
#define	IEEE80211_HTC_TXUNEQUAL	0x00800000	/* CAPABILITY: TX unequal MCS */
#define	IEEE80211_HTC_TXMCS32	0x01000000	/* CAPABILITY: MCS32 suport */

#define	IEEE80211_C_HTCAP_BITS \
	"\20\1LDPC\2CHWIDTH40\5GREENFIELD\6SHORTGI20\7SHORTGI40\10TXSTBC" \
	"\21AMPDU\22AMSDU\23HT\24SMPS\25RIFS"

void	ieee80211_ifattach(struct ieee80211com *,
		const uint8_t macaddr[IEEE80211_ADDR_LEN]);
void	ieee80211_ifdetach(struct ieee80211com *);
#ifdef __FREEBSD__
int	ieee80211_vap_setup(struct ieee80211com *, struct ieee80211vap *,
#elif __LINUX__
struct ieee80211vap * ieee80211_vap_setup(struct ieee80211com *, struct ieee80211vap *,
#endif
		const char name[IFNAMSIZ], int unit,
		enum ieee80211_opmode opmode, int flags,
		const uint8_t bssid[IEEE80211_ADDR_LEN],
		const uint8_t macaddr[IEEE80211_ADDR_LEN],
		int dont_create_ifp);
int	ieee80211_vap_attach(struct ieee80211vap *,
		ifm_change_cb_t, ifm_stat_cb_t, int dont_attach_ifp);
void	ieee80211_vap_detach(struct ieee80211vap *, int dont_touch_ifp, uint8_t wait_for_lock);
const struct ieee80211_rateset *ieee80211_get_suprates(struct ieee80211com *ic,
		const struct ieee80211_channel *);
void	ieee80211_announce(struct ieee80211com *);
void	ieee80211_announce_channels(struct ieee80211com *);
void	ieee80211_drain(struct ieee80211com *);
void	ieee80211_media_init(struct ieee80211com *);
void    scanreq_signal(struct ieee80211com *ic);
struct ieee80211com *ieee80211_find_vap(const uint8_t mac[IEEE80211_ADDR_LEN]);
int	ieee80211_media_change(struct ifnet *);
void	ieee80211_media_status(struct ifnet *, struct ifmediareq *);
#ifdef __LINUX__
int ieee80211_ioctl(struct ifnet *ifp,struct ifreq *data, int cmd);
#else
int
ieee80211_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data);
#endif
#ifdef ONEBOX_CONFIG_CFG80211
struct cfg80211_priv* cfg80211_attach(struct device *dev, uint8_t *, struct ieee80211com *);
void cfg80211_callbacks(struct ieee80211com *ic);
struct wireless_dev* cfg80211_wdev_register(struct net_device *ndev, struct cfg80211_priv *cfg_priv, int opmode);
void cfg80211_detach(struct cfg80211_priv *cfg_priv);
int onebox_inform_bss_to_cfg80211(struct ieee80211_scan_entry *ise, struct ieee80211vap *vap);
void cfg80211_init_callbacks(struct ieee80211com *ic);
#endif
struct ieee80211_channel *findchannel(struct ieee80211com *,
		int ieee, int mode);
int	ieee80211_rate2media(struct ieee80211com *, int,
		enum ieee80211_phymode);
int	ieee80211_media2rate(int);
int	ieee80211_mhz2ieee(u_int, u_int);
int	ieee80211_chan2ieee(struct ieee80211com *,
		const struct ieee80211_channel *);
u_int	ieee80211_ieee2mhz(u_int, u_int);
struct ieee80211_channel *ieee80211_find_channel(struct ieee80211com *,
		int freq, int flags);
struct ieee80211_channel *ieee80211_find_channel_byieee(struct ieee80211com *,
		int ieee, int flags);
int	ieee80211_setmode(struct ieee80211com *, enum ieee80211_phymode);
enum ieee80211_phymode ieee80211_chan2mode(const struct ieee80211_channel *);
uint32_t ieee80211_mac_hash(const struct ieee80211com *,
		const uint8_t addr[IEEE80211_ADDR_LEN]);

void	ieee80211_radiotap_attach(struct ieee80211com *,
	    struct ieee80211_radiotap_header *th, int tlen,
		uint32_t tx_radiotap,
	    struct ieee80211_radiotap_header *rh, int rlen,
		uint32_t rx_radiotap);
void	ieee80211_radiotap_detach(struct ieee80211com *);
void	ieee80211_radiotap_vattach(struct ieee80211vap *);
void	ieee80211_radiotap_vdetach(struct ieee80211vap *);
void	ieee80211_radiotap_chan_change(struct ieee80211com *);
void	ieee80211_radiotap_tx(struct ieee80211vap *, struct mbuf *);
void	ieee80211_radiotap_rx(struct ieee80211vap *, struct mbuf *);
void	ieee80211_radiotap_rx_all(struct ieee80211com *, struct mbuf *);

void hostap_recv_trigger_frame(struct ieee80211_node *ni, int no_of_frames);

static __inline int
ieee80211_radiotap_active(const struct ieee80211com *ic)
{
	return (ic->ic_flags_ext & IEEE80211_FEXT_BPF) != 0;
}

static __inline int
ieee80211_radiotap_active_vap(const struct ieee80211vap *vap)
{
	return (vap->iv_flags_ext & IEEE80211_FEXT_BPF) ||
	    vap->iv_ic->ic_montaps != 0;
}

/*
 * Enqueue a task on the state thread.
 */
static __inline void
ieee80211_runtask(struct ieee80211com *ic, struct work_struct *task)
{
	taskqueue_enqueue(ic->ic_tq, task);
}

/*
 * Wait for a queued task to complete.
 */
static __inline void
ieee80211_draintask(struct ieee80211com *ic, struct work_struct *task)
{
	taskqueue_drain(ic->ic_tq, task);
}

/* 
 * Key update synchronization methods.  XXX should not be visible.
 */
static __inline void
ieee80211_key_update_begin(struct ieee80211vap *vap)
{
	vap->iv_key_update_begin(vap);
}
static __inline void
ieee80211_key_update_end(struct ieee80211vap *vap)
{
	vap->iv_key_update_end(vap);
}

/*
 * XXX these need to be here for IEEE80211_F_DATAPAD
 */

/*
 * Return the space occupied by the 802.11 header and any
 * padding required by the driver.  This works for a
 * management or data frame.
 */
static __inline int
ieee80211_hdrspace(struct ieee80211com *ic, const void *data)
{
	int size = ieee80211_hdrsize(data);
	if (ic->ic_flags & IEEE80211_F_DATAPAD)
		size = roundup(size, sizeof(uint32_t));
	return size;
}

/*
 * Like ieee80211_hdrspace, but handles any type of frame.
 */
static __inline int
ieee80211_anyhdrspace(struct ieee80211com *ic, const void *data)
{
	int size = ieee80211_anyhdrsize(data);
	if (ic->ic_flags & IEEE80211_F_DATAPAD)
		size = roundup(size, sizeof(uint32_t));
	return size;
}

/*
 * Notify a vap that beacon state has been updated.
 */
static __inline void
ieee80211_beacon_notify(struct ieee80211vap *vap, int what)
{
	if (vap->iv_state == IEEE80211_S_RUN)
		vap->iv_update_beacon(vap, what);
}

/*
 * Calculate HT channel promotion flags for a channel.
 * XXX belongs in ieee80211_ht.h but needs IEEE80211_FHT_*
 */
static __inline int
ieee80211_htchanflags(const struct ieee80211_channel *c)
{
	return IEEE80211_IS_CHAN_HT40(c) ?
	    IEEE80211_FHT_HT | IEEE80211_FHT_USEHT40 :
	    IEEE80211_IS_CHAN_HT(c) ?  IEEE80211_FHT_HT : 0;
}

/*
 * Debugging facilities compiled in when IEEE80211_DEBUG is defined.
 *
 * The intent is that any problem in the net80211 layer can be
 * diagnosed by inspecting the statistics (dumped by the wlanstats
 * program) and/or the msgs generated by net80211.  Messages are
 * broken into functional classes and can be controlled with the
 * wlandebug program.  Certain of these msg groups are for facilities
 * that are no longer part of net80211 (e.g. IEEE80211_MSG_DOT1XSM).
 */
#define	IEEE80211_MSG_11N	0x80000000	/* 11n mode debug */
#define	IEEE80211_MSG_DEBUG	0x40000000	/* IFF_DEBUG equivalent */
#define	IEEE80211_MSG_DUMPPKTS	0x20000000	/* IFF_LINK2 equivalant */
#define	IEEE80211_MSG_CRYPTO	0x10000000	/* crypto work */
#define	IEEE80211_MSG_INPUT	0x08000000	/* input handling */
#define	IEEE80211_MSG_XRATE	0x04000000	/* rate set handling */
#define	IEEE80211_MSG_ELEMID	0x02000000	/* element id parsing */
#define	IEEE80211_MSG_NODE	0x01000000	/* node handling */
#define	IEEE80211_MSG_ASSOC	0x00800000	/* association handling */
#define	IEEE80211_MSG_AUTH	0x00400000	/* authentication handling */
#define	IEEE80211_MSG_SCAN	0x00200000	/* scanning */
#define	IEEE80211_MSG_OUTPUT	0x00100000	/* output handling */
#define	IEEE80211_MSG_STATE	0x00080000	/* state machine */
#define	IEEE80211_MSG_POWER	0x00040000	/* power save handling */
#define	IEEE80211_MSG_HWMP	0x00020000	/* hybrid mesh protocol */
#define	IEEE80211_MSG_DOT1XSM	0x00010000	/* 802.1x state machine */
#define	IEEE80211_MSG_RADIUS	0x00008000	/* 802.1x radius client */
//#define	IEEE80211_MSG_RADDUMP	0x00004000	/* dump 802.1x radius packets */
#define	IEEE80211_MSG_ERROR	0x00004000	/* Error messages */
#define	IEEE80211_MSG_MESH	0x00002000	/* mesh networking */
#define	IEEE80211_MSG_WPA	0x00001000	/* WPA/RSN protocol */
#define	IEEE80211_MSG_ACL	0x00000800	/* ACL handling */
#define	IEEE80211_MSG_WME	0x00000400	/* WME protocol */
#define	IEEE80211_MSG_SUPERG	0x00000200	/* Atheros SuperG protocol */
#define	IEEE80211_MSG_DOTH	0x00000100	/* 802.11h support */
#define	IEEE80211_MSG_INACT	0x00000080	/* inactivity handling */
#define	IEEE80211_MSG_ROAM	0x00000040	/* sta-mode roaming */
#define	IEEE80211_MSG_RATECTL	0x00000020	/* tx rate control */
#define	IEEE80211_MSG_ACTION	0x00000010	/* action frame handling */
#define	IEEE80211_MSG_WDS	0x00000008	/* WDS handling */
#define	IEEE80211_MSG_IOCTL	0x00000004	/* ioctl handling */
#define	IEEE80211_MSG_TDMA	0x00000002	/* TDMA handling */

#define	IEEE80211_MSG_ANY	0xffffffff	/* anything */

#define	IEEE80211_MSG_BITS \
	"\20\2TDMA\3IOCTL\4WDS\5ACTION\6RATECTL\7ROAM\10INACT\11DOTH\12SUPERG" \
	"\13WME\14ACL\15WPA\16RADKEYS\17RADDUMP\20RADIUS\21DOT1XSM\22HWMP" \
	"\23POWER\24STATE\25OUTPUT\26SCAN\27AUTH\30ASSOC\31NODE\32ELEMID" \
	"\33XRATE\34INPUT\35CRYPTO\36DUPMPKTS\37DEBUG\04011N"


#define IEEE80211_DBG_PRINT(_vap, _m, x ) do {		\
	if( _vap ) {	\
		if((_vap)->iv_debug & (_m)) printk x; 	\
	} else 	printk x;						\
} while (0)

#ifdef IEEE80211_DEBUG
#define	ieee80211_msg(_vap, _m)	((_vap)->iv_debug & (_m))
#define	IEEE80211_DPRINTF(_vap, _m, _fmt, ...) do {			\
	if (ieee80211_msg(_vap, _m))					\
		ieee80211_note(_vap, _fmt, __VA_ARGS__);		\
} while (0)
#define	IEEE80211_NOTE(_vap, _m, _ni, _fmt, ...) do {			\
	if (ieee80211_msg(_vap, _m))					\
		ieee80211_note_mac(_vap, (_ni)->ni_macaddr, _fmt, __VA_ARGS__);\
} while (0)
#define	IEEE80211_NOTE_MAC(_vap, _m, _mac, _fmt, ...) do {		\
	if (ieee80211_msg(_vap, _m))					\
		ieee80211_note_mac(_vap, _mac, _fmt, __VA_ARGS__);	\
} while (0)
#define	IEEE80211_NOTE_FRAME(_vap, _m, _wh, _fmt, ...) do {		\
	if (ieee80211_msg(_vap, _m))					\
		ieee80211_note_frame(_vap, _wh, _fmt, __VA_ARGS__);	\
} while (0)
void	ieee80211_note(const struct ieee80211vap *, const char *, ...);
void	ieee80211_note_mac(const struct ieee80211vap *,
		const uint8_t mac[IEEE80211_ADDR_LEN], const char *, ...);
void	ieee80211_note_frame(const struct ieee80211vap *,
		const struct ieee80211_frame *, const char *, ...);
#define	ieee80211_msg_debug(_vap) \
	((_vap)->iv_debug & IEEE80211_MSG_DEBUG)
#define	ieee80211_msg_dumppkts(_vap) \
	((_vap)->iv_debug & IEEE80211_MSG_DUMPPKTS)
#define	ieee80211_msg_input(_vap) \
	((_vap)->iv_debug & IEEE80211_MSG_INPUT)
#define	ieee80211_msg_radius(_vap) \
	((_vap)->iv_debug & IEEE80211_MSG_RADIUS)
#if 0
#define	ieee80211_msg_dumpradius(_vap) \
	((_vap)->iv_debug & IEEE80211_MSG_RADDUMP)
#endif
#define	ieee80211_msg_dumpradkeys(_vap) \
	((_vap)->iv_debug & IEEE80211_MSG_RADKEYS)
#define	ieee80211_msg_scan(_vap) \
	((_vap)->iv_debug & IEEE80211_MSG_SCAN)
#define	ieee80211_msg_assoc(_vap) \
	((_vap)->iv_debug & IEEE80211_MSG_ASSOC)

/*
 * Emit a debug message about discarding a frame or information
 * element.  One format is for extracting the mac address from
 * the frame header; the other is for when a header is not
 * available or otherwise appropriate.
 */
#define	IEEE80211_DISCARD(_vap, _m, _wh, _type, _fmt, ...) do {		\
	if ((_vap)->iv_debug & (_m))					\
		ieee80211_discard_frame(_vap, _wh, _type, _fmt, __VA_ARGS__);\
} while (0)
#define	IEEE80211_DISCARD_IE(_vap, _m, _wh, _type, _fmt, ...) do {	\
	if ((_vap)->iv_debug & (_m))					\
		ieee80211_discard_ie(_vap, _wh, _type, _fmt, __VA_ARGS__);\
} while (0)
#define	IEEE80211_DISCARD_MAC(_vap, _m, _mac, _type, _fmt, ...) do {	\
	if ((_vap)->iv_debug & (_m))					\
		ieee80211_discard_mac(_vap, _mac, _type, _fmt, __VA_ARGS__);\
} while (0)

void ieee80211_discard_frame(const struct ieee80211vap *,
	const struct ieee80211_frame *, const char *type, const char *fmt, ...);
void ieee80211_discard_ie(const struct ieee80211vap *,
	const struct ieee80211_frame *, const char *type, const char *fmt, ...);
void ieee80211_discard_mac(const struct ieee80211vap *,
	const uint8_t mac[IEEE80211_ADDR_LEN], const char *type,
	const char *fmt, ...);
#else
#define	IEEE80211_DPRINTF(_vap, _m, _fmt, ...)
#define	IEEE80211_NOTE(_vap, _m, _ni, _fmt, ...)
#define	IEEE80211_NOTE_FRAME(_vap, _m, _wh, _fmt, ...)
#define	IEEE80211_NOTE_MAC(_vap, _m, _mac, _fmt, ...)
#define	ieee80211_msg_dumppkts(_vap)	0
#define	ieee80211_msg(_vap, _m)		0

#define	IEEE80211_DISCARD(_vap, _m, _wh, _type, _fmt, ...)
#define	IEEE80211_DISCARD_IE(_vap, _m, _wh, _type, _fmt, ...)
#define	IEEE80211_DISCARD_MAC(_vap, _m, _mac, _type, _fmt, ...)
#endif

#ifdef CONFIG_11W
bool ieee80211_is_robust_mgmt_frame(struct ieee80211com *ic, int type,struct mbuf *m);
#endif
void ieee80211_process_quiet(struct ieee80211vap *vap);
void ieee80211_add_to_ascending_linkedlist (struct ieee80211vap *vap, struct quiet_info_s **quiet_list, uint8_t *pkt, uint32_t residue, uint16_t node_num);
void dfs_passive_to_active_scan_handler(struct ieee80211vap *vap);
void ieee80211_process_quiet_element(struct ieee80211vap *vap, uint8_t *pkt, uint16_t node_num);

#ifdef CONFIG_11W
static inline int ieee80211_is_mgmt(__le16 fc)
{
        return (fc & cpu_to_le16(IEEE80211_FCTL_FTYPE)) ==
               cpu_to_le16(IEEE80211_FTYPE_MGMT);
}
#endif
#ifdef ONEBOX_CONFIG_CFG80211    
int onebox_inform_cqm_to_cfg80211(struct net_device *ndev,int low, int rssi);
#endif /*ONEBOX_CONFIG_CFG80211*/

#endif /* _NET80211_IEEE80211_VAR_H_ */

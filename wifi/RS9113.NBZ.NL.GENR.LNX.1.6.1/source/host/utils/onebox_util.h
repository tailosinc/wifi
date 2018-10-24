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
 
#ifndef _ONEBOX_UTILS_H_
#define _ONEBOX_UTILS_H_

#include <sys/ioctl.h>
#include <arpa/inet.h>

#define SIOCIWFIRSTPRIV          0x8BE0        //defined in .h file

/***** PROTOCOL **************/
#define WLAN_PROTOCOL BIT(0)
#define BT_PROTOCOL   BIT(1)
#define ZIGB_PROTOCOL BIT(2)
#define PROTOCOL_ENABLE  1
#define PROTOCOL_DISABLE 0


/**** private ioctls ******/
#define RSIIOCPRINTRTP         SIOCIWFIRSTPRIV + 0  //using for bss start 
#define RSIIOCVAPCREATE          SIOCIWFIRSTPRIV + 16
#define RSIIOCVAPDEL              0x8BEF
#define RSIIOCPROTOMODE           SIOCIWFIRSTPRIV + 21
#define RSISTATIONSTATS          (SIOCIWFIRSTPRIV+17)
#define RSIVAPSTATS               0x8BF7
#define RSISTAINFO               (SIOCIWFIRSTPRIV+27)


#define SD_REQUEST_MASTER                 0x10000

#define NL80211_SET_IOCTL             0x8BFE
#define NL80211_GET_IOCTL             0x8BFD

#define IEEE80211_PARAM_ROAMING 12
#define IEEE80211_PARAM_BEACON_MISS_THRESH  73
#define IEEE80211_PARAM_SHORT_GI  86
#define IEEE80211_PARAM_CHANGE_COUNTRY_IE  85
#define IEEE80211_PARAM_HIDESSID  19
#define IEEE80211_PARAM_WMM       18
#define IEEE80211_PARAM_AMPDU_DENSITY 92
#define IEEE80211_PARAM_PUREG 37
#define IEEE80211_PARAM_PUREN 103
#define IEEE80211_PARAM_MODE 2
#define IEEE80211_PARAM_RATE 105
#define IEEE80211_PARAM_NAME 106
#define IEEE80211_PARAM_BW   107
#define IEEE80211_PARAM_SHPREAMBLE 61
#define IEEE80211_KEEP_ALIVE_PERIOD  119

#define IEEE80211_PARAM_PRIVACY 13
#define IEEE80211_MAX_CHAINS     3
#define IEEE80211_TID_SIZE      17
#define IEEE80211_RATE_MAXSIZE  15
#define IEEE80211_ADDR_LEN       6
#define MAX_RATES   20

#define RSI_VAPCREATE           2
#define RSI_VAPDEL              3
#define RSI_HOSTAPD_IOCTLS      4
#define RSI_WMM_PARAMS          5
#define RSI_GET_AUTORATE_STATS	6
#define VAPSTATS                7
#define RSI_SET_SEC             8
#define RSI_SET_ENCRYP          9
#define STATIONSTATS           10
#define STAINFO                11
#define SET_BEACON_INVL        18

#define RSI_SET_BEACON        	19
#define RSI_SET_CW_MODE         20
#define RSI_SET_BGSCAN         	21
#define RSI_DO_BGSCAN          	22
#define RSI_BGSCAN_SSID        	23
#define RSI_SET_ENDPOINT        24
#define RSI_PS_REQUEST          25
#define RSI_EEPROM_READ         26
#define RSI_EEPROM_WRITE        27
#define RSI_SET_BB_READ         28
#define RSI_SET_BB_WRITE        29
#define RSI_ANT_SEL		30
#define RSI_FLASH_VERIFY        31
#define RSI_RESET_ADAPTER       32
#define RSI_UAPSD_REQ           33
#define RSI_RESET_PER_Q_STATS   34
#define RSI_AGGR_LIMIT          35
#define RSI_RX_FILTER		36
#define RSI_MASTER_READ		37
#define RSI_MASTER_WRITE	38
#define RSI_TEST_MODE		39
#define RSI_TX_RX_RF_PWR_MODE	40
#define RSI_RF_WRITE		41
#define RSI_RF_READ		42
/*** SNIFFER APP DEFINES***/
#define SNIFFER_MODE		43
#define CH_UTILIZATION		44
#define RSI_SET_COUNTRY   45
#define RSI_GET_INFO      46
#define RSI_SET_SCAN_TYPE 47
/***WOWLAN*/
#define RSI_WOWLAN_CONFIG		  48
#define RSI_SET_ED_THRESHOLD	49
#define RSI_SET_EXT_ANT_GAIN	50
#define RSI_ENABLE_PROTOCOL   51
#define RSI_DISABLE_PROTOCOL  52
#define RSI_BEACON_RECV_DIS	  53
#define RSI_GET_TXPOWER	  54
#define RSI_ANT_TYPE	55
#define RSI_HOST_SCAN	56
#define RSI_HOST_SCAN_2G	57
#define RSI_STA_STATE		58
#define RSI_SCAN_STOP           59
#define RSI_MAX_POWER	60
#define RSI_SPECTRAL_MASK 61
#define RSI_USEONLY_RATES 62 
#define GPIO_R_W                       63
#define AFE_SPI_R_W                    64
#define AHB_R_W                        65
#define SOC_PLL_R_W                    66
#define COEX_SLOT_PARAMS               67
#define WLAN_9116_FEATURE              68
#define LOG_STRUCT_PRGMG               69
#define PROTOCOL_RF_WRITE	             70
#define PROTOCOL_RF_READ	             71
#define DISABLE_PROGRAMMING            72
#define PROG_STRUCTURE                 73
#define IPMU_R_W                      74
/*** IEEE80211K ***/
#ifdef IEEE80211K
#define RSI_SET_MSRMNT_PARAMS 75
#endif
#define RSI_GTK_OFFLOAD     76
#define RSI_PUF_REQUEST   77

/*** CMD FLAGS IN DRIVER **/
#define SET_BEACON_INVL 	   18
#define SET_BGSCAN_PARAMS      19
#define DO_BGSCAN              20
#define BGSCAN_SSID            21
#define EEPROM_READ_IOCTL      22
#define EEPROM_WRITE_IOCTL     23
#define PS_REQUEST            25
#define UAPSD_REQ             26
#define RESET_ADAPTER         32
#define RESET_PER_Q_STATS      33 
#define AGGR_LIMIT             34
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
#define ADD_APPLE_IE	46
#define IAP_INIT	47
#define IAP_MFI_CHALLENGE	48
#define IAP_READ_CERTIFICATE	49
#define CONF_BEACON_RECV	51
#define GET_TXPOWER		52
#define SET_HOST_SCAN		53
#define CHECK_STA_STATE		54
#define ENABLE_MAX_POWER	55
#define SPECTRAL_MASK   56
#define SET_USEONLY_RATES     57
/** END **/
#ifdef IEEE80211K
#define SET_MSRMNT_PARAMS    58
#endif
#define GTK_OFFLOAD     59
#define PUF_REQUEST		60

#define WME_BE_Q        0
#define WME_BK_Q        1
#define WME_VI_Q        2
#define WME_VO_Q        3

#define STATION_STATS_OFFSET  8

#define IEEE80211_IOC_WME_CWMIN       46        /* WME: ECWmin */
#define IEEE80211_IOC_WME_CWMAX       47        /* WME: ECWmax */
#define IEEE80211_IOC_WME_AIFS        48        /* WME: AIFSN */
#define IEEE80211_IOC_WME_TXOPLIMIT   49        /* WME: txops limit */
#define IEEE80211_IOC_WME_ACM         50        /* WME: ACM (bss only) */
#define IEEE80211_IOC_WME_ACKPOLICY   51        /* WME: ACK policy (!bss only)*/
#define	IEEE80211_BINTVAL_MAX	1000	/* max beacon interval (TU's) */
#define	IEEE80211_BINTVAL_MIN_AP	52	/* min beacon interval (TU's) */
#define PRIVACY_ENABLED 1
#define PRIVACY_DISABLED 0
#define BW_40Mhz 1
#define BW_20Mhz 0

/* BW Configuration in Rate flags BIT(2)-BIT(4) */

#define BW_20 	0
#define BW_1	1
#define BW_U40	2
#define BW_2	3
#define BW_L40	4
#define BW_5	5
#define BW_F40	6
#define BW_10	7


struct bgscan_ssid{
	u_int16_t ssid_len;
	u_int8_t ssid[32];
};
struct get_info {
	u_int8_t param_name[16];
	u_int8_t param_length;
	u_int8_t *data;
};
#if 0
typedef struct ps_config_s{
	unsigned short enable_disable;
	unsigned short sleep_type;
	unsigned short uapsd_acs;  
	unsigned short uapsd_type; 
	unsigned short uapsd_period;
	unsigned short mimic_support;
	unsigned short listen_interval;
	unsigned int threshold;
	unsigned short hysteresis;
	unsigned short lp_ulp;
//#define ULP_MODE 				             1		//already defined as BIT(0)
#define LP_MODE 				             2
	unsigned short wakeup_type;
#define PERIODIC_WAKEUP	1
#define GPIO_WAKEUP 2
} ps_config_t;
#endif

#define LP_SLEEP_TYPE 1
#define ULP_SLEEP_TYPE 2
struct ps_req_params
{
	u_int8_t ps_en;
	u_int8_t sleep_type;
	u_int8_t tx_threshold;
	u_int8_t rx_threshold;
	u_int8_t tx_hysterisis;
	u_int8_t rx_hysterisis;
	u_int16_t monitor_interval;
	u_int32_t listen_interval;
	u_int16_t num_beacons_per_listen_interval;
	u_int32_t dtim_interval_duration;
	u_int16_t num_dtims_per_sleep;
	u_int32_t deep_sleep_wakeup_period;
#ifdef DUTY_CYCLE_IOCTL
	u_int8_t duty_cycle;
#endif
};

struct uapsd_params
{
	u_int8_t en;
	u_int8_t acs;
	u_int8_t uapsd_wakeup_period;
};

struct rsi_clone_params 
{
	char icp_name[IFNAMSIZ];      /* device name */
	unsigned short icp_opmode;    /* operating mode */
	unsigned short icp_flags;     /* see below */
#define RSI_CLONE_BSSID     0x0001    /* allocate unique mac/bssid */
#define RSI_NO_STABEACONS   0x0002    /* Do not setup the station beacon timers */
};

struct onebox_wmm_params
{
	int wmm_param;                /* wmm parameter name */
	char icp_name[IFNAMSIZ];      /* device name */
	unsigned short icp_opmode;    /* operating mode */
	int wmm_ac;                   /* access category */
	char isbss;                   /* To check whether wmm params are for AP/STA */
	int wmm_val;                  /* value to be set */
	int wmm_end_param;            /* no more wmm params to enter */
};



/*
 * Per/node (station) statistics.
 */
struct ieee80211_nodestats 
{
	unsigned int  ns_rx_data;         /* rx data frames */
	unsigned int  ns_rx_mgmt;         /* rx management frames */
	unsigned int  ns_rx_ctrl;         /* rx control frames */
	unsigned int  ns_rx_ucast;        /* rx unicast frames */
	unsigned int  ns_rx_mcast;        /* rx multi/broadcast frames */
	unsigned long ns_rx_bytes;        /* rx data count (bytes) */
	unsigned long ns_rx_beacons;      /* rx beacon frames */
	unsigned int  ns_rx_proberesp;    /* rx probe response frames */

	unsigned int  ns_rx_dup;          /* rx discard 'cuz dup */
	unsigned int  ns_rx_noprivacy;    /* rx w/ wep but privacy off */
	unsigned int  ns_rx_wepfail;      /* rx wep processing failed */
	unsigned int  ns_rx_demicfail;    /* rx demic failed */
	unsigned int  ns_rx_decap;        /* rx decapsulation failed */
	unsigned int  ns_rx_defrag;       /* rx defragmentation failed */
	unsigned int  ns_rx_disassoc;     /* rx disassociation */
	unsigned int  ns_rx_deauth;       /* rx deauthentication */
	unsigned int  ns_rx_action;       /* rx action */
	unsigned int  ns_rx_decryptcrc;   /* rx decrypt failed on crc */
	unsigned int  ns_rx_unauth;       /* rx on unauthorized port */
	unsigned int  ns_rx_unencrypted;  /* rx unecrypted w/ privacy */
	unsigned int  ns_rx_drop;         /* rx discard other reason */

	unsigned int  ns_tx_data;         /* tx data frames */
	unsigned int  ns_tx_mgmt;         /* tx management frames */
	unsigned int  ns_tx_ctrl;         /* tx control frames */
	unsigned int  ns_tx_ucast;        /* tx unicast frames */
	unsigned int  ns_tx_mcast;        /* tx multi/broadcast frames */
	unsigned long ns_tx_bytes;        /* tx data count (bytes) */
	unsigned int  ns_tx_probereq;     /* tx probe request frames */

	unsigned int  ns_tx_novlantag;    /* tx discard 'cuz no tag */
	unsigned int  ns_tx_vlanmismatch; /* tx discard 'cuz bad tag */

	unsigned int  ns_ps_discard;      /* ps discard 'cuz of age */

	/* MIB-related state */
	unsigned int  ns_tx_assoc;        /* [re]associations */
	unsigned int  ns_tx_assoc_fail;   /* [re]association failures */
	unsigned int  ns_tx_auth;         /* [re]authentications */
	unsigned int  ns_tx_auth_fail;    /* [re]authentication failures*/
	unsigned int  ns_tx_deauth;       /* deauthentications */
	unsigned int  ns_tx_deauth_code;  /* last deauth reason */
	unsigned int  ns_tx_disassoc;     /* disassociations */
	unsigned int  ns_tx_disassoc_code; /* last disassociation reason */
};


struct stainforeq 
{
	struct ieee80211vap *vap;
	struct ieee80211req_sta_info *si;
	size_t space;
};

struct ieee80211_mimo_info 
{
	char  rssi[IEEE80211_MAX_CHAINS];     /* per-antenna rssi */
	char  noise[IEEE80211_MAX_CHAINS];    /* per-antenna noise floor */
	char  pad[2];
	int   evm[3];           /* EVM data */
};

struct ieee80211req_sta_info 
{
	short  isi_len;                 /* total length (mult of 4) */
	short  isi_ie_off;              /* offset to IE data */
	short  isi_ie_len;              /* IE length */
	unsigned int    isi_jointime;           /* time of assoc/join */
	short  isi_freq;                /* MHz */
	int    isi_flags;               /* channel flags */
	int    isi_state;               /* state flags */
	char   isi_authmode;            /* authentication algorithm */
	unsigned int   isi_rssi;                /* receive signal strength */
	char   isi_noise;               /* noise floor */
	char   isi_capinfo;             /* capabilities */
	char   isi_erp;                 /* ERP element */
	unsigned char  isi_macaddr[IEEE80211_ADDR_LEN];
	//uint8_t  isi_macaddr[IEEE80211_ADDR_LEN];
	char   isi_nrates;
	/* negotiated rates */
	char    isi_rates[IEEE80211_RATE_MAXSIZE];
	unsigned int    isi_txrate;             /* legacy/IEEE rate or MCS */
	short   isi_associd;            /* assoc response */
	short   isi_txpower;            /* current tx power */
	short   isi_vlan;               /* vlan tag */
	/* NB: [IEEE80211_NONQOS_TID] holds seq#'s for non-QoS stations */
	short   isi_txseqs[IEEE80211_TID_SIZE];/* tx seq #/TID */
	short   isi_rxseqs[IEEE80211_TID_SIZE];/* rx seq#/TID */
	short   isi_inact;              /* inactivity timer */
	short   isi_txmbps;             /* current tx rate in .5 Mb/s */
	short   isi_pad;
	struct ieee80211_mimo_info isi_mimo; /* MIMO info for 11n sta's */
	/* 11s info */
	short   isi_peerid;
	short   isi_localid;
	char    isi_peerstate;
	/* XXX frag state? */
	/* variable length IE data */
};

struct ieee80211_stats{

	int   is_rx_badversion;	/* rx frame with bad version */
	int   is_rx_tooshort;		/* rx frame too short */
	int   is_rx_wrongbss;		/* rx from wrong bssid */
	int   is_rx_dup;		/* rx discard 'cuz dup */
	int   is_rx_wrongdir;		/* rx w/ wrong direction */
	int   is_rx_mcastecho;	/* rx discard 'cuz mcast echo */
	int   is_rx_notassoc;		/* rx discard 'cuz sta !assoc */
	int   is_rx_noprivacy;	/* rx w/ wep but privacy off */
	int   is_rx_unencrypted;	/* rx w/o wep and privacy on */
	int   is_rx_wepfail;		/* rx wep processing failed */
	int   is_rx_decap;		/* rx decapsulation failed */
	int   is_rx_mgtdiscard;	/* rx discard mgt frames */
	int   is_rx_ctl;		/* rx ctrl frames */
	int   is_rx_beacon;		/* rx beacon frames */
	int   is_rx_rstoobig;		/* rx rate set truncated */
	int   is_rx_elem_missing;	/* rx required element missing*/
	int   is_rx_elem_toobig;	/* rx element too big */
	int   is_rx_elem_toosmall;	/* rx element too small */
	int   is_rx_elem_unknown;	/* rx element unknown */
	int   is_rx_badchan;		/* rx frame w/ invalid chan */
	int   is_rx_chanmismatch;	/* rx frame chan mismatch */
	int   is_rx_nodealloc;	/* rx frame dropped */
	int   is_rx_ssidmismatch;	/* rx frame ssid mismatch  */
	int   is_rx_auth_unsupported;	/* rx w/ unsupported auth alg */
	int   is_rx_auth_fail;	/* rx sta auth failure */
	int   is_rx_auth_countermeasures;/* rx auth discard 'cuz CM */
	int   is_rx_assoc_bss;	/* rx assoc from wrong bssid */
	int   is_rx_assoc_notauth;	/* rx assoc w/o auth */
	int   is_rx_assoc_capmismatch;/* rx assoc w/ cap mismatch */
	int   is_rx_assoc_norate;	/* rx assoc w/ no rate match */
	int   is_rx_assoc_badwpaie;	/* rx assoc w/ bad WPA IE */
	int   is_rx_deauth;		/* rx deauthentication */
	int   is_rx_disassoc;		/* rx disassociation */
	int   is_rx_badsubtype;	/* rx frame w/ unknown subtype*/
	int   is_rx_nobuf;		/* rx failed for lack of buf */
	int   is_rx_decryptcrc;	/* rx decrypt failed on crc */
	int   is_rx_ahdemo_mgt;	/* rx discard ahdemo mgt frame*/
	int   is_rx_bad_auth;		/* rx bad auth request */
	int   is_rx_unauth;		/* rx on unauthorized port */
	int   is_rx_badkeyid;		/* rx w/ incorrect keyid */
	int   is_rx_ccmpreplay;	/* rx seq# violation (CCMP) */
	int   is_rx_ccmpformat;	/* rx format bad (CCMP) */
	int   is_rx_ccmpmic;		/* rx MIC check failed (CCMP) */
	int   is_rx_tkipreplay;	/* rx seq# violation (TKIP) */
	int   is_rx_tkipformat;	/* rx format bad (TKIP) */
	int   is_rx_tkipmic;		/* rx MIC check failed (TKIP) */
	int   is_rx_tkipicv;		/* rx ICV check failed (TKIP) */
	int   is_rx_badcipher;	/* rx failed 'cuz key type */
	int   is_rx_nocipherctx;	/* rx failed 'cuz key !setup */
	int   is_rx_acl;		/* rx discard 'cuz acl policy */
	int   is_tx_nobuf;		/* tx failed for lack of buf */
	int   is_tx_nonode;		/* tx failed for no node */
	int   is_tx_unknownmgt;	/* tx of unknown mgt frame */
	int   is_tx_badcipher;	/* tx failed 'cuz key type */
	int   is_tx_nodefkey;		/* tx failed 'cuz no defkey */
	int   is_tx_noheadroom;	/* tx failed 'cuz no space */
	int   is_tx_fragframes;	/* tx frames fragmented */
	int   is_tx_frags;		/* tx fragments created */
	int   is_scan_active;		/* active scans started */
	int   is_scan_passive;	/* passive scans started */
	int   is_node_timeout;	/* nodes timed out inactivity */
	int   is_crypto_nomem;	/* no memory for crypto ctx */
	int   is_crypto_tkip;		/* tkip crypto done in s/w */
	int   is_crypto_tkipenmic;	/* tkip en-MIC done in s/w */
	int   is_crypto_tkipdemic;	/* tkip de-MIC done in s/w */
	int   is_crypto_tkipcm;	/* tkip counter measures */
	int   is_crypto_ccmp;		/* ccmp crypto done in s/w */
	int   is_crypto_wep;		/* wep crypto done in s/w */
	int   is_crypto_setkey_cipher;/* cipher rejected key */

	int   is_crypto_setkey_nokey;	/* no key index for setkey */
	int   is_crypto_delkey;	/* driver key delete failed */
	int   is_crypto_badcipher;	/* unknown cipher */
	int   is_crypto_nocipher;	/* cipher not available */
	int   is_crypto_attachfail;	/* cipher attach failed */
	int   is_crypto_swfallback;	/* cipher fallback to s/w */
	int   is_crypto_keyfail;	/* driver key alloc failed */
	int   is_crypto_enmicfail;	/* en-MIC failed */
	int   is_ibss_capmismatch;	/* merge failed-cap mismatch */
	int   is_ibss_norate;		/* merge failed-rate mismatch */
	int   is_ps_unassoc;		/* ps-poll for unassoc. sta */
	int   is_ps_badaid;		/* ps-poll w/ incorrect aid */
	int   is_ps_qempty;		/* ps-poll w/ nothing to send */
	int   is_ff_badhdr;		/* fast frame rx'd w/ bad hdr */
	int   is_ff_tooshort;		/* fast frame rx decap error */
	int   is_ff_split;		/* fast frame rx split error */
	int   is_ff_decap;		/* fast frames decap'd */
	int   is_ff_encap;		/* fast frames encap'd for tx */
	int   is_rx_badbintval;	/* rx frame w/ bogus bintval */
	int   is_rx_demicfail;	/* rx demic failed */
	int   is_rx_defrag;		/* rx defragmentation failed */
	int   is_rx_mgmt;		/* rx management frames */
	int   is_rx_action;		/* rx action mgt frames */
	int   is_amsdu_tooshort;	/* A-MSDU rx decap error */
	int   is_amsdu_split;		/* A-MSDU rx split error */
	int   is_amsdu_decap;		/* A-MSDU decap'd */
	int   is_amsdu_encap;		/* A-MSDU encap'd for tx */
	int   is_ampdu_bar_bad;	/* A-MPDU BAR out of window */
	int   is_ampdu_bar_oow;	/* A-MPDU BAR before ADDBA */
	int   is_ampdu_bar_move;	/* A-MPDU BAR moved window */
	int   is_ampdu_bar_rx;	/* A-MPDU BAR frames handled */
	int   is_ampdu_rx_flush;	/* A-MPDU frames flushed */
	int   is_ampdu_rx_oor;	/* A-MPDU frames out-of-order */
	int   is_ampdu_rx_copy;	/* A-MPDU frames copied down */
	int   is_ampdu_rx_drop;	/* A-MPDU frames dropped */
	int   is_tx_badstate;		/* tx discard state != RUN */
	int   is_tx_notassoc;		/* tx failed, sta not assoc */
	int   is_tx_classify;		/* tx classification failed */
	int   is_dwds_mcast;		/* discard mcast over dwds */
	int   is_dwds_qdrop;		/* dwds pending frame q full */
	int   is_ht_assoc_nohtcap;	/* non-HT sta rejected */
	int   is_ht_assoc_downgrade;	/* HT sta forced to legacy */
	int   is_ht_assoc_norate;	/* HT assoc w/ rate mismatch */
	int   is_ampdu_rx_age;	/* A-MPDU sent up 'cuz of age */
	int   is_ampdu_rx_move;	/* A-MPDU MSDU moved window */
	int   is_addba_reject;	/* ADDBA reject 'cuz disabled */
	int   is_addba_norequest;	/* ADDBA response w/o ADDBA */
	int   is_addba_badtoken;	/* ADDBA response w/ wrong
	                                  dialogtoken */
	int   is_addba_badpolicy;	/* ADDBA resp w/ wrong policy */
	int   is_ampdu_stop;		/* A-MPDU stream stopped */
	int   is_ampdu_stop_failed;	/* A-MPDU stream not running */
	int   is_ampdu_rx_reorder;	/* A-MPDU held for rx reorder */
	int   is_scan_bg;		/* background scans started */
	int   is_rx_deauth_code;	/* last rx'd deauth reason */
	int   is_rx_disassoc_code;	/* last rx'd disassoc reason */
	int   is_rx_authfail_code;	/* last rx'd auth fail reason */
	int   is_beacon_miss;		/* beacon miss notification */
	int   is_rx_badstate;		/* rx discard state != RUN */
	int   is_ff_flush;		/* ff's flush'd from stageq */
	int   is_tx_ctl;		/* tx ctrl frames */
	int   is_ampdu_rexmt;		/* A-MPDU frames rexmt ok */
	int   is_ampdu_rexmt_fail;	/* A-MPDU frames rexmt fail */

	int   is_mesh_wrongmesh;	/* dropped 'cuz not mesh sta*/
	int   is_mesh_nolink;		/* dropped 'cuz link not estab*/
	int   is_mesh_fwd_ttl;	/* mesh not fwd'd 'cuz ttl 0 */
	int   is_mesh_fwd_nobuf;	/* mesh not fwd'd 'cuz no mbuf*/
	int   is_mesh_fwd_tooshort;	/* mesh not fwd'd 'cuz no hdr */
	int   is_mesh_fwd_disabled;	/* mesh not fwd'd 'cuz disabled */
	int   is_mesh_fwd_nopath;	/* mesh not fwd'd 'cuz path unknown */
	int   is_hwmp_wrongseq;	/* wrong hwmp seq no. */
	int   is_hwmp_rootreqs;	/* root PREQs sent */
	int   is_hwmp_rootrann;	/* root RANNs sent */

	int   is_mesh_badae;		/* dropped 'cuz invalid AE */
	int   is_mesh_rtaddfailed;	/* route add failed */
	int   is_mesh_notproxy;	/* dropped 'cuz not proxying */
	int   is_rx_badalign;		/* dropped 'cuz misaligned */
	int   is_hwmp_proxy;		/* PREP for proxy route */
	int   is_spare[11]; 
};

struct minstrel_rate 
{ 
	/* Free BSD */
	int bitrate;
	int rix;
	int hix; // Here we will store the index expected by hardware(ppe)
	
	unsigned int perfect_tx_time;
	unsigned int ack_time;
	
	int sample_limit;
	unsigned int retry_count;
	unsigned int retry_count_cts;
	unsigned int retry_count_rtscts;
	unsigned int adjusted_retry_count;
	
	unsigned int success;
	unsigned int attempts;
	unsigned int last_attempts;
	unsigned int last_success;
	
	/* parts per thousand */
	unsigned int cur_prob;
	unsigned int probability;
	
	/* per-rate throughput */
	unsigned int cur_tp;
	unsigned int throughput;
	
	unsigned int succ_hist;
	unsigned int att_hist;
};

/* per-node state */
struct minstrel_node 
{
	unsigned int static_rate_ndx; /*User has bypassed dynamic selection. Fix on one rate */
	
	unsigned int current_rate;

	/* Free BSD */
	unsigned long stats_update; /* It will be holding jiffies so shud be long*/
	unsigned int sp_ack_dur;
	unsigned int rate_avg;
	
	unsigned int lowest_rix;
	
	unsigned int max_tp_rate;
	unsigned int max_tp_rate2;
	unsigned int max_prob_rate;
	unsigned int packet_count;
	unsigned int sample_count;
	int sample_deferred;
	
	unsigned int sample_idx;
	unsigned int sample_column;
	
	int n_rates;
	struct minstrel_rate *r;
	char prev_sample;
	
	/* sampling table */
	char *sample_table;

};

enum rsi_opmode 
{
	RSI_M_IBSS     = 0,  /* IBSS (adhoc) station */
	RSI_M_STA      = 1,  /* infrastructure station */
	RSI_M_WDS      = 2,  /* WDS link */
	RSI_M_AHDEMO   = 3, /* Old lucent compatible adhoc demo */
	RSI_M_HOSTAP   = 4, /* Software Access Point */
	RSI_M_MONITOR  = 5, /* Monitor mode */
	RSI_M_MBSS     = 6, /* MBSS (Mesh Point) link */
	RSI_M_P2P_GO   = 8,
	RSI_M_P2P      = 9 
};


typedef struct 
{
  //! no. of tx pkts
  unsigned short tx_pkts;
  //! no. of rx pkts
  unsigned short rx_pkts;
  //! no. of tx retries
  unsigned short tx_retries;
  //! no. of pkts that pass crc
  unsigned short crc_pass;
  //! no. of pkts failing crc chk
  unsigned short crc_fail;
  //! no. of times cca got stuck
  unsigned short cca_stk;
  //! no of times cca didn't get stuck
  unsigned short cca_not_stk;
  //! no. of pkt aborts
  unsigned short pkt_abort;
  //! no. of false rx starts
  unsigned short fls_rx_start;
  //! cca idle time
  unsigned short cca_idle;
  //! no. of greenfield pkts
  unsigned short gf_pkts;
  //! no. of high throughput pkts
  unsigned short ht_pkts;
  //! no. of legacy pkts
  unsigned short leg_pkt;
  //! add description
  unsigned short leg_prty_fails;
  //! no. of ht pkts failing crc chk
  unsigned short ht_crc_fails;
  //! add description
  unsigned short sp_rejected;
  //! add description
  unsigned short lp_rejected;
  //! Channel 1 signal power 
  unsigned short ch1_sig_pow;
  //! channel 1 noise power
  unsigned short ch1_noise_pow;
  //! channel 2 signal power
  unsigned short ch2_sig_pow;
  //! channel 2 noise power
  unsigned short ch2_noise_pow;
  //! channel 3 signal power
  unsigned short ch3_sig_pow;
  //! channel 3 noise power
  unsigned short ch3_noise_pow;
  //! no. of rx retries
  unsigned short rx_retries;
  //! rssi value
  unsigned short bea_avg_rssi;
  //! cal_rssi
  unsigned short cal_rssi;
  //! lna_gain bb_gain
  unsigned short lna_bb_gain;
  //! avg_val
  unsigned short avg_val;
 //! xretries pkts dropped
  unsigned short xretries;
 //! consecutive pkts dropped
  unsigned short max_cons_pkts_dropped;
  //! 
  unsigned short false_under_sat;
  //!BSS MATCHED BROADCAST PKT STATS
  unsigned short bss_broadcast_pkts;
  //!BSS MATCHED MULTICAST PKT STATS
  unsigned short bss_multicast_pkts;
  //!BSS and DA MATCHED MULTICAST PKT STATS
  unsigned short bss_da_matched_multicast_pkts;
#ifdef WLAN_PER_PKT_RCV_RATE_STATS
  //!No.of pkts rcvd with mcs0
	unsigned short pkt_rcvd_with_mcs0;
  //!No.of pkts rcvd with mcs1
	unsigned short pkt_rcvd_with_mcs1;
  //!No.of pkts rcvd with mcs2
	unsigned short pkt_rcvd_with_mcs2;
  //!No.of pkts rcvd with mcs3
	unsigned short pkt_rcvd_with_mcs3;
  //!No.of pkts rcvd with mcs4
	unsigned short pkt_rcvd_with_mcs4;
  //!No.of pkts rcvd with mcs5
	unsigned short pkt_rcvd_with_mcs5;
  //!No.of pkts rcvd with mcs6
	unsigned short pkt_rcvd_with_mcs6;
  //!No.of pkts rcvd with mcs7
	uint16 pkt_rcvd_with_mcs7;
  //!No.of pkts rcvd with 48M
	uint16 pkt_rcvd_with_48M;
  //!No.of pkts rcvd with 24M
	uint16 pkt_rcvd_with_24M;
  //!No.of pkts rcvd with 12M
	uint16 pkt_rcvd_with_12M;
  //!No.of pkts rcvd with 6M
	uint16 pkt_rcvd_with_6M;
  //!No.of pkts rcvd with 54M
	uint16 pkt_rcvd_with_54M;
  //!No.of pkts rcvd with 36M
	uint16 pkt_rcvd_with_36M;
  //!No.of pkts rcvd with 18M
	uint16 pkt_rcvd_with_18M;
  //!No.of pkts rcvd with 9M
	uint16 pkt_rcvd_with_9M;
  //!No.of pkts rcvd with 11M
	uint16 pkt_rcvd_with_11M;
  //!No.of pkts rcvd with 5.5M
	uint16 pkt_rcvd_with_5M;
  //!No.of pkts rcvd with 2M
	uint16 pkt_rcvd_with_2M;
  //!No.of pkts rcvd with 1M
	uint16 pkt_rcvd_with_1M;
#endif
  unsigned int eof_pkt_drop_count;
  unsigned int mask_pkt_drop_count;
  unsigned int ack_sent;
  /* Channel Utilization related stats */
	unsigned int utilization;
	unsigned int rssi_utilization;
	unsigned int tot_bytes;
	unsigned int rssi_bytes;
	unsigned int interval_duration;
	unsigned int false_cca_avg_rssi;
	unsigned int max_cca_avg_rssi;
	unsigned int cca_duration;
	unsigned int ed_duration;
	unsigned short int noise_rssi;
 	int stop_per; 
}per_stats;
typedef struct wlan_iq_struct_s
{
  unsigned int freq;
  unsigned int rate_flags;
  unsigned int no_of_samples;
  unsigned short *iq_stats;
  unsigned int eof;
} wlan_iq_struct_t;
typedef struct gpio_registers
{
  uint32_t read_write;
  uint32_t id;
  uint32_t mode;
  uint32_t value;
  uint32_t direction;
  unsigned int address ;          
}gpio_reg_t;
#define ONEBOX_HOST_IOCTL     SIOCIWLASTPRIV - 0x0B
#define ONEBOX_SET_BB_RF      SIOCIWLASTPRIV - 0x08 
#define ONEBOX_SET_CW_MODE    SIOCIWLASTPRIV - 0x05 
#define WLAN_I_Q_STATS        SIOCIWLASTPRIV - 0xc 


#define PER_TRANSMIT            1
#define PER_PACKET            8
#define PER_RECEIVE             2
#define PER_RCV_STOP            4
#define PER_AGGR_LIMIT_PER_PKT            1792
#define SET_ENDPOINT            7
#define ANT_SEL			9
#define CH_UTIL_START			10
#define CH_UTIL_STOP			11
#define RADAR_DETECT			12
#define RADAR_ENABLE      		13
#define RADAR_PKT			14
#define ANT_TYPE		        15
#define DEFAULT_SP_LEN      0
#define MAX_SP_LEN          3
#define SP_LEN_POS          5
#define DEFAULT_UAPSD_ACS  15
#define WAKEUP_PERIOD_MIN  10
#define WAKEUP_PERIOD_MAX  100

typedef struct per_params_s
{
	unsigned short  enable;
	signed short  power;
	unsigned int    rate;
	unsigned short  pkt_length;
	unsigned short  mode;
	unsigned short  channel;
	unsigned short  rate_flags;
	unsigned short  per_ch_bw;
	unsigned short  aggr_enable;
	unsigned short  aggr_count;
	unsigned short  no_of_pkts;
	unsigned int    delay;
	unsigned char ctry_region;
	unsigned char enable_11j;
}per_params_t;

typedef struct wlan_9116_features_s {
  uint8_t pll_mode;
  uint8_t rf_type;
  uint8_t wireless_mode;
  uint8_t afe_type;
  uint8_t enable_ppe;
  uint8_t dpd;
  uint8_t SIFSTransmitenable;
  uint32_t pwrsave_options;
}w_9116_features_t;

typedef struct per_packet_s
{
	unsigned char  enable;
	unsigned int  length;
	unsigned char  insert_seq;
	unsigned char packet[1536];
}per_packet_t;

typedef struct prog_structure_s {
  unsigned char prog_type ;
  unsigned char structure_present; 
  unsigned int TA_RAM_ADDRESS;
  unsigned int bb_rf_flags;
  unsigned char len;
} prog_structure_t;

typedef struct eepromrw_info_s
{
	unsigned int    offset;
	unsigned int    length;
	unsigned char   write;
	unsigned short  eeprom_erase;
	unsigned char   data[480];
}eepromrw, EEPROMRW;

#define MAX_EEPROM_LEN  300
EEPROMRW        eeprom;

#define RSI_RATE_00   0x00
#define RSI_RATE_1    0x0
#define RSI_RATE_2    0x2
#define RSI_RATE_5_5  0x4
#define RSI_RATE_11   0x6
#define RSI_RATE_6    0x8b
#define RSI_RATE_9    0x8f
#define RSI_RATE_12   0x8a
#define RSI_RATE_18   0x8e
#define RSI_RATE_24   0x89
#define RSI_RATE_36   0x8d
#define RSI_RATE_48   0x88
#define RSI_RATE_54   0x8c
#define RSI_RATE_MCS0 0x100
#define RSI_RATE_MCS1 0x101
#define RSI_RATE_MCS2 0x102
#define RSI_RATE_MCS3 0x103
#define RSI_RATE_MCS4 0x104
#define RSI_RATE_MCS5 0x105
#define RSI_RATE_MCS6 0x106
#define RSI_RATE_MCS7 0x107
#define RSI_RATE_MCS7_SG 0x307


#define RSI_RATE_1M     1 
#define RSI_RATE_2M     2 
#define RSI_RATE_5_5M   5.5 
#define RSI_RATE_11M    11 
#define RSI_RATE_6M     6 
#define RSI_RATE_9M     9 
#define RSI_RATE_12M    12 
#define RSI_RATE_18M    18 
#define RSI_RATE_24M    24 
#define RSI_RATE_36M    36 
#define RSI_RATE_48M    48 
#define RSI_RATE_54M    54 
#define RSI_RATE_6_5M   mcs0 
#define RSI_RATE_13M    mcs1 
#define RSI_RATE_19_5M  mcs2 
#define RSI_RATE_26M    mcs3 
#define RSI_RATE_39M    mcs4 
#define RSI_RATE_52M    mcs5 
#define RSI_RATE_58_5M  mcs6 
#define RSI_RATE_65M    mcs7 

#define streq(a,b) (strncasecmp(a, b, sizeof(b) - 1) == 0)

#define ONEBOX_PRINT(fmt, args...) fprintf(stdout, fmt, ## args)

#define ONEBOX_PRINT_INFO(a, fmt) \
	if(a)\
		printf(fmt);

#define streq(a,b)  (strncasecmp(a, b, sizeof(b) - 1) == 0)

/* Function prototypes */
void usage();
void printvapstats(struct ieee80211_stats *stat, unsigned char verbose);
void printstationstats(struct ieee80211_nodestats *ni_stats, unsigned char verbose);
void printstationinfo(struct ieee80211req_sta_info *sta_info, unsigned char verbose);
void byteconversion(char *src,char *macaddr);
int getcmdnumber(char *command, char *ifName);
void destroy_vap(int sock, char *ifname);
void get_driver_state(char *state, char *ifname);



///***  MATLAB utils ****///

/*** Matlab To Driver Command Type Defines ***/

#define ONEBOX_STATUS_FAILURE           -1
#define ONEBOX_STATUS_SUCCESS           0
#define ONEBOX_STATUS                   int_32

#define ADDR0           0x315
#define ADDR1           0x316
#define ADDR2           0x317

#define BIT(n)         (1 << (n))

typedef unsigned char   uint_8;
typedef char            int_8;
typedef unsigned short  uint_16;
typedef short           int_16;
typedef unsigned int    uint_32;
typedef int             int_32;

// Read values for MATALB frame types should always be a EVEN NUBMBER
#define BB_READ                         0x0
#define BB_WRITE                        0x1
#define RF_READ                         0x2
#define RF_WRITE                        0x3
#define ULP_READ                        0x4
#define ULP_WRITE                       0x5
#define BUFFER_READ                     0x6
#define BUFFER_WRITE                    0x7
#define RX_STATS                        0x8
#define EEPROM_RF_PROG_WRITE            0x9
#define LMAC_RF_M3                   	  0xC
#define LMAC_RF_M2                    	0xA
#define RF_RESET                        0xB
#define TX_STATS			                  0xD
#define SOC_REG_WRITE                   0xF
#define SOC_REG_READ                    0x10
#define ENDPOINT                        0x11
#define EEPROM_RF_PROG_READ             0x12
#define CALIB_FLASHING                  0x13
#define RADAR_READ			0x14
#define RADAR_PACKET			0x15

/*** Src Destn Port And IP Defines ***/
#define DESTN_PORT_NUM                  6666
#define SOURCE_PORT_NUM                 6666
#define DESTN_IP                        "192.168.70.222"
//#define DESTN_IP                        "127.0.0.1"
#define ONEBOX_MAX_PKT_LEN              6348 // Bytes
#define MAX_NUM_OF_PKTS                 100

#define IFNAMSIZ                        16

#define READ                           0
#define WRITE                          1   

struct bb_rf_param_t
{
        unsigned short  Data[1024];
        unsigned short  no_of_fields;
        unsigned short  no_of_values;
        unsigned char  value; //type is present here
        unsigned char  soft_reset;
        unsigned char  protocol_id;
} ;
     
#define MAX_NUM_BGCHAN   24
struct bgscan_params
{
	unsigned short bgscan_threshold;
	unsigned short roam_threshold;
	unsigned short bgscan_periodicity;
	unsigned char num_bg_channels;
	unsigned char two_probe;
	unsigned short active_scan_duration;
	unsigned short passive_scan_duration;
	unsigned short channels2scan[MAX_NUM_BGCHAN];
};
struct aggr_params_s
{
	unsigned short tx_limit;
	unsigned short rx_limit;
};

struct master_params_s
{
	unsigned int address;
	unsigned short no_of_bytes;
	unsigned char *data;
  unsigned int read_write;
};

typedef struct coex_cmd_s {
  unsigned char start_stop ;
  unsigned char protocol_id;
  unsigned int slotting_time;
}coex_cmd_t;
typedef struct programming_stats_s {
  unsigned char start_stop ;
  unsigned long int interval;
} programming_stats_t;
struct test_mode
{
	unsigned short subtype;
	unsigned short args;
};

#ifdef IEEE80211K
#define CHANNEL_LOAD 3
#define BEACON_REPORT 5
#define FRAME_REQUEST 6
#define MCAST_DIG_REQUEST 10
#endif
struct wowlan_config
{
  unsigned char macaddr[IEEE80211_ADDR_LEN];
  unsigned short flags;
  unsigned short host_wakeup_state;
};

typedef struct ipmu_params_s {
    unsigned int address;
    unsigned int value;
    unsigned char mode ;
} ipmu_params_t;

#ifdef IEEE80211K
struct msrmnt_req_params 
{
	uint8_t frame_type,ssid_len;
	unsigned char ucast_macaddr[IEEE80211_ADDR_LEN];

	union
	{
	  unsigned char frame_req_macaddr[IEEE80211_ADDR_LEN];
	  unsigned char bssid_macaddr[IEEE80211_ADDR_LEN];
	};
	
	uint8_t ssid_beacon_rpt[32];	
}; 
#endif
/* PUF related defines and structures */
//! PUF key related size defines
#define ACTIVATION_CODE_SIZE  1192
#define MAX_KEY_SIZE    32
#define KEY_CODE_SIZE   44
#define MAX_IV_SIZE     32

//! PUF operation defines
#define PUF_ENROLL            0
#define PUF_START             1
#define PUF_SET_KEY           2
#define PUF_SET_INTRINSIC_KEY 3
#define PUF_GET_KEY           4
#define PUF_LOAD_KEY          5
#define PUF_AES_ENCRYPT       6
#define PUF_AES_DECRYPT       7
#define PUF_AES_MAC           8
#define PUF_BLOCK_ENROLL      9
#define PUF_BLOCK_SET_KEY     10
#define PUF_BLOCK_GET_KEY     11

//! structure used for puf enroll and start operation
struct puf_init_params_s
{
  unsigned char puf_sub_cmd;
  unsigned char puf_ac_source;
  unsigned char *activation_code;
};

//! structure used for puf set key and set intrinsic key operation
struct puf_set_key_params_s
{
  unsigned char puf_sub_cmd;
  unsigned char key_index;
  unsigned char key_size;
  unsigned char key[MAX_KEY_SIZE];
  unsigned char key_code[KEY_CODE_SIZE];
};

//! structure used for puf get key and load key operation
struct puf_get_key_params_s
{
  unsigned char puf_sub_cmd;
  unsigned char key_code[KEY_CODE_SIZE];
  unsigned char key_holder;
  unsigned short key_size;
  unsigned char key[MAX_KEY_SIZE];
};

//! structure used for puf AES encryption, decryption and mac generation operation
struct puf_aes_data_params_s
{
  unsigned char puf_sub_cmd;
  unsigned char mode;
  unsigned char key_source;
  unsigned char key_size;
  unsigned char iv_size;
  unsigned char key[MAX_KEY_SIZE];
  unsigned char iv[MAX_IV_SIZE];
  unsigned short data_size;
  unsigned char *data;
  unsigned char *enc_dec_mac_data;
};
/* PUF END */
int get_zigb_state (void);
int get_bt_state (void);

#define FUNCTION_ENTRY()    NULL//            printf("+%s\n", __func__)
#define FUNCTION_EXIT()     NULL//            printf("-%s\n", __func__)

int getopmode (const char *s);
int verify_flash( char *ifName, int sfd, int len);

#endif

int master_ops(uint_8 type, uint_32 addr, uint_8 *data,uint_8 len);


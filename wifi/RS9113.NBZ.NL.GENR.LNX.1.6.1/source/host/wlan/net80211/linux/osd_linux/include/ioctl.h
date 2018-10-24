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

#ifndef __OSD_IOCTL_H__
#define __OSD_IOCTL_H__
//#include <net80211/ieee80211_ioctl.h>
#include <linux/wireless.h>
#include <net/iw_handler.h>


struct ieee80211req_getset_appiebuf 
{
        u_int32_t       app_frmtype;            /* management frame type for which buffer is added */
        u_int32_t       app_buflen;             /* application-supplied buffer length */
        u_int8_t        app_buf[0];             /* application-supplied IE(s) */
};


#define IW_PRIV_BLOB_LENGTH_ENCODING(_SIZE) \
     (((_SIZE) == ((_SIZE) & IW_PRIV_SIZE_MASK)) ? \
        (_SIZE) : \
        (((_SIZE) / sizeof(uint32_t)) + \
           (((_SIZE) == (((_SIZE) / sizeof(uint32_t)) * sizeof(int))) ? \
           0 : 1)))
#define IW_PRIV_BLOB_TYPE_ENCODING(_SIZE) \
       (((_SIZE) == ((_SIZE) & IW_PRIV_SIZE_MASK)) ? \
               (IW_PRIV_TYPE_BYTE | (_SIZE)) : \
               (IW_PRIV_TYPE_INT  | IW_PRIV_BLOB_LENGTH_ENCODING((_SIZE))))

struct ieee80211req_set_filter 
{
        u_int32_t app_filterype;                /* management frame filter type */
}; 


#define SIOCGDRVSPEC   -122
#define SIOCSDRVSPEC   -123
#define SIOCGPRIVATE_0 -124

#define IEEE80211_APPIE_MAX     1024

#define IW_PRIV_TYPE_OPTIE \
	IW_PRIV_BLOB_TYPE_ENCODING(IEEE80211_MAX_OPT_IE)
#define IW_PRIV_TYPE_KEY  \
	IW_PRIV_BLOB_TYPE_ENCODING(sizeof(struct ieee80211req_key))
#define IW_PRIV_TYPE_DELKEY \
	IW_PRIV_BLOB_TYPE_ENCODING(sizeof(struct ieee80211req_del_key))
#define IW_PRIV_TYPE_MLME \
	IW_PRIV_BLOB_TYPE_ENCODING(sizeof(struct ieee80211req_mlme))
#define IW_PRIV_TYPE_CHANLIST \
	IW_PRIV_BLOB_TYPE_ENCODING(sizeof(struct ieee80211req_chanlist))
#define IW_PRIV_TYPE_CHANINFO \
	IW_PRIV_BLOB_TYPE_ENCODING(sizeof(struct ieee80211req_chaninfo))
#define IW_PRIV_TYPE_APPIEBUF \
	IW_PRIV_BLOB_TYPE_ENCODING(sizeof(struct ieee80211req_getset_appiebuf) + IEEE80211_APPIE_MAX)
#define IW_PRIV_TYPE_FILTER \
	IW_PRIV_BLOB_TYPE_ENCODING(sizeof(struct ieee80211req_set_filter))

#define IEEE80211_IOCTL_SETPARAM        (SIOCIWFIRSTPRIV+0)
#define IEEE80211_IOCTL_GETPARAM        (SIOCIWFIRSTPRIV+1)
#define IEEE80211_IOCTL_SETMODE         (SIOCIWFIRSTPRIV+2)
#define IEEE80211_IOCTL_GETMODE         (SIOCIWFIRSTPRIV+3)
#define IEEE80211_IOCTL_SETWMMPARAMS    (SIOCIWFIRSTPRIV+4)
#define IEEE80211_IOCTL_GETWMMPARAMS    (SIOCIWFIRSTPRIV+5)
#define IEEE80211_IOCTL_SETCHANLIST     (SIOCIWFIRSTPRIV+6)
#define IEEE80211_IOCTL_GETCHANLIST     (SIOCIWFIRSTPRIV+7)
#define IEEE80211_IOCTL_CHANSWITCH      (SIOCIWFIRSTPRIV+8)
#define IEEE80211_IOCTL_GET_APPIEBUF    (SIOCIWFIRSTPRIV+9)
#define IEEE80211_IOCTL_SET_APPIEBUF    (SIOCIWFIRSTPRIV+10)
#define SIOCG80211                      (SIOCIWFIRSTPRIV+11)
#define SIOCSIFMEDIA                    (SIOCIWFIRSTPRIV+12)
#define IEEE80211_IOCTL_GETCHANINFO     (SIOCIWFIRSTPRIV+13)
#define IEEE80211_IOCTL_SETENCRYPTION   (SIOCIWFIRSTPRIV+13)
#define IEEE80211_IOCTL_SETOPTIE        (SIOCIWFIRSTPRIV+14)
#define IEEE80211_IOCTL_GETOPTIE        (SIOCIWFIRSTPRIV+15)
#define IEEE80211_IOCTL_SETMLME         (SIOCIWFIRSTPRIV+16)
#define SIOCG80211STASTATS              (SIOCIWFIRSTPRIV+17)
#define IEEE80211_IOCTL_RADAR           (SIOCIWFIRSTPRIV+17) //: look for alternate ioctl number

#define IEEE80211_IOCTL_SETKEY          (SIOCIWFIRSTPRIV+18)
#define SIOCGIFMEDIA                    (SIOCIWFIRSTPRIV+19)
#define IEEE80211_IOCTL_DELKEY          (SIOCIWFIRSTPRIV+20)
#define IEEE80211_IOCTL_ADDMAC          (SIOCIWFIRSTPRIV+22)
#define SIOCG80211STATS                 (SIOCIWFIRSTPRIV+23)	
#define IEEE80211_IOCTL_DELMAC          (SIOCIWFIRSTPRIV+24)
#define IEEE80211_IOCTL_WDSADDMAC       (SIOCIWFIRSTPRIV+33) //Moving value from 25 to 33 at present, handling for this ioctl call is not present
#define IEEE80211_IOCTL_WDSSETMAC       (SIOCIWFIRSTPRIV+26)
#define SIOCG80211STAINFO               (SIOCIWFIRSTPRIV+27)
#define SIOCS80211                      (SIOCIWFIRSTPRIV+28)
#define IEEE80211_IOCTL_KICKMAC         (SIOCIWFIRSTPRIV+29)
#define IEEE80211_IOCTL_FILTERFRAME     (SIOCIWFIRSTPRIV+30)
#define IEEE80211_IOCTL_SETSCANLIST     (SIOCIWFIRSTPRIV+31)
#ifdef IEEE80211K
#define IEEE80211_IOCTL_11K_MSRMNT_REQ  (SIOCIWFIRSTPRIV+26)
#define IEEE80211_IOCTL_11K_MSRMNT_RPT  (SIOCIWFIRSTPRIV+32)
#define IEEE80211_IOCTL_11K_SM_MSRMNT_REQ  (SIOCIWFIRSTPRIV+25)
#endif

#ifdef ONEBOX_CONFIG_CFG80211
#define NL80211_SET_IOCTL             0x8BFE      // FILTERFRAME
#define NL80211_GET_IOCTL             0x8BFD      // KICKMAC
#endif

enum 
{
	IEEE80211_WMMPARAMS_CWMIN       = 1,
	IEEE80211_WMMPARAMS_CWMAX       = 2,
	IEEE80211_WMMPARAMS_AIFS        = 3,
	IEEE80211_WMMPARAMS_TXOPLIMIT   = 4,
	IEEE80211_WMMPARAMS_ACM         = 5,
	IEEE80211_WMMPARAMS_NOACKPOLICY = 6,
};
typedef enum 
{
	IEEE80211_PARAM_TURBO           = 1,
	IEEE80211_PARAM_MODE            = 2,
	IEEE80211_PARAM_AUTHMODE        = 3,
	IEEE80211_PARAM_PROTMODE        = 4,
	IEEE80211_PARAM_MCASTCIPHER     = 5,
	IEEE80211_PARAM_MCASTKEYLEN     = 6,
	IEEE80211_PARAM_UCASTCIPHERS    = 7,
	IEEE80211_PARAM_UCASTCIPHER     = 8,
	IEEE80211_PARAM_UCASTKEYLEN     = 9,
	IEEE80211_PARAM_WPA             = 10,
	IEEE80211_PARAM_ROAMING         = 12,
	IEEE80211_PARAM_PRIVACY         = 13,
	IEEE80211_PARAM_COUNTERMEASURES = 14,
	IEEE80211_PARAM_DROPUNENCRYPTED = 15,
	IEEE80211_PARAM_DRIVER_CAPS     = 16,
	IEEE80211_PARAM_MACCMD          = 17,
	IEEE80211_PARAM_WMM             = 18,
	IEEE80211_PARAM_HIDESSID        = 19,
	IEEE80211_PARAM_APBRIDGE        = 20,
	IEEE80211_PARAM_KEYMGTALGS      = 21,
	IEEE80211_PARAM_RSNCAPS         = 22,
	IEEE80211_PARAM_INACT           = 23,
	IEEE80211_PARAM_INACT_AUTH      = 24,
	IEEE80211_PARAM_INACT_INIT      = 25,
	IEEE80211_PARAM_ABOLT           = 26,
	IEEE80211_PARAM_DTIM_PERIOD     = 28,
	IEEE80211_PARAM_BEACON_INTERVAL = 29,
	IEEE80211_PARAM_DOTH            = 30,
	IEEE80211_PARAM_PWRTARGET       = 31,
	IEEE80211_PARAM_GENREASSOC      = 32,
	IEEE80211_PARAM_COMPRESSION     = 33,
	IEEE80211_PARAM_FF              = 34,
	IEEE80211_PARAM_XR              = 35,
	IEEE80211_PARAM_BURST           = 36,
	IEEE80211_PARAM_PUREG           = 37,
	IEEE80211_PARAM_AR              = 38,
	IEEE80211_PARAM_WDS             = 39,
	IEEE80211_PARAM_BGSCAN          = 40,
	IEEE80211_PARAM_BGSCAN_IDLE     = 41,
	IEEE80211_PARAM_BGSCAN_INTERVAL = 42,
	IEEE80211_PARAM_MCAST_RATE      = 43,
	IEEE80211_PARAM_COVERAGE_CLASS  = 44,
	IEEE80211_PARAM_COUNTRY_IE      = 45,
	IEEE80211_PARAM_SCANVALID       = 46,
	IEEE80211_PARAM_ROAM_RSSI_11A   = 47,
	IEEE80211_PARAM_ROAM_RSSI_11B   = 48,
	IEEE80211_PARAM_ROAM_RSSI_11G   = 49,
	IEEE80211_PARAM_ROAM_RATE_11A   = 50,
	IEEE80211_PARAM_ROAM_RATE_11B   = 51,
	IEEE80211_PARAM_ROAM_RATE_11G   = 52,
	IEEE80211_PARAM_UAPSDINFO       = 53,
	IEEE80211_PARAM_SLEEP           = 54,
	IEEE80211_PARAM_QOSNULL         = 55,
	IEEE80211_PARAM_PSPOLL          = 56,
	IEEE80211_PARAM_EOSPDROP        = 57,
	IEEE80211_PARAM_MARKDFS         = 58,
	IEEE80211_PARAM_REGCLASS        = 59,
	IEEE80211_PARAM_DROPUNENC_EAPOL = 60,
	IEEE80211_PARAM_SHPREAMBLE      = 61,
	IEEE80211_PARAM_DUMPREGS        = 62,
	IEEE80211_PARAM_DOTH_ALGORITHM  = 63,
	IEEE80211_PARAM_DOTH_MINCOM     = 64,
	IEEE80211_PARAM_DOTH_SLCG       = 65,
	IEEE80211_PARAM_DOTH_SLDG       = 66,
	IEEE80211_PARAM_TXCONT          = 67,
	IEEE80211_PARAM_TXCONT_RATE     = 68,
	IEEE80211_PARAM_TXCONT_POWER    = 69,
	IEEE80211_PARAM_DFS_TESTMODE    = 70,
	IEEE80211_PARAM_DFS_CACTIME     = 71,
	IEEE80211_PARAM_DFS_EXCLPERIOD  = 72,
	IEEE80211_PARAM_BEACON_MISS_THRESH      = 73,
	IEEE80211_PARAM_BEACON_MISS_THRESH_MS   = 74,
	IEEE80211_PARAM_MAXRATE         = 75,
	IEEE80211_PARAM_MINRATE         = 76,
	IEEE80211_PARAM_PROTMODE_RSSI   = 77,
	IEEE80211_PARAM_PROTMODE_TIMEOUT= 78,
	IEEE80211_PARAM_BGSCAN_THRESH   = 79,
	IEEE80211_PARAM_RSSI_DIS_THR    = 80, 
	IEEE80211_PARAM_RSSI_DIS_COUNT  = 81,
	IEEE80211_PARAM_WDS_SEP         = 82,
	IEEE80211_PARAM_MAXASSOC        = 83,
	IEEE80211_PARAM_PROBEREQ        = 84,
	IEEE80211_PARAM_CHANGE_COUNTRY_IE = 85,
	IEEE80211_PARAM_SHORT_GI        = 86,
	IEEE80211_PARAM_RIFS            = 87,
	IEEE80211_PARAM_GREEN_FIELD     = 88,
	IEEE80211_PARAM_AP_IOSOLATION   = 89,
	IEEE80211_PARAM_AMPDU           = 90,
	IEEE80211_PARAM_AMPDU_LIMIT     = 91,
	IEEE80211_PARAM_AMPDU_DENSITY   = 92,
	IEEE80211_PARAM_AMSDU           = 93,
	IEEE80211_PARAM_AMSDU_LIMIT     = 94,
	IEEE80211_PARAM_SMPS            = 95,
	IEEE80211_PARAM_HTCONF          = 96,
	IEEE80211_PARAM_HTPROTMODE      = 97,
	IEEE80211_PARAM_DOTD            = 98,
	IEEE80211_PARAM_DFS             = 99,
	IEEE80211_PARAM_TSN             = 100,
	IEEE80211_PARAM_WPS             = 101,
	IEEE80211_PARAM_HTCOMPAT        = 102,
	IEEE80211_PARAM_PUREN           = 103,
	IEEE80211_PARAM_TXPOWER         = 104,
#ifdef ONEBOX_CONFIG_CFG80211
	IEEE80211_PARAM_RATE            = 105,
	IEEE80211_PARAM_NAME            = 106,
#endif
	IEEE80211_PARAM_BW            = 107,
	IEEE80211_PARAM_RTP_PRINTS            = 108,
	IEEE80211_DFS_CHAN_TO_SWITCH	= 118,
	IEEE80211_KEEP_ALIVE_PERIOD    = 119,
	IEEE80211_PARAM_DBG_ZONE    = 120,
	IEEE80211_DEFAULT_MGMT_RATE = 121,
#ifdef IEEE80211K
	  IEEE80211_11K_MSRMNT_REQ        = 122,
#endif 
}enum_list;
#define IS_UP_AUTO(_vap) \
	(IFNET_IS_UP_RUNNING((_vap)->iv_ifp) && \
	 (_vap)->iv_roaming == IEEE80211_ROAMING_AUTO)


#ifndef ifr_media
#define ifr_media       ifr_ifru.ifru_ivalue
#endif

#define IFM_IEEE80211_OFDM1_50  10      /* OFDM 1.5Mbps */
#define IFM_IEEE80211_OFDM2_25  11      /* OFDM 2.25Mbps */
#define IFM_IEEE80211_OFDM4_50  13      /* OFDM 4.5Mbps */
#define IFM_IEEE80211_OFDM13_5  17      /* OFDM 13.5Mpbs */


#define IEEE80211_RATE_AUTO     0
#define IEEE80211_RATE_1M       2
#define IEEE80211_RATE_2M       4
#define IEEE80211_RATE_5_5M     11
#define IEEE80211_RATE_11M      22
#define IEEE80211_RATE_6M       12
#define IEEE80211_RATE_9M       18
#define IEEE80211_RATE_12M      24
#define IEEE80211_RATE_18M      36
#define IEEE80211_RATE_24M      48
#define IEEE80211_RATE_36M      72
#define IEEE80211_RATE_48M      96
#define IEEE80211_RATE_54M      108
#define IEEE80211_RATE_6_5M     13
#define IEEE80211_RATE_13M      26
#define IEEE80211_RATE_19_5M    39
#define IEEE80211_RATE_26M      52
#define IEEE80211_RATE_39M      78
#define IEEE80211_RATE_52M      104
#define IEEE80211_RATE_58_5M    117
#define IEEE80211_RATE_65M      130

/* SHORT GI RATES*/
#define IEEE80211_RATE_MCS7_SGI      144
#define IEEE80211_RATE_MCS6_SGI      130
#define IEEE80211_RATE_MCS5_SGI      116
#define IEEE80211_RATE_MCS4_SGI      87
#define IEEE80211_RATE_MCS3_SGI      58
#define IEEE80211_RATE_MCS2_SGI      44
#define IEEE80211_RATE_MCS1_SGI      29
#define IEEE80211_RATE_MCS0_SGI     15

#define IEEE80211_RATE_MCS7_SGI_40   300
#define IEEE80211_RATE_MCS6_SGI_40   270
#define IEEE80211_RATE_MCS5_SGI_40   240
#define IEEE80211_RATE_MCS4_SGI_40   180
#define IEEE80211_RATE_MCS3_SGI_40   120
#define IEEE80211_RATE_MCS2_SGI_40   90
#define IEEE80211_RATE_MCS1_SGI_40   60
#define IEEE80211_RATE_MCS0_SGI_40   30

#define IEEE80211_RATE_MCS7_40      270
#define IEEE80211_RATE_MCS6_40      243
#define IEEE80211_RATE_MCS5_40      216
#define IEEE80211_RATE_MCS4_40      162
#define IEEE80211_RATE_MCS3_40      108
#define IEEE80211_RATE_MCS2_40      81
#define IEEE80211_RATE_MCS1_40      54
#define IEEE80211_RATE_MCS0_40      27

#define ONEBOX_RATE_1M      0x00
#define ONEBOX_RATE_2M      0x02
#define ONEBOX_RATE_5_5M    0x04
#define ONEBOX_RATE_11M     0x06
#define ONEBOX_RATE_6M      0x4b
#define ONEBOX_RATE_9M      0x4f
#define ONEBOX_RATE_12M     0x4a
#define ONEBOX_RATE_18M     0x4e
#define ONEBOX_RATE_24M     0x49
#define ONEBOX_RATE_36M     0x4d
#define ONEBOX_RATE_48M     0x48
#define ONEBOX_RATE_54M     0x4c
#define ONEBOX_RATE_MCS0    0x80
#define ONEBOX_RATE_MCS1    0x81
#define ONEBOX_RATE_MCS2    0x82
#define ONEBOX_RATE_MCS3    0x83
#define ONEBOX_RATE_MCS4    0x84
#define ONEBOX_RATE_MCS5    0x85
#define ONEBOX_RATE_MCS6    0x86
#define ONEBOX_RATE_MCS7    0x87
#define IEEE80211_MODE_TURBO_STATIC_A   IEEE80211_MODE_MAX
#define MCS_RATE            BIT(8) 
#define SGI_ENABLE          BIT(9)
#define MCS_INDEX           0x7  /** [MCS0 - MCS7] **/   
struct ieee80211_channel;
int check_mode_consistency(const struct ieee80211_channel *c, int mode);


#ifdef ONEBOX_CONFIG_CFG80211
int
ieee80211_ioctl_siwrate(struct net_device *dev, 
                        struct iw_request_info *info,
                        struct iw_param *rrq, 
                        char *extra);
int
ieee80211_ioctl_setparam(struct net_device *dev, 
                         struct iw_request_info *info,
                         void *w, 
                         char *extra);
int
ieee80211_ioctl_getparam(struct net_device *dev,
                         struct iw_request_info *info,
                         void *w, 
                         char *extra);

int
ieee80211_ioctl_setmode(struct net_device *dev, struct iw_request_info *info,
                        struct iw_point *wri, char *extra);
int
ieee80211_ioctl_getmode(struct net_device *dev, 
                        struct iw_request_info *info,
                        struct iw_point *wri, 
                        char *extra);
int
ieee80211_ioctl_giwname(struct net_device *dev, 
                        struct iw_request_info *info,
                        char *name, 
                        char *extra);
int
ieee80211_ioctl_giwrate(struct net_device *dev, 
                        struct iw_request_info *info,
                        struct iw_param *rrq, 
                        char *extra);
#endif
#endif

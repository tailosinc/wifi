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

#include <linux/if_arp.h>
#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_ioctl.h>
#include <linux/wireless.h>
#include <net/iw_handler.h>
#include "ioctl.h"
#include "ieee80211_linux.h"
#include "ieee80211_input.h"

#ifdef ONEBOX_CONFIG_CFG80211
int
ieee80211_ioctl_setparam(struct net_device *dev, 
                         struct iw_request_info *info,
                         void *w, 
                         char *extra);
#if 0
int
ieee80211_ioctl_getparam(struct net_device *dev,
                         struct iw_request_info *info,
                         void *w, 
                         char *extra);
#endif
int cfg80211_ioctl(struct ifnet *ifp, struct ifreq *data, int cmd)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = ifp->priv;
#else
	struct ieee80211vap *vap = netdev_priv(ifp);
#endif
	struct ieee80211com *ic = vap->iv_ic;

	int status = -1;
	struct iw_request_info *info = NULL;
	uint8_t *extra = NULL;
	void *w =NULL;
	struct iwreq *wrq    = (struct iwreq *)data;
	struct iw_point wri;
	struct iw_param rrq;
	char  extra1[50];
	char str[10];

	//memcpy(extra, wrq->u.data.pointer, 4);
	switch (cmd)
	{
		case NL80211_SET_IOCTL:
		{
			//extra[4]  = (char )wrq->u.data.length;
			//extra[4]  = (char )wrq->u.data.length;
			extra = wrq->u.data.pointer;
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("IOCTL: In %s Line %d param = %d value = %d extra[1] = %d\n", __func__, __LINE__, extra[0], extra[4], extra[1]));
			if(!extra[1])
			{
				status = ieee80211_ioctl_setparam(ifp, info, w, extra);
			}
			else
			{
				switch(extra[0])
				{
					case IEEE80211_PARAM_RATE:
						rrq.value = extra[4];
						extra[1] = 0;
						status = ieee80211_ioctl_siwrate(ifp, NULL, &rrq, &extra1[0]);
						if(status == EINVAL)
						{
							status = -1;
						}
					break;
					case IEEE80211_PARAM_MODE:
						//status = ieee80211_ioctl_setparam(ifp, info, w, &extra[0]);
						memcpy(str, &extra[5], extra[1]);
						str[extra[1]] = '\0';
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("str is %s\n", str));
						wri.length = extra[1];
						wri.pointer = &extra[5];
						extra[1] = 0;
						//memcpy(wri.pointer, &extra[4], 7);
						return ieee80211_ioctl_setmode(ifp, NULL, &wri, NULL);
					break;
					case IEEE80211_PARAM_SHPREAMBLE:
						if(extra[1] == 1) /* Indicates auto mode */
						{
							/* Use whatever capability AP supports, indicates auto mode support */	
							vap->hal_priv_vap->preamble = 1;

						}
						else
						{
							/* Indicates use short preamble */
							vap->hal_priv_vap->preamble = 0;
						}
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Preamble: In %s %d preamble=%d\n", __func__, __LINE__, vap->hal_priv_vap->preamble));	
						status = 0;
					break;
					case IEEE80211_PARAM_ROAMING:
						vap->hal_priv_vap->roam_ioctl = extra[4];
						return 0;
					default:
						//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Invalid IOctl\n"));
					break;
		
				}
			}
			break;
		}
		case NL80211_GET_IOCTL:
		{
			//if(extra[0] != IEEE80211_PARAM_MODE)
			extra = wrq->u.data.pointer;
			if(!extra[1])
			{
				status = ieee80211_ioctl_getparam(ifp, info, w, extra);
				memcpy(wrq->u.data.pointer, extra, 2);
				//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("returnning val is %d ptr = %p\n", *(uint8_t *)wrq->u.data.pointer,wrq->u.data.pointer ));
			}
			else
			{
				switch(extra[0])
				{
					case IEEE80211_PARAM_MODE:
						status = ieee80211_ioctl_getmode(ifp, NULL, &wri, &extra1[0]);
						extra[wri.length] = '\0';
						//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Mode is %s\n", extra1));
						memcpy(wrq->u.data.pointer, extra1, wri.length);
					break;
					case IEEE80211_PARAM_RATE:
						rrq.value = extra[4];
						//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("In %s Line %d the rate from user is %d\n", __func__, __LINE__, rrq.value));
						status = ieee80211_ioctl_giwrate(ifp, NULL, &rrq, &extra1[0]);
						extra[0] = rrq.value/500000;
						//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("In %s Line %d rate is %d returning %d\n", __func__, __LINE__, rrq.value, extra[0]));
					break;
					case IEEE80211_PARAM_NAME :
						//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("In %s and %d \n", __func__, __LINE__));
						status = ieee80211_ioctl_giwname(ifp, NULL, extra1, NULL);
						//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("NAME is %s len is %d \n", extra1, strlen(extra1)));
						memcpy(wrq->u.data.pointer, extra1, 20);
						//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("In %s and %d \n", __func__, __LINE__));
					break;
					case IEEE80211_PARAM_BW:
						if(ic->ic_curchan->ic_flags & IEEE80211_CHAN_HT40)
						{
							IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("In %s Line %d connected in 40Mhz \n", __func__, __LINE__));
							extra[0] = 1;
						}
						else
						{
							IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("In %s Line %d connected in 20Mhz \n", __func__, __LINE__));
							extra[0] = 0;
						}
						status = 0;
					break;
					default:
						//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Invalid IOctl\n"));
					break;
				}
			}
			break;
		}
		default:
			//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("INValid IOCTL\n"));
		break;
	}
	return status;
}
#endif

int user_rate_to_rps_rate(int ucastrate)
{

	switch(ucastrate)
	{
		case IEEE80211_RATE_1M:
			return 0;
		case IEEE80211_RATE_2M:
			return 0x02;
		case IEEE80211_RATE_5_5M:
			return 0x04;
		case IEEE80211_RATE_11M:
			return 0x06;
		case IEEE80211_RATE_6M:
			return 0x8b;
		case IEEE80211_RATE_9M:
			return 0x8f;
		case IEEE80211_RATE_12M:
			return 0x8a;
		case IEEE80211_RATE_18M:
			return 0x8e;
		case IEEE80211_RATE_24M:
			return 0x89;
		case IEEE80211_RATE_36M:
			return 0x8d;
		case IEEE80211_RATE_48M:
			return 0x88;
		case IEEE80211_RATE_54M:
			return 0x8c;
		case IEEE80211_RATE_6_5M:
			return 0x100;
		case IEEE80211_RATE_13M:
			return 0x101;
		case IEEE80211_RATE_19_5M:
			return 0x102;
		case IEEE80211_RATE_26M:
			return 0x103;
		case IEEE80211_RATE_39M:
			return 0x104;
		case IEEE80211_RATE_52M:
			return 0x105;
		case IEEE80211_RATE_58_5M:
			return 0x106;
		case IEEE80211_RATE_65M:
			return 0x107;
		default:
			return -1;
			break;
	}

}

uint16_t map_idx_to_rate(uint16_t rate_idx)
{
    switch(rate_idx) 
    {
        case 0x00:
            return IEEE80211_RATE_1M;
        case 0x02:
            return IEEE80211_RATE_2M;
        case 0x04:
            return IEEE80211_RATE_5_5M;
        case 0x06:
            return IEEE80211_RATE_11M;
        case 0x8b:
            return IEEE80211_RATE_6M;
        case 0x8f:
            return IEEE80211_RATE_9M;
        case 0x8a:
            return IEEE80211_RATE_12M;
        case 0x8e:
            return IEEE80211_RATE_18M;
        case 0x89:
            return IEEE80211_RATE_24M;
        case 0x8d:
            return IEEE80211_RATE_36M;
        case 0x88:
            return IEEE80211_RATE_48M;
        case 0x8c:
            return IEEE80211_RATE_54M;
        default:
            printk("Invalid rate index =%x received from FW\n",rate_idx);
            return 0;
    }
}

void set_fixedrate(struct ieee80211vap *vap, int ucastrate)
{
	vap->hal_priv_vap->fixed_rate_enable = 1; 
	switch(ucastrate)
	{
		case IEEE80211_RATE_AUTO:
			vap->hal_priv_vap->rate_hix  = 0;
			vap->hal_priv_vap->fixed_rate_enable = 0;
			break;
		case IEEE80211_RATE_1M:
			vap->hal_priv_vap->rate_hix  = 0;
			break;
		case IEEE80211_RATE_2M:
			vap->hal_priv_vap->rate_hix  = 0x02;
			break;
		case IEEE80211_RATE_5_5M:
			vap->hal_priv_vap->rate_hix  = 0x04;
			break;
		case IEEE80211_RATE_11M:
			vap->hal_priv_vap->rate_hix  = 0x06;
			break;
		case IEEE80211_RATE_6M:
			vap->hal_priv_vap->rate_hix  = 0x8b;
			break;
		case IEEE80211_RATE_9M:
			vap->hal_priv_vap->rate_hix  = 0x8f;
			break;
		case IEEE80211_RATE_12M:
			vap->hal_priv_vap->rate_hix  = 0x8a;
			break;
		case IEEE80211_RATE_18M:
			vap->hal_priv_vap->rate_hix  = 0x8e;
			break;
		case IEEE80211_RATE_24M:
			vap->hal_priv_vap->rate_hix  = 0x89;
			break;
		case IEEE80211_RATE_36M:
			vap->hal_priv_vap->rate_hix = 0x8d;
			break;
		case IEEE80211_RATE_48M:
			vap->hal_priv_vap->rate_hix = 0x88;
			break;
		case IEEE80211_RATE_54M:
			vap->hal_priv_vap->rate_hix = 0x8c;
			break;
		case IEEE80211_RATE_6_5M:
			vap->hal_priv_vap->rate_hix = 0x100;
			break;
		case IEEE80211_RATE_13M:
			vap->hal_priv_vap->rate_hix = 0x101;
			break;
		case IEEE80211_RATE_19_5M:
			vap->hal_priv_vap->rate_hix = 0x102;
			break;
		case IEEE80211_RATE_26M:
			vap->hal_priv_vap->rate_hix = 0x103;
			break;
		case IEEE80211_RATE_39M:
			vap->hal_priv_vap->rate_hix = 0x104;
			break;
		case IEEE80211_RATE_52M:
			vap->hal_priv_vap->rate_hix = 0x105;
			break;
		case IEEE80211_RATE_58_5M:
			vap->hal_priv_vap->rate_hix = 0x106;
			break;
		case IEEE80211_RATE_65M:
			vap->hal_priv_vap->rate_hix = 0x107;
			break;
		default:
			vap->hal_priv_vap->rate_hix = 0;
			vap->hal_priv_vap->fixed_rate_enable = 0;
			break;
	}
}

static int
ieee80211_convert_mode(const char *mode, int type)
{
#define TOUPPER(c) ((((c) > 0x60) && ((c) < 0x7b)) ? ((c) - 0x20) : (c))
	static const struct 
	{
		char *name;
		int mode;
		int ifm_value;
	} mappings[] = {
	            /* NB: need to order longest strings first for overlaps */
	            { "11AST" , IEEE80211_MODE_TURBO_STATIC_A , IFM_IEEE80211_11A|IFM_IEEE80211_TURBO}, 
	            { "AUTO"  , IEEE80211_MODE_AUTO , IFM_AUTO },
	            { "11A"   , IEEE80211_MODE_11A , IFM_IEEE80211_11A },
	            { "11B"   , IEEE80211_MODE_11B , IFM_IEEE80211_11B },
	            { "11G"   , IEEE80211_MODE_11G , IFM_IEEE80211_11G },
	            { "FH"    , IEEE80211_MODE_FH , IFM_IEEE80211_FH },
	            { "0"     , IEEE80211_MODE_AUTO , IFM_AUTO },
	            { "1"     , IEEE80211_MODE_11A , IFM_IEEE80211_11A },
	            { "2"     , IEEE80211_MODE_11B , IFM_IEEE80211_11B },
	            { "3"     , IEEE80211_MODE_11G , IFM_IEEE80211_11G },
	            { "4"     , IEEE80211_MODE_FH , IFM_IEEE80211_FH},
	            { "5"     , IEEE80211_MODE_TURBO_STATIC_A, IFM_IEEE80211_11A|IFM_IEEE80211_TURBO },
	            { "6"     , IEEE80211_MODE_11NG, IFM_IEEE80211_11NG},
	            { "11N"   , IEEE80211_MODE_11NG, IFM_IEEE80211_11NG},
	            { "11AN"   , IEEE80211_MODE_11NA, IFM_IEEE80211_11NA},
	            { NULL }
	    };
	int i, j;
	const char *cp;

	for (i = 0; mappings[i].name != NULL; i++)
	{
		cp = mappings[i].name;
		for (j = 0; j < strlen(mode) + 1; j++)
		{
		/* convert user-specified string to upper case */
			if (TOUPPER(mode[j]) != cp[j])
			{
				break;
			}
			if (cp[j] == '\0')
			{
				if (type == 1)
				{
					return mappings[i].ifm_value;
				}
				else 
				{
					return mappings[i].mode;
				}
			}
		}
	}
	return -1;
#undef TOUPPER
}

#if 0
static int
preempt_scan(struct net_device *dev, int max_grace, int max_wait)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
        struct ieee80211com *ic = vap->iv_ic;
        int total_delay = 0;
        int canceled = 0, ready = 0;

        while (!ready && total_delay < max_grace + max_wait) {
                if ((ic->ic_flags & IEEE80211_F_SCAN) == 0) {
                        ready = 1;
                } else {
                        if (!canceled && (total_delay > max_grace)) {
                                /* Cancel any existing active scan, so that any new parameters
                                 * in this scan ioctl (or the defaults) can be honored, then
                                 * wait around a while to see if the scan cancels properly. */
                                IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
                                                "%s: cancel pending scan request\n", __func__);
                                (void) ieee80211_cancel_scan(vap);
                                canceled = 1;
                        }
                        mdelay (1);
                        total_delay += 1;
                }
        }

        if (!ready) {
                IEEE80211_DPRINTF(vap, IEEE80211_MSG_SCAN,
                                "%s: Timeout canceling current scan.\n",
                                __func__);
        }

        return ready;
}
#endif

static int 
ieee80211_translate_cmd_number(unsigned int param)
{
   /* Fill in body */
	if (param == IEEE80211_PARAM_BEACON_INTERVAL)
	{
		return IEEE80211_IOC_BEACON_INTERVAL;
	}
	else if (param == IEEE80211_PARAM_DTIM_PERIOD)
	{
		return IEEE80211_IOC_DTIM_PERIOD;
	}
	else if (param == IEEE80211_PARAM_CHANGE_COUNTRY_IE)
	{
		return IEEE80211_IOC_CHANGE_COUNTRY_IE;
	}
	else if (param == IEEE80211_PARAM_PUREG)
	{
		return IEEE80211_IOC_PUREG;
	}
	else if (param == IEEE80211_PARAM_PUREN)
	{
		return IEEE80211_IOC_PUREN;
	}
	else if (param == IEEE80211_PARAM_SHORT_GI)
	{
		return IEEE80211_IOC_SHORTGI;
	}
	else if (param == IEEE80211_PARAM_PRIVACY)
	{
		return IEEE80211_IOC_PRIVACY;
	}
	else if (param == IEEE80211_PARAM_HIDESSID)
	{
		return IEEE80211_IOC_HIDESSID;
	}
	else if (param == IEEE80211_PARAM_DBG_ZONE)
	{
		return IEEE80211_IOC_DBG_ZONE;
	}
#ifdef IEEE80211K
	else if( param == IEEE80211_IOCTL_11K_MSRMNT_RPT )
	{
		return IEEE80211_IOC_11K_MSRMNT_REQ;
	}
#endif	
	else
	{
		return 0;
	}
}

#ifdef ONEBOX_CONFIG_CFG80211
int
#else
static int
#endif
ieee80211_ioctl_giwname(struct net_device *dev, 
                        struct iw_request_info *info,
                        char *name, 
                        char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211_channel *c = vap->iv_ic->ic_curchan;

	if (IEEE80211_IS_CHAN_108G(c))
	{
		strncpy(name, "IEEE 802.11Tg", IFNAMSIZ);
	}
	else if (IEEE80211_IS_CHAN_108A(c))
	{
		strncpy(name, "IEEE 802.11Ta", IFNAMSIZ);
	}
	else if (IEEE80211_IS_CHAN_TURBO(c))
	{
		strncpy(name, "IEEE 802.11T", IFNAMSIZ);
	}
	else if (IEEE80211_IS_CHAN_HT(c) && IEEE80211_IS_CHAN_A(c))
	{
		strncpy(name, "IEEE 802.11abgn", IFNAMSIZ);
	}
	else if (IEEE80211_IS_CHAN_HT(c))
	{
		strncpy(name, "IEEE 802.11bgn", IFNAMSIZ);
	}
	else if (IEEE80211_IS_CHAN_ANYG(c))
	{
		strncpy(name, "IEEE 802.11g", IFNAMSIZ);
	}
	else if (IEEE80211_IS_CHAN_A(c))
	{
		strncpy(name, "IEEE 802.11a", IFNAMSIZ);
	}
	else if (IEEE80211_IS_CHAN_B(c))
	{
		strncpy(name, "IEEE 802.11b", IFNAMSIZ);
	}
	else
	{
		strncpy(name, "IEEE 802.11", IFNAMSIZ);
	}
	/* XXX FHSS */
	return 0;
}

static int
ieee80211_ioctl_setconfig(struct net_device *dev,
			  struct iw_request_info *info,
			  void *w,
			  char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	int error;

	/* cfg80211 strips 16 bytes of name(i_name)
	 * and passes the pointer to driver
	 */
	struct ieee80211req *ireq = (struct ieee80211req *)
					(extra - sizeof(ireq->i_name));
	error = ieee80211_ioctl_set80211(vap, info->cmd, ireq);

	return error;
}

static int
ieee80211_ioctl_getconfig(struct net_device *dev,
			  struct iw_request_info *info,
			  void *w, char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	int error;

	/* cfg80211 strips 16 bytes of name(i_name)
	 * and passes the pointer to driver
	 */
	struct ieee80211req *ireq = (struct ieee80211req *)
					(extra - sizeof(ireq->i_name));
	error = ieee80211_ioctl_get80211(vap, 0, ireq);

	return error;
}

static int
ieee80211_ioctl_setifmedia(struct net_device *dev,
			   struct iw_request_info *info,
			   void *w,
			   char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	int error;

	/* cfg80211 strips 16 bytes of name(ifr_name)
	 * and passes the pointer to driver
	 */
	struct ifreq *ifr = (struct ifreq *)
				(extra - sizeof(ifr->ifr_name));
	error = ieee80211_ioctl(vap->iv_ifp, ifr, info->cmd);

	return error;
}

static int
ieee80211_ioctl_ifmedia(struct net_device *dev,
			struct iw_request_info *info,
			void *w,
			char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18, 22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	int error;

	/* cfg80211 strips 16 bytes of name(ifr_name)
	 * and passes the pointer to driver
	 */
	struct ifreq *ifr = (struct ifreq *)
				(extra - sizeof(ifr->ifr_name));
	error = ieee80211_ioctl(vap->iv_ifp, ifr, info->cmd);

	return error;
}

static int
ieee80211_ioctl_siwfreq(struct net_device *dev, 
                        struct iw_request_info *info,
                        struct iw_freq *freq, 
                        char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
	unsigned int frequency = 0, mode_11j = 0;

  
	if (freq->e > 1)
	{
		return -EINVAL;
	}
	if (freq->e == 1)
	{
		ireq.i_val = ieee80211_mhz2ieee(freq->m/100000, 0); /* Divide by 10000?  take care of flags - */
		frequency = freq->m/100000;
		if( frequency >= 4920 && frequency <= 5080 )
				mode_11j = 1;
		else 
				mode_11j = 0;
	}
	else
	{
		ireq.i_val = freq->m;
	}
	ireq.i_data = &mode_11j;
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Freq is %d %d val is %d\n", freq->m, freq->e, ireq.i_val));
	ireq.i_type = IEEE80211_IOC_CHANNEL;
	return ieee80211_ioctl_set80211(vap, 0, &ireq);
}

static int
ieee80211_ioctl_giwfreq(struct net_device *dev, 
                        struct iw_request_info *info,
                        struct iw_freq *freq, 
                        char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211_channel *c = vap->iv_ic->ic_curchan;
	struct ieee80211req ireq;
	int ret = 0;

	ireq.i_type = IEEE80211_IOC_CHANNEL;
	ret = ieee80211_ioctl_get80211(vap, 0, &ireq);
	if (!ret) {
		freq->m = ieee80211_ieee2mhz(ireq.i_val, c->ic_flags) * 100000;
		freq->e = 1;
		return 0;
	}

	return ret;
}

static int
ieee80211_ioctl_siwmode(struct net_device *dev,
                        struct iw_request_info *info,
                        __u32 *mode, 
                        char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	int valid = 0;

	if (vap->iv_opmode == IEEE80211_M_HOSTAP)
	{
		valid = (*mode == IW_MODE_MASTER);
	}
	else if (vap->iv_opmode == IEEE80211_M_MONITOR)
	{
		valid = (*mode == IW_MODE_MONITOR);
	}
	else if (vap->iv_opmode == IEEE80211_M_IBSS)
	{
		valid = (*mode == IW_MODE_ADHOC);
	}
	else if (vap->iv_opmode == IEEE80211_M_WDS)
	{
		valid = (*mode == IW_MODE_REPEAT);
	}
	else
	{
		valid = (*mode == IW_MODE_INFRA);
	}
	return valid ? 0 : -EINVAL;
}

static int
ieee80211_ioctl_giwmode(struct net_device *dev,
                        struct iw_request_info *info,
                        __u32 *mode, 
                        char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ifmediareq imr;

	vap->iv_media.ifm_status(vap->iv_ifp, &imr);

	if (vap->iv_opmode == IEEE80211_M_HOSTAP)
	{
		*mode = IW_MODE_MASTER;
	}
	else if (vap->iv_opmode == IEEE80211_M_MONITOR)
	{
		*mode = IW_MODE_MONITOR;
	}
	else if (vap->iv_opmode == IEEE80211_M_IBSS)
	{
		*mode = IW_MODE_ADHOC;
	}
	else if (vap->iv_opmode == IEEE80211_M_WDS)
	{
		*mode = IW_MODE_REPEAT;
	}
	else
	{
		*mode = IW_MODE_INFRA;
	}
	return 0;
}

static int
ieee80211_ioctl_siwsens(struct net_device *dev,
                        struct iw_request_info *info,
                        struct iw_param *sens, 
                        char *extra)
{
	return -EOPNOTSUPP;
}
static int
ieee80211_ioctl_giwsens(struct net_device *dev,
                        struct iw_request_info *info,
                        struct iw_param *sens, 
                        char *extra)
{
	sens->value = 1;
	sens->fixed = 1;

	return 0;
}

static int
ieee80211_ioctl_siwap(struct net_device *dev, 
                      struct iw_request_info *info,
                      struct sockaddr *ap_addr, 
                      char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;

	ireq.i_len  = IEEE80211_ADDR_LEN;
	ireq.i_data = &ap_addr->sa_data;
	ireq.i_type = IEEE80211_IOC_BSSID;
	return ieee80211_ioctl_set80211(vap, 0, &ireq);
}

static int
ieee80211_ioctl_giwap(struct net_device *dev, 
                      struct iw_request_info *info,
                      struct sockaddr *ap_addr, 
                      char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;

	ireq.i_len  = IEEE80211_ADDR_LEN;
	ireq.i_data = &ap_addr->sa_data;
	ireq.i_type = IEEE80211_IOC_BSSID;
	ap_addr->sa_family = ARPHRD_ETHER;

	return ieee80211_ioctl_get80211(vap, 0, &ireq);
}

#define ONEBOX_STATUS_SUCCESS 0
#define ONEBOX_STATUS_FAILURE -1


static int
ieee80211_ioctl_siwscan(struct net_device *dev,
                        struct iw_request_info *info,
                        struct iw_point *data,
                        char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
	struct ieee80211_scan_req *sr = NULL;
	struct iw_scan_req *req = (struct iw_scan_req *)extra;
	struct ifreq ifr;
  	int status = 0;

  
  	if((vap->iv_opmode != IEEE80211_M_STA) || (vap->iv_ic->ic_flags & IEEE80211_F_SCAN))
    		return -1;
  	sr = kmalloc(sizeof(struct ieee80211_scan_req), GFP_KERNEL);
  	if(sr == NULL) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Failed to Allocate Memory\n"));
		return -1;
  	}
	memset(sr, 0, sizeof(struct ieee80211_scan_req));

	sr->sr_flags = IEEE80211_IOC_SCAN_ACTIVE | IEEE80211_IOC_SCAN_ONCE | IEEE80211_IOC_SCAN_NOJOIN|IEEE80211_IOC_SCAN_CHECK;
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, ("In %s Line %d flags %02x\n", __func__, __LINE__, sr->sr_flags));

	//sr->num_freqs = 0;
	//sr->sr_nssid = 0;
  sr->sr_duration = IEEE80211_IOC_SCAN_FOREVER;
  //sr->sr_mindwell = 0;
  //sr->sr_maxdwell = 0;
	
  if (req)
	{
		if (data->flags & IW_SCAN_THIS_ESSID) 
		{
			sr->sr_ssid[0].len = req->essid_len;  
			memcpy(sr->sr_ssid[0].ssid, req->essid, sr->sr_ssid[0].len);
      sr->sr_flags |= IEEE80211_IOC_SCAN_CHECK;
		}

	}

	memset(&ifr, 0, sizeof(ifr));

	if(ieee80211_ioctl(vap->iv_ifp, &ifr, SIOCGIFFLAGS) < 0 ) {
    	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("In %s Line %d flags %02x\n", __func__, __LINE__, sr->sr_flags));
	 	kfree(sr);
    	return -1;  
  	}

  if (!(ifr.ifr_flags & IFF_UP))
  {
    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, ("Bringing UP the interface*******\n"));
    ifr.ifr_flags |= IFF_UP;
    vap->iv_ifp->if_flags|= IFF_UP;
  }

	ireq.i_type = IEEE80211_IOC_SCAN_REQ;
	ireq.i_data = sr;
  ireq.i_val = 0 ;
	ireq.i_len  = sizeof(struct ieee80211_scan_req);
  IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_DEBUG, ("In %s Line %d flags %02x\n", __func__, __LINE__, sr->sr_flags));

	status = ieee80211_ioctl_set80211(vap, 0, &ireq);
  kfree(sr);
  return status;

}

static int
ieee80211_ioctl_giwscan(struct net_device *dev,
                        struct iw_request_info *info,
                        struct iw_point *data, 
                        char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
	struct ieee80211req_scan_result *se;
	struct iw_event iwe;

	uint8_t *frm;
	uint8_t	*efrm;
	uint8_t *end_se;
	uint8_t *scan_res;
	uint8_t *ie;
	uint8_t *ssid;
	uint8_t ii;

	uint8_t  *current_ev = extra;
	uint16_t extra_length = data->length;
	uint8_t  *current_val = NULL;

	ireq.i_type = IEEE80211_IOC_SCAN_RESULTS;
	scan_res = kmalloc(24*1024, GFP_KERNEL);
	ireq.i_data = scan_res;
	ireq.i_len = 24*1024;
	
	//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Len is %d\n", ireq.i_len));

	if (ieee80211_ioctl_get80211(vap, 0, &ireq))
	{
		kfree(scan_res);
		return -EAGAIN;
	}
	//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("i_Len is %d\n", ireq.i_len));
	se = (struct ieee80211req_scan_result *)scan_res;
	end_se = (uint8_t *)se + ireq.i_len;

	while((end_se != (uint8_t *)se))
	{

		iwe.cmd = SIOCGIWAP;
		iwe.u.ap_addr.sa_family = ARPHRD_ETHER;
		memcpy(iwe.u.ap_addr.sa_data, se->isr_bssid,  6);

		//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("MAC addr is %02x %02x %02x %02x %02x %02x \n", se->isr_bssid[0], se->isr_bssid[1], se->isr_bssid[2], se->isr_bssid[3], se->isr_bssid[4], se->isr_bssid[5]));

#if KERNEL_VERSION_BTWN_2_6_(18, 26)
		current_ev = iwe_stream_add_event(current_ev,
				extra + extra_length,
				&iwe,
				&se->isr_bssid[IEEE80211_ADDR_LEN]);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
		current_ev = iwe_stream_add_event(info,
				current_ev,
				extra + extra_length,
				&iwe,
				IW_EV_ADDR_LEN);
#endif

		iwe.cmd = SIOCGIWESSID;
		iwe.u.data.length =  se->isr_ssid_len;

		if (iwe.u.data.length > 32)
		{
			iwe.u.data.length = 32;
		}
		iwe.u.data.flags = 1;
		//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("ssid length is %d\n", se->isr_ssid_len));

		ssid = ((uint8_t *)se + sizeof(struct ieee80211req_scan_result)); 

#if	KERNEL_VERSION_BTWN_2_6_(18, 26)
		current_ev = iwe_stream_add_point(current_ev,
				extra + extra_length,
				&iwe,
				ssid);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
		current_ev = iwe_stream_add_point(info,
				current_ev,
				extra + extra_length,
				&iwe,
				ssid);
#endif

		iwe.cmd = SIOCGIWMODE;
		if ( se->isr_capinfo & 0x0002)
		{
			iwe.u.mode = IW_MODE_ADHOC;
		}
		else
		{
			iwe.u.mode = IW_MODE_MASTER;
		}		/* End if <condition> */

#if KERNEL_VERSION_BTWN_2_6_(18, 26)
		current_ev = iwe_stream_add_event(current_ev,
				extra + extra_length,
				&iwe,
				IW_EV_UINT_LEN);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
		current_ev = iwe_stream_add_event(info,
				current_ev,
				extra + extra_length,
				&iwe,
				IW_EV_UINT_LEN);
#endif
		iwe.cmd = SIOCGIWFREQ;
		iwe.u.freq.e = 1;
		iwe.u.freq.m =(se->isr_freq * 100000);

#if KERNEL_VERSION_BTWN_2_6_(18, 26)
		current_ev = iwe_stream_add_event(current_ev,
				extra + extra_length,
				&iwe,
				IW_EV_FREQ_LEN);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
		current_ev = iwe_stream_add_event(info,
				current_ev,
				extra + extra_length,
				&iwe,
				IW_EV_FREQ_LEN);

#endif
		iwe.cmd = IWEVQUAL;
		iwe.u.qual.level   = - se->isr_rssi;
		iwe.u.qual.qual    = (100 - (se->isr_rssi));
		iwe.u.qual.updated = IW_QUAL_LEVEL_UPDATED | IW_QUAL_DBM;
		iwe.u.qual.noise   = - se->isr_noise;

#if KERNEL_VERSION_BTWN_2_6_(18, 26)
		current_ev = iwe_stream_add_event(current_ev,
				extra + extra_length,
				&iwe,
				IW_EV_QUAL_LEN);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
		current_ev = iwe_stream_add_event(info,
				current_ev,
				extra + extra_length,
				&iwe,
				IW_EV_QUAL_LEN);
#endif

		iwe.cmd = SIOCGIWENCODE;
		if ( se->isr_capinfo  & 0x0010)
		{
			iwe.u.data.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
		}
		else
		{
			iwe.u.data.flags = IW_ENCODE_DISABLED;
		} 	

		iwe.u.data.length = 0;

#if KERNEL_VERSION_BTWN_2_6_(18, 26)
		current_ev = iwe_stream_add_point(current_ev,
				extra + extra_length,
				&iwe,
				NULL);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
		current_ev = iwe_stream_add_point(info,current_ev,
				extra + extra_length,
				&iwe,
				NULL);
#endif
		iwe.cmd = SIOCGIWRATE;
		current_val = current_ev + IW_EV_LCP_LEN;
		iwe.u.bitrate.fixed = iwe.u.bitrate.disabled = 0;
		for(ii=0; ii< se->isr_nrates; ii++)
		{
			if(se->isr_rates[ii] == 0 )		
				break;

			iwe.u.bitrate.value = ((se->isr_rates[ii] & 0x7f)*500000) ;

			current_val = iwe_stream_add_value(info,
					current_ev,current_val,
					extra + extra_length,
					&iwe,
					IW_EV_PARAM_LEN);
		}

		if((current_val -current_ev) > IW_EV_LCP_LEN)
			current_ev = current_val;


		ie = (uint8_t *)((uint8_t *)se + (sizeof(struct ieee80211req_scan_result)) + (se->isr_ssid_len) + (se->isr_meshid_len));

		frm = ie;
		efrm = ie + se->isr_ie_len; 

		if ( se->isr_capinfo  & 0x0010)
		{
			while (efrm != frm ) {

				switch (*frm) {
					case IEEE80211_ELEMID_RSN:
						iwe.u.data.length = frm[1] + 2;
						iwe.cmd = IWEVGENIE;
#if KERNEL_VERSION_BTWN_2_6_(18, 26)
						current_ev = iwe_stream_add_point(info,current_ev,
								extra + extra_length,
								&iwe,
								frm);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
						current_ev = iwe_stream_add_point(info,current_ev,
								extra + extra_length,
								&iwe,
								frm);
#endif
						break;
					case IEEE80211_ELEMID_VENDOR:
						if (iswpaoui(frm)){
							iwe.u.data.length = frm[1] + 2;
							iwe.cmd = IWEVGENIE;
#if KERNEL_VERSION_BTWN_2_6_(18, 26)
							current_ev = iwe_stream_add_point(info,current_ev,
									extra + extra_length,
									&iwe,
									frm);
#elif KERNEL_VERSION_GREATER_THAN_2_6_(26)
							current_ev = iwe_stream_add_point(info,current_ev,
									extra + extra_length,
									&iwe,
									frm);
#endif
						}
						break;

					default:
						break;
				}/* End of switch case */
				frm += frm[1] + 2;
			}/* End of inner while loop */
		}/* End of if */
		se = (struct ieee80211req_scan_result *)(((uint8_t *)se) + se->isr_len);

	}/* End of outer while loop */

	data->length = ireq.i_len;
	//IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Length is %d\n", data->length));
	data->flags = 0;
	kfree(scan_res);
	return 0;	
}
	
static int
ieee80211_ioctl_siwessid(struct net_device *dev, 
                         struct iw_request_info *info,
                         struct iw_point *data, 
                         char *ssid)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;

	if (data->flags == 0) 
	{
		ireq.i_val = 0;
		ireq.i_len = 0;
	}
	else 
	{
		if (data->length > IEEE80211_NWID_LEN)
		{
			data->length = IEEE80211_NWID_LEN;
		}
		ireq.i_val = 0;
		ireq.i_len  = data->length;
		ireq.i_data = ssid;

#if WIRELESS_EXT < 21
		/*
		 * Deduct a trailing \0 since iwconfig passes a string
		 * length that includes this.  Unfortunately this means
		 * that specifying a string with multiple trailing \0's
		 * won't be handled correctly.  Not sure there's a good
		 * solution; the API is botched (the length should be
		 * exactly those bytes that are meaningful and not include
		 * extraneous stuff).
		 */
		/* The API was fixed in WE21 */
		if (data->length > 0 && ssid[data->length - 1] == '\0')
		{
			--ireq.i_len;
		}
#endif /* WIRELESS_EXT < 21 */
	}

	ireq.i_type = IEEE80211_IOC_SSID;
	return  ieee80211_ioctl_set80211(vap, 0, &ireq);
}

static int
ieee80211_ioctl_giwessid(struct net_device *dev,
                         struct iw_request_info *info,
                         struct iw_point *data, 
                         char *essid)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;

	/* Just set the pointer */
	ireq.i_data = essid;

	ireq.i_type = IEEE80211_IOC_SSID;

	if (ieee80211_ioctl_get80211(vap, 0, &ireq))
		return -EINVAL;

	data->length = strlen(essid);
	data->flags = 1;
	return 0;
}

static int
ieee80211_ioctl_siwnickn(struct net_device *dev, 
                         struct iw_request_info *info,
                         struct iw_point *data,
                         char *nickname)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif

	if (data->length > IEEE80211_NWID_LEN)
	{
		return -E2BIG;
	}

	memset(vap->iv_nickname, 0, IEEE80211_NWID_LEN);
	memcpy(vap->iv_nickname, nickname, data->length);
	vap->iv_nicknamelen = data->length;

	return 0;
}

static int
ieee80211_ioctl_giwnickn(struct net_device *dev, 
                         struct iw_request_info *info,
                         struct iw_point *data, 
                         char *nickname)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif

	if (data->length > vap->iv_nicknamelen + 1)
	{
		data->length = vap->iv_nicknamelen + 1;
	}
	if (data->length > 0)
	{
		memcpy(nickname, vap->iv_nickname, data->length - 1); /* XXX: strcpy? */
		nickname[data->length-1] = '\0';
	}
	return 0;
}

static int
ieee80211_ioctl_siwrts(struct net_device *dev,
                       struct iw_request_info *info,
                       struct iw_param *rts,
                       char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;

	if (rts->disabled)
	{
		ireq.i_val = IEEE80211_RTS_MAX;
	}
	else if ((IEEE80211_RTS_MIN <= rts->value) &&
	         (rts->value <= IEEE80211_RTS_MAX))
	{
		ireq.i_val = rts->value;
	}
	else
	{
		return -EINVAL;
	}

	ireq.i_type = IEEE80211_IOC_RTSTHRESHOLD;
	return ieee80211_ioctl_set80211(vap, 0, &ireq);
}

static int
ieee80211_ioctl_giwrts(struct net_device *dev, 
                       struct iw_request_info *info,
                       struct iw_param *rts, 
                       char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;

	ireq.i_type = IEEE80211_IOC_RTSTHRESHOLD;

	if (ieee80211_ioctl_get80211(vap, 0, &ireq))
	{
		return -EINVAL;
	}

	/* Value retuned in i_val */
	rts->value = ireq.i_val;
	rts->disabled = (rts->value == IEEE80211_RTS_MAX);
	rts->fixed = 1;

	return 0;
}
static int
ieee80211_ioctl_siwmlme(struct net_device *dev,
                        struct iw_request_info *info, 
                        struct iw_point *erq, 
                        char *data)
{
	return 0;
}
static int
ieee80211_ioctl_siwfrag(struct net_device *dev,
                        struct iw_request_info *info,
                        struct iw_param *frag, 
                        char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
	if (frag->disabled)
	{
		ireq.i_val = 2346;
	}
	else if ((frag->value < 256) || (frag->value > 2346))
	{
		return -EINVAL;
	}
	else
	{
		ireq.i_val = (frag->value & ~0x1);
	}

	ireq.i_type = IEEE80211_IOC_FRAGTHRESHOLD;
	return ieee80211_ioctl_set80211(vap, 0, &ireq);
}

static int
ieee80211_ioctl_giwfrag(struct net_device *dev,
                        struct iw_request_info *info,
                        struct iw_param *frag, 
                        char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
	ireq.i_type = IEEE80211_IOC_FRAGTHRESHOLD;

	if (ieee80211_ioctl_get80211(vap, 0, &ireq))
	{
		return -EINVAL;
	}

	/* Value retuned in i_val */
	frag->value = ireq.i_val;
	frag->disabled = (frag->value == 2346);
	frag->fixed = 1;

	return 0;
}

/*This function sets the transmit power to the devcie
 */
static int
ieee80211_ioctl_siwtxpow(struct net_device *dev, 
                         struct iw_request_info *info,
                         struct iw_param *rrq, 
                         char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
  
	ireq.i_val = (unsigned short int)rrq->value;
	ireq.i_type = IEEE80211_IOC_TXPOWER;
	if(ieee80211_ioctl_set80211(vap, 0, &ireq))
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Unable to issue Transmit power ioctl\n"));
		return -1;
	}
	return 0;
}

static int
ieee80211_ioctl_siwretry(struct net_device *dev, 
                         struct iw_request_info *info,
                         struct iw_param *rrq, 
                         char *extra)
{
#if 0
	struct ieee80211vap *vap = dev->priv;
	struct ieee80211com *ic = vap->iv_ic;

	if (rrq->disabled) {
		if (vap->iv_flags & IEEE80211_F_SWRETRY) {
			vap->iv_flags &= ~IEEE80211_F_SWRETRY;
			goto done;
		}
		return 0;
	}

	if ((vap->iv_caps & IEEE80211_C_SWRETRY) == 0)
		return -EOPNOTSUPP;
	if (rrq->flags == IW_RETRY_LIMIT) {
		if (rrq->value >= 0) {
			vap->iv_txmin = rrq->value;
			vap->iv_txmax = rrq->value;	/* XXX */
			vap->iv_txlifetime = 0;		/* XXX */
			vap->iv_flags |= IEEE80211_F_SWRETRY;
		} else {
			vap->iv_flags &= ~IEEE80211_F_SWRETRY;
		}
		return 0;
	}
done:
	return IS_UP(vap->iv_dev) ? ic->ic_reset(vap->iv_dev) : 0;
#endif
	return 0;
}

static int
ieee80211_ioctl_giwretry(struct net_device *dev, 
                         struct iw_request_info *info,
                         struct iw_param *rrq, 
                         char *extra)
{
#if 0
	struct ieee80211vap *vap = dev->priv;

	rrq->disabled = (vap->iv_flags & IEEE80211_F_SWRETRY) == 0;
	if (!rrq->disabled) {
		switch (rrq->flags & IW_RETRY_TYPE) {
		case IW_RETRY_LIFETIME:
			rrq->flags = IW_RETRY_LIFETIME;
			rrq->value = IEEE80211_TU_TO_MS(vap->iv_txlifetime);
			break;
		case IW_RETRY_LIMIT:
			rrq->flags = IW_RETRY_LIMIT;
			switch (rrq->flags & IW_RETRY_MODIFIER) {
			case IW_RETRY_MIN:
				rrq->flags |= IW_RETRY_MAX;
				rrq->value = vap->iv_txmin;
				break;
			case IW_RETRY_MAX:
				rrq->flags |= IW_RETRY_MAX;
				rrq->value = vap->iv_txmax;
				break;
			}
			break;
		}
	}
	return 0;
#endif
	return -1;
}

//#define	IEEE80211_BINTVAL_MAX		5000	/* max beacon interval (TUs) */
//#define	IEEE80211_BINTVAL_MIN		10	/* min beacon interval (TUs) */
//#define	IEEE80211_BINTVAL_DEFAULT 	100	/* default beacon interval (TUs) */
                                  /* RSI CHANGES */
//#define	IEEE80211_BINTVAL_DEFAULT 	200	/* default beacon interval (TUs) */
#define IEEE80211_BINTVAL_VALID(_bi) \
	((IEEE80211_BINTVAL_MIN <= (_bi)) && \
	 ((_bi) <= IEEE80211_BINTVAL_MAX))
#define IEEE80211_BINTVAL_SANITISE(_bi) \
	(IEEE80211_BINTVAL_VALID(_bi) ? \
	 (_bi) : IEEE80211_BINTVAL_DEFAULT)
#define IEEE80211_TU_TO_MS(x)	(((x) * 1024) / 1000)

static int
ieee80211_ioctl_siwpower(struct net_device *dev,
                         struct iw_request_info *info,
                         struct iw_param *wrq,
                         char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
	/*
	 * if ((ic->ic_caps & IEEE80211_C_PMGT) == 0)
	 *	return -EOPNOTSUPP;
	 */

	if (wrq->disabled) 
	{
		ireq.i_val = IEEE80211_POWERSAVE_OFF;
	}
	else 
	{
		ireq.i_val = IEEE80211_POWERSAVE_ON;
	}

	ireq.i_type = IEEE80211_IOC_POWERSAVE;
	if (ieee80211_ioctl_set80211(vap, 0, &ireq))
	{
		return -EINVAL;
	}
	else 
	{
		if (IEEE80211_BINTVAL_VALID(wrq->value)) 
		{
			ireq.i_val = wrq->value;
		}
		else
		{
			return -EINVAL;
		}
		
		ireq.i_type = IEEE80211_IOC_POWERSAVESLEEP;
		return ieee80211_ioctl_set80211(vap, 0, &ireq);
	}
}

static int
ieee80211_ioctl_giwpower(struct net_device *dev, 
                         struct iw_request_info *info,
                         struct iw_param *rrq, 
                         char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
	ireq.i_type = IEEE80211_IOC_POWERSAVE;
	if (ieee80211_ioctl_get80211(vap, 0, &ireq))
	{
		return -EINVAL;
	}
	if (ireq.i_val == IEEE80211_POWERSAVE_OFF) 
	{
		rrq->disabled = 1;  
	}
	else 
	{
		rrq->disabled = 0;  
		rrq->flags |= IW_POWER_ALL_R; 
		/* Get the listen interval value */
		ireq.i_type = IEEE80211_IOC_POWERSAVESLEEP;
		if (ieee80211_ioctl_get80211(vap, 0, &ireq))
		{
			return -EINVAL;
		}
		rrq->value = IEEE80211_TU_TO_MS(ireq.i_val);
	}
	return 0;
}
static int
ieee80211_ioctl_giwtxpow(struct net_device *dev,
                         struct iw_request_info *info,
                         struct iw_param *rrq, 
                         char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
	
	ireq.i_type = IEEE80211_IOC_TXPOWER;

	if (ieee80211_ioctl_get80211(vap, 0, &ireq))
	{
		return -EINVAL;
	}


  rrq->value = ireq.i_val;
  rrq->fixed  = 1;
  rrq->flags  = IW_TXPOW_DBM;
  rrq->disabled = 0;

  return 0;
}

static int
ieee80211_ioctl_giwrange(struct net_device *dev,
                        struct iw_request_info *info,
                        struct iw_param *rrq, 
                        char *extra)
{
	struct iw_range *range = (struct iw_range *) extra;
	range->max_qual.qual  = 80;
	//rrq->disabled = 1;
	/* Set the Wireless Extension versions */
	range->we_version_compiled = WIRELESS_EXT;
	return 0;
}
/* 
 * If this function is invoked it means someone is using the wireless extensions
 * API instead of the private madwifi ioctls.  That's fine.  We translate their
 * request into the format used by the private ioctls.  Note that the 
 * iw_request_info and iw_param structures are not the same ones as the 
 * private ioctl handler expects.  Luckily, the private ioctl handler doesn't
 * do anything with those at the moment.  We pass NULL for those, because in 
 * case someone does modify the ioctl handler to use those values, a null 
 * pointer will be easier to debug than other bad behavior.
 */
static int
ieee80211_ioctl_siwauth(struct net_device *dev,
                        struct iw_request_info *info,
                        struct iw_param *erq,
                        char *buf)
{
	int rc = -EOPNOTSUPP;

#if 0
	switch (erq->flags & IW_AUTH_INDEX) {
	case IW_AUTH_WPA_VERSION:
		rc = siwauth_wpa_version(dev, info, erq, buf);
		break;
	case IW_AUTH_CIPHER_PAIRWISE:
		rc = siwauth_cipher_pairwise(dev, info, erq, buf);
		break;
	case IW_AUTH_CIPHER_GROUP:
		rc = siwauth_cipher_group(dev, info, erq, buf);
		break;
	case IW_AUTH_KEY_MGMT:
		rc = siwauth_key_mgmt(dev, info, erq, buf);
		break;
	case IW_AUTH_TKIP_COUNTERMEASURES:
		rc = siwauth_tkip_countermeasures(dev, info, erq, buf);
		break;
	    
	case IW_AUTH_DROP_UNENCRYPTED:
		rc = siwauth_drop_unencrypted(dev, info, erq, buf);
		break;
	case IW_AUTH_80211_AUTH_ALG:
		rc = siwauth_80211_auth_alg(dev, info, erq, buf);
		break;
	case IW_AUTH_WPA_ENABLED:
		rc = siwauth_wpa_enabled(dev, info, erq, buf);
		break;
	case IW_AUTH_RX_UNENCRYPTED_EAPOL:
		rc = siwauth_rx_unencrypted_eapol(dev, info, erq, buf);
		break;
	case IW_AUTH_ROAMING_CONTROL:
		rc = siwauth_roaming_control(dev, info, erq, buf);
		break;
	case IW_AUTH_PRIVACY_INVOKED:
		rc = siwauth_privacy_invoked(dev, info, erq, buf);
		break;
	default:
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, (KERN_WARNING "%s: unknown SIOCSIWAUTH flag %d\n",
			dev->name, erq->flags));
		break;
	}

#endif
	return rc;
}

int
ieee80211_ioctl_getparam(struct net_device *dev,
                         struct iw_request_info *info,
                         void *w, 
                         char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
	unsigned int *param = (unsigned int *)extra;

	*param = ieee80211_translate_cmd_number(param[0]);
	ireq.i_type = *param;
	if (ieee80211_ioctl_get80211(vap, 0, &ireq)) 
	{
		return -EINVAL;
	}
	else 
	{
		
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("NL80211 : ireq.i_val %d \n", ireq.i_val));
		param[0] = ireq.i_val;
	}
	return 0;
}

static int
giwauth_wpa_version(struct net_device *dev,
                    struct iw_request_info *info, 
                    struct iw_param *erq, 
                    char *buf)
{
	int ver;
	int rc;
	int arg = IEEE80211_PARAM_WPA;

	rc = ieee80211_ioctl_getparam(dev, NULL, NULL, (char*)&arg);
	if (rc)
	{
		return rc;
	}

	switch (arg) 
	{
		case 1:
			ver = IW_AUTH_WPA_VERSION_WPA;
			break;
		case 2:
			ver = IW_AUTH_WPA_VERSION_WPA2;
			break;
		case 3:
			ver = IW_AUTH_WPA_VERSION|IW_AUTH_WPA_VERSION_WPA2;
			break;
		default:
			ver = IW_AUTH_WPA_VERSION_DISABLED;
			break;
	}

	erq->value = ver;
	return rc;
}

static int
ieee80211cipher2iwcipher(int ieee80211ciph)
{
	return 0;
}

static int
giwauth_cipher_pairwise(struct net_device *dev,
                        struct iw_request_info *info, 
                        struct iw_param *erq, 
                        char *buf)
{
	int rc;
	int arg = IEEE80211_PARAM_UCASTCIPHER;

	rc = ieee80211_ioctl_getparam(dev, NULL, NULL, (char*)&arg);
	if (rc)
	{
		return rc;
	}

	erq->value = ieee80211cipher2iwcipher(arg);
	if (erq->value < 0)
	{
		return -EINVAL;
	}
	return 0;
}


static int
giwauth_cipher_group(struct net_device *dev,
                     struct iw_request_info *info, 
                     struct iw_param *erq,
                     char *buf)
{
	int rc;
	int arg = IEEE80211_PARAM_MCASTCIPHER;

	rc = ieee80211_ioctl_getparam(dev, NULL, NULL, (char*)&arg);
	if (rc)
	{
		return rc;
	}

	erq->value = ieee80211cipher2iwcipher(arg);
	if (erq->value < 0)
	{
		return -EINVAL;
	}
	return 0;
}
static int
giwauth_key_mgmt(struct net_device *dev,
                 struct iw_request_info *info, 
                 struct iw_param *erq,
                 char *buf)
{
	int arg;
	int rc;

	arg = IEEE80211_PARAM_KEYMGTALGS;
	rc = ieee80211_ioctl_getparam(dev, NULL, NULL, (char*)&arg);
	if (rc)
	{
		return rc;
	}
	erq->value = 0;
	if (arg & WPA_ASE_8021X_UNSPEC)
	{
		erq->value |= IW_AUTH_KEY_MGMT_802_1X;
	}
	if (arg & WPA_ASE_8021X_PSK)
	{
		erq->value |= IW_AUTH_KEY_MGMT_PSK;
	}
	return 0;
}

static int
giwauth_tkip_countermeasures(struct net_device *dev,
                             struct iw_request_info *info, 
                             struct iw_param *erq, 
                             char *buf)
{
	int arg;
	int rc;

	arg = IEEE80211_PARAM_COUNTERMEASURES;
	rc = ieee80211_ioctl_getparam(dev, NULL, NULL, (char*)&arg);
	if (rc)
	{
		return rc;
	}
	erq->value = arg;
	return 0;
}

static int
giwauth_drop_unencrypted(struct net_device *dev,
                         struct iw_request_info *info,
                         struct iw_param *erq,
                         char *buf)
{
	int arg;
	int rc;
	arg = IEEE80211_PARAM_DROPUNENCRYPTED;
	rc = ieee80211_ioctl_getparam(dev, NULL, NULL, (char*)&arg);
	if (rc)
	{
		return rc;
	}
	erq->value = arg;
	return 0;
}

static int
giwauth_80211_auth_alg(struct net_device *dev,
                       struct iw_request_info *info, 
                       struct iw_param *erq, 
                       char *buf)
{
	return -EOPNOTSUPP;
}

static int
giwauth_wpa_enabled(struct net_device *dev,
                    struct iw_request_info *info, 
                    struct iw_param *erq, 
                    char *buf)
{
	int rc;
	int arg = IEEE80211_PARAM_WPA;

	rc = ieee80211_ioctl_getparam(dev, NULL, NULL, (char*)&arg);
	if (rc)
	{
		return rc;
	}
	erq->value = arg;
	return 0;

}

static int
giwauth_rx_unencrypted_eapol(struct net_device *dev,
                             struct iw_request_info *info, 
                             struct iw_param *erq, 
                             char *buf)
{
	return -EOPNOTSUPP;
}

static int
giwauth_roaming_control(struct net_device *dev,
                        struct iw_request_info *info, 
                        struct iw_param *erq, 
                        char *buf)
{
	int rc;
	int arg;

	arg = IEEE80211_PARAM_ROAMING;
	rc = ieee80211_ioctl_getparam(dev, NULL, NULL, (char*)&arg);
	if (rc)
	{
		return rc;
	}
	switch (arg) 
	{
		case IEEE80211_ROAMING_DEVICE:
		case IEEE80211_ROAMING_AUTO:
			erq->value = IW_AUTH_ROAMING_ENABLE;
			break;
		default:
			erq->value = IW_AUTH_ROAMING_DISABLE;
			break;
	}
	return 0;
}

static int
giwauth_privacy_invoked(struct net_device *dev,
                        struct iw_request_info *info, 
                        struct iw_param *erq,
                        char *buf)
{
	int rc;
	int arg;
	arg = IEEE80211_PARAM_PRIVACY;
	rc = ieee80211_ioctl_getparam(dev, NULL, NULL, (char*)&arg);
	if (rc)
	{
		return rc;
	}
	erq->value = arg;
	return 0;
}

static int
ieee80211_ioctl_giwauth(struct net_device *dev,
                        struct iw_request_info *info,
                        struct iw_param *erq,
                        char *buf)
{
	int rc = -EOPNOTSUPP;

	switch (erq->flags & IW_AUTH_INDEX) 
	{
		case IW_AUTH_WPA_VERSION:
			rc = giwauth_wpa_version(dev, info, erq, buf);
			break;
		case IW_AUTH_CIPHER_PAIRWISE:
			rc = giwauth_cipher_pairwise(dev, info, erq, buf);
			break;
		case IW_AUTH_CIPHER_GROUP:
			rc = giwauth_cipher_group(dev, info, erq, buf);
			break;
		case IW_AUTH_KEY_MGMT:
			rc = giwauth_key_mgmt(dev, info, erq, buf);
			break;
		case IW_AUTH_TKIP_COUNTERMEASURES:
			rc = giwauth_tkip_countermeasures(dev, info, erq, buf);
			break;
		case IW_AUTH_DROP_UNENCRYPTED:
			rc = giwauth_drop_unencrypted(dev, info, erq, buf);
			break;
		case IW_AUTH_80211_AUTH_ALG:
			rc = giwauth_80211_auth_alg(dev, info, erq, buf);
			break;
		case IW_AUTH_WPA_ENABLED:
			rc = giwauth_wpa_enabled(dev, info, erq, buf);
			break;
		case IW_AUTH_RX_UNENCRYPTED_EAPOL:
			rc = giwauth_rx_unencrypted_eapol(dev, info, erq, buf);
			break;
		case IW_AUTH_ROAMING_CONTROL:
			rc = giwauth_roaming_control(dev, info, erq, buf);
			break;
		case IW_AUTH_PRIVACY_INVOKED:
			rc = giwauth_privacy_invoked(dev, info, erq, buf);
			break;
		default:
			printk(KERN_WARNING "%s: unknown SIOCGIWAUTH flag %d\n",
			       dev->name, erq->flags);
			break;
	}

	return rc;
}

#ifdef ONEBOX_CONFIG_CFG80211
int
#else
static int
#endif
ieee80211_ioctl_setparam(struct net_device *dev, 
                         struct iw_request_info *info,
                         void *w, 
                         char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211com *ic = vap->iv_ic;
	struct ieee80211req ireq;
#ifdef ONEBOX_CONFIG_CFG80211
	struct iw_point wri;
#endif
	unsigned int param = *(unsigned int*)&extra[0];		/* parameter id is 1st */
	unsigned int value = *(unsigned int*)&extra[4];		/* NB: most values are TYPE_INT */
	int rate_hix = 0;
	struct ieee80211_rate *rs_supp = NULL; 
	int mode = vap->iv_des_mode, is11n;

	ireq.i_val = value; 
	IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("In %s and %d param %d value %d \n", __func__, __LINE__, param, value));
	switch (param) 
	{
		case IEEE80211_PARAM_PUREG: 
			if (value) 
			{
				ireq.i_val = !value;
				ireq.i_type = IEEE80211_IOC_PUREN;
				ieee80211_ioctl_set80211(vap, 0, &ireq);
			}
			ireq.i_val = value;
			ireq.i_type= IEEE80211_IOC_PUREG;
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_PUREN:
			if (value) 
			{
				ireq.i_val = !value;
				ireq.i_type = IEEE80211_IOC_PUREG;
				ieee80211_ioctl_set80211(vap, 0, &ireq);
			}
			ireq.i_val = value;
			ireq.i_type= IEEE80211_IOC_PUREN;
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_WMM:
			ireq.i_val = value;
			ireq.i_type= IEEE80211_IOC_WME;
			ic->ic_caps |= IEEE80211_C_WME; /* Enable WMM support */
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_MACCMD:
			ireq.i_val = value;
			ireq.i_type= IEEE80211_IOC_MACCMD;
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_DTIM_PERIOD:
			ireq.i_val = value;
			ireq.i_type = IEEE80211_IOC_DTIM_PERIOD;
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_CHANGE_COUNTRY_IE:
			ireq.i_val = value;
			ireq.i_type = IEEE80211_IOC_CHANGE_COUNTRY_IE;
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_HIDESSID:
			ireq.i_val = value;
			ireq.i_type = IEEE80211_IOC_HIDESSID;
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_DBG_ZONE:
			ireq.i_val = value;
			ireq.i_type = IEEE80211_IOC_DBG_ZONE;
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("Set debug zone as %d\n", ireq.i_val));
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_HTCONF:
			ireq.i_val = value;
			ireq.i_type = IEEE80211_IOC_HTCONF;
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("Set ht conf as %d\n", ireq.i_val));
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_AMPDU_DENSITY:
			ireq.i_val = value;
			ireq.i_type = IEEE80211_IOC_AMPDU_DENSITY;
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("Set AMPDU DENS as %d\n", ireq.i_val));
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_SHORT_GI:
			if(value == 0)
			{
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("Disable Short_Gi\n"));
				ireq.i_val = 0;
			}
			else if(value == 1)
			{
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("Set shortgi for 20Mhz\n"));
				ireq.i_val = IEEE80211_HTCAP_SHORTGI20;
			}
			else if(value == 2)
			{
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("Set shortgi for 40Mhz\n"));
				ireq.i_val = IEEE80211_HTCAP_SHORTGI40;
			}
			else if(value == 3)
			{
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("Set shortgi for 20Mhz and 40Mhz\n"));
				ireq.i_val = IEEE80211_HTCAP_SHORTGI20 | IEEE80211_HTCAP_SHORTGI40;
			}
			else
			{
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("invalid option\n"));
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Usage: 1-shortgi for 20MHz\t 2-shortgi for 40MHz\t 3-shortgi for both no argument for disable shortgi\n"));
        return -EINVAL;
			}
			ireq.i_type =  IEEE80211_IOC_SHORTGI;
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_HTCOMPAT:
			ireq.i_val = value;
			ireq.i_type = IEEE80211_IOC_HTCOMPAT;
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("Set HT compat \n"));
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_AMPDU:
			ireq.i_val = value;
			ireq.i_type = IEEE80211_IOC_AMPDU;
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("AMPDU parameter =%d \n", value));
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_AMPDU_LIMIT:
			ireq.i_val = value;
			ireq.i_type = IEEE80211_IOC_AMPDU_LIMIT;
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("Set AMPDURX_MAX limit \n"));
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_AMSDU_LIMIT:
			ireq.i_val = value;
			ireq.i_type = IEEE80211_IOC_AMSDU_LIMIT;
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("Set AMSDU limit \n"));
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_GREEN_FIELD:
			ireq.i_val = value;
			ireq.i_type = IEEE80211_IOC_GREENFIELD;
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("Set green field \n"));
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_PRIVACY:
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("In %s and %d Privacy %d \n", __func__, __LINE__, value));
			ireq.i_val  = value;
			ireq.i_type = IEEE80211_IOC_PRIVACY;
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_PARAM_ROAMING:
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("In %s and %d value %d \n", __func__, __LINE__, value));
			vap->hal_priv_vap->roam_ioctl = value;
			return 0;
			break;
		case IEEE80211_PARAM_BEACON_MISS_THRESH:
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("In %s and %d bmiss threshold %d \n", __func__, __LINE__, value));
			ireq.i_val  = value;
			ireq.i_type = IEEE80211_IOC_BMISSTHRESHOLD;
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
#ifdef ONEBOX_CONFIG_CFG80211
		case IEEE80211_PARAM_MODE:
			wri.length = extra[1];
			wri.pointer = &extra[4];
			return ieee80211_ioctl_setmode(dev, info, &wri, NULL);
#endif
		case IEEE80211_PARAM_RTP_PRINTS:
			vap->hal_priv_vap->rtp_prints = value;
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("RTP prints output=%d\n", vap->hal_priv_vap->rtp_prints));
		break;
		case IEEE80211_PARAM_SHPREAMBLE:
			if(value == 1) /* Indicates auto mode */
			{
				/* Use whatever capability AP supports, indicates auto mode support */	
				vap->hal_priv_vap->preamble = 1;
			}
			else
			{
				/* Indicates use short preamble */
				vap->hal_priv_vap->preamble = 0;
			}
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("Preamble: In %s %d preamble=%d\n", __func__, __LINE__, vap->hal_priv_vap->preamble));
		break;
		
		case IEEE80211_KEEP_ALIVE_PERIOD:
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("In %s and %d keep alive period %d \n", __func__, __LINE__, value));
			ireq.i_val  = value;
			ireq.i_type = IEEE80211_IOC_KEEP_ALIVE_PERIOD;
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		case IEEE80211_DFS_CHAN_TO_SWITCH:
			ireq.i_val = value;
			ireq.i_type = IEEE80211_IOC_CHAN_TO_SWITCH;
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
		case IEEE80211_DEFAULT_MGMT_RATE:
			if (value == 0) {
				vap->hal_priv_vap->mgmt_rate_enable = 0;
				return 0;
			}
			rs_supp = (struct ieee80211_rate *)&ic->ic_sup_rates[mode]; /* NB: 11n maps to legacy */
			is11n = (mode == IEEE80211_MODE_11NA ||mode == IEEE80211_MODE_11NG);
			if(vap->iv_opmode == IEEE80211_M_HOSTAP) {
				if(((ic->ic_regdomain.isocc[0] == 'J') && (ic->ic_regdomain.isocc[1] == 'P')) && (ic->ic_curchan->ic_ieee == 14 )){
					if(!checkrate((const struct ieee80211_rateset *)rs_supp, value)) {
						IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Fixed rate : Invalid rate, Please give valid rate\n"));
						return EINVAL;
					}
				}
				else if ((!checkrate((const struct ieee80211_rateset *)rs_supp, value)) &&
						!(is11n && ((ishtrate(ic, value))))) {
					IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Fixed rate : Invalid rate, Please give valid rate\n"));
					return EINVAL;
				}
			}
			else if(vap->iv_opmode == IEEE80211_M_STA){
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Couldn't issue this ioctl in current operating mode\n"));
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Set mgmt rate is applicable only in AP mode\n"));
				return EINVAL;
			}
			rate_hix = user_rate_to_rps_rate(value);
			if (rate_hix >= 0) {
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, 
						("In %s and %d User Rate %d Rate Index %d\n", __func__, __LINE__, 
						 value, vap->hal_priv_vap->default_mgmt_rate));
				vap->hal_priv_vap->default_mgmt_rate = rate_hix;
				vap->hal_priv_vap->mgmt_rate_enable = 1;
				return 0;
			}
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("In %s and %d Invalid Rate %d \n", __func__, __LINE__, value));
			break;
		case IEEE80211_PARAM_DOTH:
			ireq.i_val = value;
			ireq.i_type = IEEE80211_IOC_DOTH;
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("Set 802.11h Enable/disable value=%d \n", value));
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
		default:
			ireq.i_type = param;
			return ieee80211_ioctl_set80211(vap, 0, &ireq);
			break;
	}
	return -1;
}

static int
ieee80211_ioctl_siwencode(struct net_device *dev,
                          struct iw_request_info *info, 
                          struct iw_point *erq, 
                          char *keybuf)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
	int ret;

	return -1;
#if 0
	if(erq->flags == IW_ENCODE_RESTRICTED)
	{
		/*set the encryption mode*/
		ireq.i_type = IEEE80211_IOC_WEP;
		ireq.i_val = IEEE80211_WEP_MIXED;
		ret = ieee80211_ioctl_set80211(vap, 0, &ireq);

		/*set the transmit key index*/	
		ireq.i_type = IEEE80211_IOC_WEPTXKEY;
		ireq.i_val = 0; /*Default key index will set to 0*/
		ret = ieee80211_ioctl_set80211(vap, 0, &ireq);
		
		/*set the key*/
		//ireq.i_val = 0; /* key index*/
		ireq.i_data = erq->pointer;
		ireq.i_len = erq->length;
		ireq.i_type = IEEE80211_IOC_WEPKEY;
		return ieee80211_ioctl_set80211(vap, 0, &ireq);
	}
	else if(erq->flags == IW_ENCODE_OPEN)
	{
		ireq.i_type = IEEE80211_IOC_AUTHMODE;
		ireq.i_val = IEEE80211_AUTH_OPEN;
		return ieee80211_ioctl_set80211(vap, 0, &ireq);
	}
	return 0;
#endif
}

static int
ieee80211_ioctl_giwencode(struct net_device *dev,
        struct iw_request_info *info, union iwreq_data *encoding, char *keybuf)
{

#if 1
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
	uint32_t ii=0;
	
	if(vap->iv_state != IEEE80211_S_RUN)
	{
			encoding->encoding.length=0;
			encoding->encoding.flags = IW_ENCODE_DISABLED;
			return 0;
	}

	memset(&ireq, 0, sizeof(struct ieee80211req));
	ireq.i_type = IEEE80211_IOC_AUTHMODE;

	if (ieee80211_ioctl_get80211(vap, 0, &ireq))
		return -1;

	if(ireq.i_val == IEEE80211_AUTH_WPA)
	{
		if(vap->iv_def_txkey <= 3)
		{
			encoding->encoding.length = (vap->iv_nw_keys[vap->iv_def_txkey].wk_keylen)/4;
			encoding->encoding.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY | IW_ENCODE_RESTRICTED ;
		}
		else
		{
			for(ii=0; ii<4; ii++) {
					if((vap->iv_nw_keys[ii].wk_keylen)){
							encoding->encoding.length = vap->iv_nw_keys[ii].wk_keylen/4;
							break;
					}
			}
			encoding->encoding.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY | IW_ENCODE_RESTRICTED ;
		}
	}
	else
	{
		ireq.i_type = IEEE80211_IOC_WEP;
		if (ieee80211_ioctl_get80211(vap, 0, &ireq))
			return -1;
		
		if(ireq.i_val == IEEE80211_WEP_ON || ireq.i_val == IEEE80211_WEP_MIXED) 
		{
			ireq.i_type = IEEE80211_IOC_WEPKEY;

			if(vap->iv_def_txkey <=3 ) {
					ireq.i_val = vap->iv_def_txkey;
			} else {
					ireq.i_val = 0;
			}

			if (ieee80211_ioctl_get80211(vap, 0, &ireq) < 0)
			{
				return -1;
			}

					encoding->encoding.length = ireq.i_len ; 
					encoding->encoding.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY | IW_ENCODE_RESTRICTED ;
		}
		else
		{
			encoding->encoding.length=0;
			encoding->encoding.flags = IW_ENCODE_DISABLED;
		}
	}
#endif
	return 0;
}
static int
ieee80211_ioctl_giwgenie(struct net_device *dev,
                         struct iw_request_info *info,
                         struct iw_point *out,
                         char *buf)
{
	return 0;
}

static int
ieee80211_ioctl_addmac(struct net_device *dev,
                       struct iw_request_info *info,
                       void *w, 
                       char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct sockaddr *sa = (struct sockaddr *)extra;
	struct ieee80211req ireq;

	memset(&ireq, 0, sizeof(struct ieee80211req));
	ireq.i_data = sa->sa_data;
	ireq.i_type = IEEE80211_IOC_ADDMAC;
	ireq.i_len  = IEEE80211_ADDR_LEN;

	return ieee80211_ioctl_set80211(vap , 0 , &ireq);
}

static int
ieee80211_ioctl_delmac(struct net_device *dev,
                       struct iw_request_info *info,
                       void *w, 
                       char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct sockaddr *sa = (struct sockaddr *)extra;
	struct ieee80211req ireq;

	memset(&ireq, 0, sizeof(struct ieee80211req));
	ireq.i_data = sa->sa_data;
	ireq.i_type = IEEE80211_IOC_DELMAC;
	ireq.i_len  = IEEE80211_ADDR_LEN;

	return ieee80211_ioctl_set80211(vap , 0 , &ireq);
}


static int
ieee80211_ioctl_siwencodeext(struct net_device *dev,
                             struct iw_request_info *info, 
                             struct iw_point *erq, 
                             char *extra)
{
	return 0;
}
static int
ieee80211_ioctl_setscanlist(struct net_device *dev,
                            struct iw_request_info *info,
                            struct iw_point *data, 
                            char *extra)
{
	return 0;
}
#ifdef IEEE80211K
static int
ieee80211_ioctl_msrmnt_req(struct net_device *dev,
                            struct iw_request_info *info,
                            struct iw_point *data, 
                            char *extra)                      //This function is called when iwpriv ioctl with rm requestis given with required msrmnt parameters           
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
	memset(&ireq, 0, sizeof(struct ieee80211req));
	ireq.i_type = IEEE80211_IOC_11K_MSRMNT_REQ;
	ireq.i_data = extra;
		
	return ieee80211_ioctl_set80211(vap, 0, &ireq);

}

static int
ieee80211_ioctl_sm_msrmnt_req(struct net_device *dev,
														struct iw_request_info *info,
														struct iw_point *data, 
														char *extra)                             //This function is called when iwpriv ioctl with sm requestis given with required msrmnt parameters
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;

	memset(&ireq, 0, sizeof(struct ieee80211req));
	ireq.i_type = IEEE80211_IOC_11K_SM_MSRMNT_REQ;
	ireq.i_data = extra;	
	return ieee80211_ioctl_set80211(vap, 0, &ireq);
}
#endif
static int
ieee80211_ioctl_kickmac(struct net_device *dev, 
                        struct iw_request_info *info,
                        void *w, 
                        char *extra)
{
	return 0;
}

static int
ieee80211_ioctl_wdssetmac(struct net_device *dev, 
                          struct iw_request_info *info,
                          void *w, 
                          char *extra)
{
	return 0;
}
static int
ieee80211_ioctl_wdsaddmac(struct net_device *dev, 
                          struct iw_request_info *info,
                          void *w, 
                          char *extra)
{
	return 0;
}
static int
ieee80211_ioctl_delkey(struct net_device *dev, 
                       struct iw_request_info *info,
                       void *w, 
                       char *extra)
{
	return 0;
}
static int
ieee80211_ioctl_setkey(struct net_device *dev, 
                       struct iw_request_info *info,
                       void *w, 
                       char *extra)
{
	return 0;
}
static int
ieee80211_ioctl_getoptie(struct net_device *dev, 
                         struct iw_request_info *info,
                         struct iw_point *wri, 
                         char *extra)
{
	return 0;
}
static int
ieee80211_ioctl_setmlme(struct net_device *dev, struct iw_request_info *info,
                        void *w, char *extra)
{
	return 0;
}
#ifdef ONEBOX_CONFIG_CFG80211
int
#else
static int
#endif
ieee80211_ioctl_setmode(struct net_device *dev, struct iw_request_info *info,
                        struct iw_point *wri, char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211vap *vap_temp;
	struct ifreq ifr;
	struct ieee80211com *ic = vap->iv_ic;
	char s[6];
	int retv, mode, ifr_mode;
	int band = 0;

	if (vap->iv_media.ifm_cur == NULL)
	{
		return -EINVAL;                // XXX: Wrong error 
	}
	if (wri->length > sizeof(s))            // silently truncate 
	{
		wri->length = sizeof(s) - 1;
	}

	if (copyin(wri->pointer, s, wri->length))
	{
		return -EFAULT;
	}
	s[wri->length]= '\0';
	mode = ieee80211_convert_mode(s, 0);
	if (mode < 0)
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("No mode match: %s\n", s));
		return -EINVAL;
	}

	band = check_band(mode);

	ifr_mode = mode;
	memset(&ifr, 0, sizeof(ifr));
	ifr.ifr_media = vap->iv_media.ifm_cur->ifm_media & ~IFM_MMASK;
	if (mode == IEEE80211_MODE_TURBO_STATIC_A)
	{
		ifr_mode = IEEE80211_MODE_11A;
	}
	ifr.ifr_media |=  ieee80211_convert_mode(s, 1);

	retv = ifmedia_ioctl(vap->iv_ifp, &ifr, &vap->iv_media, SIOCSIFMEDIA);

	if ((!retv || retv == -ENETRESET) && 
	    (mode != vap->iv_des_mode)) 
	{
		vap->iv_des_mode = mode;
		if (IS_UP_AUTO(vap))
		{
			ieee80211_new_state(vap, IEEE80211_S_SCAN, 0);
		}
		retv = 0;
	}
	else 
	{
		if (vap->iv_opmode == IEEE80211_M_HOSTAP) {
			TAILQ_FOREACH(vap_temp, &ic->ic_vaps, iv_next) {
				if ((vap_temp->iv_opmode == IEEE80211_M_STA) &&
						(vap_temp->iv_state == IEEE80211_S_RUN)) {
					IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, (" Station Mode vap is already running so do not set band\n"));
					return 0;
				}
			}
		}

		ic->ic_set_params(ic, IEEE80211_IOCTL_SETMODE, band);
	}
	return -retv;
}

#ifndef ONEBOX_CONFIG_CFG80211
static int
#else
int
#endif
ieee80211_ioctl_getmode(struct net_device *dev, 
                        struct iw_request_info *info,
                        struct iw_point *wri, 
                        char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif

	struct ifmediareq imr;

	vap->iv_media.ifm_status(vap->iv_ifp, &imr);
	switch (IFM_MODE(imr.ifm_active)) 
	{
		case IFM_IEEE80211_11A:
			strcpy(extra, "11a");
			break;
		case IFM_IEEE80211_11B:
			strcpy(extra, "11b");
			break;
		case IFM_IEEE80211_11G:
			strcpy(extra, "11g");
			break;
		case IFM_IEEE80211_11NG:
			strcpy(extra, "11n");
			break;
		case IFM_IEEE80211_FH:
			strcpy(extra, "FH");
			break;
		case IFM_IEEE80211_11NA:
			strcpy(extra, "11na");
			break;
		case IFM_AUTO:
			strcpy(extra, "auto");
			break;
		default:
			return -EINVAL;
	}
	wri->length = strlen(extra);
	return 0;
}

static int
ieee80211_ioctl_giwencodeext(struct net_device *dev,
                             struct iw_request_info *info, 
                             struct iw_point *erq, 
                             char *extra)
{
	return 0;
}
static int
ieee80211_ioctl_setoptie(struct net_device *dev,  
                         struct iw_request_info *info,
                         struct iw_point *wri,
                         char *extra)
{
	return 0;
}

static int
ieee80211_ioctl_setwmmparams(struct net_device *dev,
                             struct iw_request_info *info, 
                             struct iw_point *data, 
                             char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
	uint32_t *param = (uint32_t *)extra;

	vap->hal_priv_vap->update_wmmparams = 0;/**Clear this flag for every ioctl as count is increasing for every ioctl**/
	memset(&ireq, 0, sizeof(struct ieee80211req));
	ireq.i_type = param[0];
	ireq.i_val = param[1];
	ireq.i_len = (param[2] & 0x7FFF);
	if(param[3])
	{
		ireq.i_len |= IEEE80211_WMEPARAM_BSS;
	}

	if(param[4])
	{
		/* Used to indicate the end of wmm parameters from script .....Need to improvise */
		vap->hal_priv_vap->update_wmmparams = 1;
	}
	return ieee80211_ioctl_set80211(vap, 0, &ireq); 
}

static int
ieee80211_ioctl_getwmmparams(struct net_device *dev,
                             struct iw_request_info *info, 
                             void *w, 
                             char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
	unsigned int *param = (unsigned int *) extra;
	
	memset(&ireq, 0, sizeof(struct ieee80211req));
	ireq.i_type = param[0];

	if (ieee80211_ioctl_get80211(vap, 0, &ireq)) 
	{
		return -EINVAL;
	}
	else
	{
		param[0] = ireq.i_val;
		/* Fill in AC */
		param[1] = (((ireq.i_len & 0x7fff) >> 12) & 0x7);
		/* Fill in BSS */
		param[2] = (ireq.i_len & 0x8000); 
	}
	return 0;
}


/*static int
ieee80211_ioctl_setchanlist(struct net_device *dev,
	struct iw_request_info *info, void *w, char *extra)
{
	struct ieee80211vap *vap = dev->priv;
	struct ieee80211req ireq;

	memset(&ireq, 0, sizeof(ireq));
	ireq.i_length = IEEE80211_CHAN_MAX;
	ireq.i_data = extra;
	ireq.i_type = IEEE80211_IOC_CHANLIST;

	return ieee80211_ioctl_set80211(vap , 0 , &ireq);
}*/

static int
ieee80211_ioctl_getchanlist(struct net_device *dev,
                            struct iw_request_info *info, 
                            void *w, 
                            char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;

	memset(&ireq, 0, sizeof(struct ieee80211req));
	ireq.i_type = IEEE80211_IOC_CHANLIST;
	ireq.i_data = extra;

	if (ieee80211_ioctl_get80211(vap, 0, &ireq))
	{
		return -EINVAL;
	}
	else
	{
		return 0;
	}
}

/*static int
ieee80211_ioctl_chanswitch(struct net_device *dev, struct iw_request_info *info,
	void *w, char *extra)
{
	struct ieee80211vap *vap = dev->priv;
	struct ieeee80211req ireq;

	memset(&ireq, 0, sizeof(struct ieee80211req));

	ireq.i_type   = IEEE80211_IOC_CHANSWITCH;
	ireq.i_length = sizeof(struct ieee80211_chanswitch_req);
	ireq.i_data   = extra;

	return ieee80211_ioctl_set80211(vap , 0 , &ireq);
}*/

static int
ieee80211_ioctl_setappiebuf(struct net_device *dev, 
                            struct iw_request_info *info,
                            struct iw_point *data, 
                            char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
	struct ieee80211req_getset_appiebuf *iebuf = (struct ieee80211req_getset_appiebuf *)extra;

	memset(&ireq, 0, sizeof(struct ieee80211req));
	ireq.i_val  = iebuf->app_frmtype;
	ireq.i_len  = data->length;  
	ireq.i_data = data->pointer;
	ireq.i_type = IEEE80211_IOC_APPIE;

	return ieee80211_ioctl_set80211(vap , 0 , &ireq);
}

static int
ieee80211_ioctl_getappiebuf(struct net_device *dev, 
                            struct iw_request_info *info,
                            struct iw_point *data, 
                            char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211req ireq;
	struct ieee80211req_getset_appiebuf *iebuf = (struct ieee80211req_getset_appiebuf *)extra;

	memset(&ireq, 0, sizeof(struct ieee80211req));
	ireq.i_data = data->pointer;
	ireq.i_val  = iebuf->app_frmtype; /* Check this */
	ireq.i_type = IEEE80211_IOC_APPIE;

	if (ieee80211_ioctl_get80211(vap, 0, &ireq))
	{
		return -EINVAL;
	}
	else 
	{
		data->length = ireq.i_len;
	}
	return 0;
}

#ifdef ONEBOX_CONFIG_CFG80211
int
#else
static int
#endif
ieee80211_ioctl_giwrate(struct net_device *dev, 
                        struct iw_request_info *info,
                        struct iw_param *rrq, 
                        char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	uint16_t rate = 0;
    uint16_t rate_val = 0;

	if(vap == NULL)
	{		
			return -1;
	}

	if (vap->hal_priv_vap->fixed_rate_enable)
	{
	/* since the mode is always 11bgn, so hardcoding for now */
	rrq->value = (vap->iv_txparms[vap->iv_des_mode].ucastrate * 500000);
	rrq->fixed = 1;
	}
	else
    {
        if(vap->iv_opmode == IEEE80211_M_HOSTAP)
        {
            rate_val = vap->cur_rate;
        }

        if(vap->iv_opmode == IEEE80211_M_STA)
        {
            rate_val = vap->iv_bss->ni_txrate;
        }
	
		//! Checking ni_chan for IEEE80211_CHAN_ANYC token, ni_chan is getting initialized with
		//! IEEE80211_CHAN_ANYC value while removing driver, so check is needed to avoid dereferencing
		//! the ni_chan structure, if any rate ioctl is issued soon after removing driver.
		if (vap->iv_bss->ni_chan == IEEE80211_CHAN_ANYC) {
			return -ENODEV;
		}
        if (rate_val & MCS_RATE) {
            const struct ieee80211_mcs_rates *mcs =
                &ieee80211_htrates[rate_val & MCS_INDEX];

            if (IEEE80211_IS_CHAN_HT40(vap->iv_bss->ni_chan)) {
                if (rate_val & SGI_ENABLE) 
                    rate = mcs->ht40_rate_400ns;
                else
                    rate = mcs->ht40_rate_800ns;
            } else {
                if (rate_val & SGI_ENABLE)
                    rate = mcs->ht20_rate_400ns;
                else
                    rate = mcs->ht20_rate_800ns;
            }
        } else {
            rate = map_idx_to_rate(rate_val);
        }

        rrq->value = rate*500000;
        rrq->fixed = 0;

    }
	return 0;
}

#ifdef ONEBOX_CONFIG_CFG80211
int
#else
static int
#endif
ieee80211_ioctl_siwrate(struct net_device *dev, 
                        struct iw_request_info *info,
                        struct iw_param *rrq, 
                        char *extra)
{
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif
	struct ieee80211com *ic = vap->iv_ic;
	int user_rate, mode = vap->iv_des_mode, is11n;
	struct ieee80211_rate *rs_supp = NULL; 
	struct ieee80211_node *ni = vap->iv_bss; 
	struct ieee80211_rateset *rs = NULL;
	struct ieee80211_htrateset *htrs = NULL;
	int mcs[] = {13, 26, 39, 52, 78, 104, 117, 130};
	uint8_t i, found = 0 ;

	if (rrq->value) 
	{
		user_rate = (rrq->value);
	}
	else
	{ 
		user_rate = IEEE80211_FIXED_RATE_NONE;
		vap->hal_priv_vap->fixed_rate_enable = 0;
		vap->vap_dynamic_update(vap);
		return 0;
	}

	rs_supp = (struct ieee80211_rate *)&ic->ic_sup_rates[mode]; /* NB: 11n maps to legacy */
	is11n = (mode == IEEE80211_MODE_11NA ||mode == IEEE80211_MODE_11NG);
	if(vap->iv_opmode == IEEE80211_M_HOSTAP) {
		if(((ic->ic_regdomain.isocc[0] == 'J') && (ic->ic_regdomain.isocc[1] == 'P')) && (ic->ic_curchan->ic_ieee == 14 )){
			if(!checkrate((const struct ieee80211_rateset *)rs_supp, user_rate)) {
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Fixed rate : Invalid rate, Please give valid rate\n"));
				return EINVAL;
			}
		}
		else if ((!checkrate((const struct ieee80211_rateset *)rs_supp, user_rate)) &&
				!(is11n && ((ishtrate(ic, user_rate))))) {
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Fixed rate : Invalid rate, Please give valid rate\n"));
			return EINVAL;
		}
		found = 1;
	} else if(vap->iv_opmode == IEEE80211_M_STA) {
		if(vap->iv_state == IEEE80211_S_RUN) {
			/*set values if vap in connected state*/
			rs = (struct ieee80211_rateset *)&ni->ni_rates;
			IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_IOCTL, ("In %s line %d supported rates by ap\n",__func__,__LINE__));
			htrs=(struct ieee80211_htrateset *)&ni->ni_htrates;
			if(checkrate(rs, user_rate)) {
				for(i = 0; i < rs->rs_nrates; i++) {
					if((rs->rs_rates[i] & IEEE80211_RATE_VAL) == user_rate) {	
						found = 1;
					}
				}
			} else if((htrs->rs_nrates > 0) && ishtrate(ic, user_rate)) {
				for(i = 0; i < htrs->rs_nrates; i++) {
					if(mcs[htrs->rs_rates[i] & IEEE80211_RATE_VAL] == user_rate) {
						found = 1;
					}
				}
			}	else {
				IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("\n<<<given rate is  not supported in current mode>>>\n"));
				return EINVAL;
			}						
		}
	}

	if(found) {
		set_fixedrate(vap, user_rate);
		vap->iv_txparms[vap->iv_des_mode].ucastrate = user_rate;
		vap->vap_dynamic_update(vap);
		return 0;
	}
	return EINVAL;

}

/*static int
ieee80211_ioctl_getchaninfo(struct net_device *dev,
	struct iw_request_info *info, void *w, char *extra)
{
	struct ieee80211vap *vap = dev->priv;
	struct ieee80211req ireq;

	memset(&ireq, 0, sizeof(ireq));
	ireq.i_data = extra;
	ireq.i_type = IEEE80211_IOC_CHANINFO;

	if (ieee80211_ioctl_get80211(vap, 0, &ireq))
		return -EINVAL;
	else 
		return 0;	
}**/

#define set_handler(x,f) [x - SIOCIWFIRST] = (iw_handler) f
static const iw_handler ieee80211_handlers[] = 
{
	set_handler(SIOCGIWNAME, ieee80211_ioctl_giwname),
	set_handler(SIOCSIWFREQ, ieee80211_ioctl_siwfreq),
	set_handler(SIOCGIWFREQ, ieee80211_ioctl_giwfreq),
	set_handler(SIOCSIWMODE, ieee80211_ioctl_siwmode),
	set_handler(SIOCGIWMODE, ieee80211_ioctl_giwmode),
	set_handler(SIOCSIWSENS, ieee80211_ioctl_siwsens),
	set_handler(SIOCGIWSENS, ieee80211_ioctl_giwsens),
	set_handler(SIOCGIWRANGE,ieee80211_ioctl_giwrange), /* NA */
	set_handler(SIOCSIWSPY,  NULL), /* NA */
	set_handler(SIOCGIWSPY,  NULL), /* NA */
#if WIRELESS_EXT >= 16
	set_handler(SIOCSIWTHRSPY, NULL), /* NA */
	set_handler(SIOCGIWTHRSPY, NULL), /* NA */
#endif
	set_handler(SIOCSIWAP, ieee80211_ioctl_siwap), 
	set_handler(SIOCGIWAP, ieee80211_ioctl_giwap), 
#ifdef SIOCSIWMLME
	set_handler(SIOCSIWMLME, ieee80211_ioctl_siwmlme), /* NA */
#endif
	set_handler(SIOCGIWAPLIST, NULL), 
#ifdef SIOCGIWSCAN
	set_handler(SIOCSIWSCAN, ieee80211_ioctl_siwscan),
	set_handler(SIOCGIWSCAN, ieee80211_ioctl_giwscan), /* NY */
#endif /* SIOCGIWSCAN */
	set_handler(SIOCSIWESSID, ieee80211_ioctl_siwessid), 
	set_handler(SIOCGIWESSID, ieee80211_ioctl_giwessid), 
	set_handler(SIOCSIWNICKN, ieee80211_ioctl_siwnickn),
	set_handler(SIOCGIWNICKN, ieee80211_ioctl_giwnickn),
	set_handler(SIOCSIWRATE, ieee80211_ioctl_siwrate), /* NY */
	set_handler(SIOCGIWRATE, ieee80211_ioctl_giwrate), /* NY */
	set_handler(SIOCSIWRTS, ieee80211_ioctl_siwrts),
	set_handler(SIOCGIWRTS, ieee80211_ioctl_giwrts),
	set_handler(SIOCSIWFRAG, ieee80211_ioctl_siwfrag),
	set_handler(SIOCGIWFRAG, ieee80211_ioctl_giwfrag),
	set_handler(SIOCSIWTXPOW, ieee80211_ioctl_siwtxpow), /* NY */
	set_handler(SIOCGIWTXPOW, ieee80211_ioctl_giwtxpow), 
	set_handler(SIOCSIWRETRY, ieee80211_ioctl_siwretry),
	set_handler(SIOCGIWRETRY, ieee80211_ioctl_giwretry),
	set_handler(SIOCSIWENCODE, ieee80211_ioctl_siwencode),
	set_handler(SIOCGIWENCODE, ieee80211_ioctl_giwencode), /* NY */
	set_handler(SIOCSIWPOWER, ieee80211_ioctl_siwpower),
	set_handler(SIOCGIWPOWER, ieee80211_ioctl_giwpower), 
#if WIRELESS_EXT >= 18
	set_handler(SIOCSIWGENIE, NULL),         //ieee80211_ioctl_siwgenie), /* NY */
	set_handler(SIOCGIWGENIE, ieee80211_ioctl_giwgenie),
	set_handler(SIOCSIWAUTH, ieee80211_ioctl_siwauth),
	set_handler(SIOCGIWAUTH, ieee80211_ioctl_giwauth),
	set_handler(SIOCSIWENCODEEXT, ieee80211_ioctl_siwencodeext),
	set_handler(SIOCGIWENCODEEXT, ieee80211_ioctl_giwencodeext),
#endif /* WIRELESS_EXT >= 18 */
};

#define set_priv(x,f) [x - SIOCIWFIRSTPRIV] = (iw_handler) f
static const iw_handler ieee80211_priv_handlers[] = 
{
	set_priv(SIOCS80211, ieee80211_ioctl_setconfig),
	set_priv(SIOCG80211, ieee80211_ioctl_getconfig),
	set_priv(SIOCSIFMEDIA, ieee80211_ioctl_setifmedia),
	set_priv(SIOCGIFMEDIA, ieee80211_ioctl_ifmedia),
	set_priv(IEEE80211_IOCTL_SETPARAM, ieee80211_ioctl_setparam),
	set_priv(IEEE80211_IOCTL_GETPARAM, ieee80211_ioctl_getparam),
	set_priv(IEEE80211_IOCTL_SETMODE, ieee80211_ioctl_setmode),
	set_priv(IEEE80211_IOCTL_GETMODE, ieee80211_ioctl_getmode),
	set_priv(IEEE80211_IOCTL_SETWMMPARAMS, ieee80211_ioctl_setwmmparams),
	set_priv(IEEE80211_IOCTL_GETWMMPARAMS, ieee80211_ioctl_getwmmparams),
	set_priv(IEEE80211_IOCTL_SETCHANLIST, NULL),      //ieee80211_ioctl_setchanlist),
	set_priv(IEEE80211_IOCTL_GETCHANLIST, ieee80211_ioctl_getchanlist),
	set_priv(IEEE80211_IOCTL_CHANSWITCH, NULL),      //ieee80211_ioctl_chanswitch),
	set_priv(IEEE80211_IOCTL_GET_APPIEBUF, ieee80211_ioctl_getappiebuf),
	set_priv(IEEE80211_IOCTL_SET_APPIEBUF, ieee80211_ioctl_setappiebuf),
	set_priv(IEEE80211_IOCTL_GETCHANINFO, NULL),         //ieee80211_ioctl_getchaninfo),
	set_priv(IEEE80211_IOCTL_SETOPTIE, ieee80211_ioctl_setoptie),
	set_priv(IEEE80211_IOCTL_GETOPTIE, ieee80211_ioctl_getoptie),
	set_priv(IEEE80211_IOCTL_SETMLME, ieee80211_ioctl_setmlme),
	set_priv(IEEE80211_IOCTL_SETKEY, ieee80211_ioctl_setkey),
	set_priv(IEEE80211_IOCTL_DELKEY, ieee80211_ioctl_delkey),
#ifdef AR_DEBUG
	set_priv(IEEE80211_IOCTL_HALMAP, ieee80211_ioctl_hal_map),
#endif
	set_priv(IEEE80211_IOCTL_ADDMAC, ieee80211_ioctl_addmac),
	set_priv(IEEE80211_IOCTL_DELMAC, ieee80211_ioctl_delmac),
	set_priv(IEEE80211_IOCTL_WDSADDMAC, ieee80211_ioctl_wdsaddmac),
	set_priv(IEEE80211_IOCTL_WDSSETMAC, ieee80211_ioctl_wdssetmac),
	set_priv(IEEE80211_IOCTL_KICKMAC, ieee80211_ioctl_kickmac),
	set_priv(IEEE80211_IOCTL_SETSCANLIST, ieee80211_ioctl_setscanlist),
#ifdef ATH_REVERSE_ENGINEERING
	set_priv(IEEE80211_IOCTL_READREG, ieee80211_ioctl_readreg),
	set_priv(IEEE80211_IOCTL_WRITEREG, ieee80211_ioctl_writereg),
#endif /* #ifdef ATH_REVERSE_ENGINEERING */
#ifdef IEEE80211K
	set_priv(IEEE80211_IOCTL_11K_MSRMNT_REQ, ieee80211_ioctl_msrmnt_req),
	set_priv(IEEE80211_IOCTL_11K_SM_MSRMNT_REQ, ieee80211_ioctl_sm_msrmnt_req),
#endif

};

static const struct iw_priv_args ieee80211_priv_args[] = 
{
	/* NB: setoptie & getoptie are !IW_PRIV_SIZE_FIXED */
	{SIOCS80211,
	0X4000 | 0X0800 | 1, 0, "set80211"},
	{ SIOCG80211,
	0X4000 | 0x0800 | 1, 0, "get80211" },
	{SIOCSIFMEDIA,
	0X4000 | 0X0800 | 1, 0, "onebox_ifmedia"},
	{SIOCGIFMEDIA,
	0X4000 | 0X0800 | 1, 0, "ifmedia_ioctl_set"},
	{ IEEE80211_IOCTL_SETOPTIE,IW_PRIV_TYPE_OPTIE, 0,"setoptie" },
	{ IEEE80211_IOCTL_GETOPTIE,0, IW_PRIV_TYPE_OPTIE,"getoptie" },
	{ IEEE80211_IOCTL_SETKEY,
	  IW_PRIV_TYPE_KEY | IW_PRIV_SIZE_FIXED, 0,"setkey" },
	{ IEEE80211_IOCTL_DELKEY,
	  IW_PRIV_TYPE_DELKEY | IW_PRIV_SIZE_FIXED, 0,"delkey" },
	{ IEEE80211_IOCTL_SETMLME,
	  IW_PRIV_TYPE_MLME | IW_PRIV_SIZE_FIXED, 0,"setmlme" },
	{ IEEE80211_IOCTL_ADDMAC,
	  IW_PRIV_TYPE_ADDR | IW_PRIV_SIZE_FIXED | 1, 0,"addmac" },
	{ IEEE80211_IOCTL_DELMAC,
	  IW_PRIV_TYPE_ADDR | IW_PRIV_SIZE_FIXED | 1, 0,"delmac" },
	{ IEEE80211_IOCTL_KICKMAC,
	  IW_PRIV_TYPE_ADDR | IW_PRIV_SIZE_FIXED | 1, 0, "kickmac"},
	{ IEEE80211_IOCTL_WDSADDMAC,
	  IW_PRIV_TYPE_ADDR | IW_PRIV_SIZE_FIXED | 1, 0,"wds_add" },
	{ IEEE80211_IOCTL_WDSSETMAC,
	  IW_PRIV_TYPE_ADDR | IW_PRIV_SIZE_FIXED | 1, 0,"wds_set" },
	{ IEEE80211_IOCTL_SETCHANLIST,
	  IW_PRIV_TYPE_CHANLIST | IW_PRIV_SIZE_FIXED, 0,"setchanlist" },
	{ IEEE80211_IOCTL_GETCHANLIST,
	  0, IW_PRIV_TYPE_CHANLIST | IW_PRIV_SIZE_FIXED,"getchanlist" },
	{ IEEE80211_IOCTL_GETCHANINFO,
	  0, IW_PRIV_TYPE_CHANINFO | IW_PRIV_SIZE_FIXED,"getchaninfo" },
	{ IEEE80211_IOCTL_SETMODE,
	  IW_PRIV_TYPE_CHAR |  6, 0, "mode" },
	{ IEEE80211_IOCTL_GETMODE,
	  0, IW_PRIV_TYPE_CHAR | 6, "get_mode" },
	{ IEEE80211_IOCTL_SETWMMPARAMS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 5, 0,"setwmmparams" },
	{ IEEE80211_IOCTL_GETWMMPARAMS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 3,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,   "getwmmparams" },
	{ IEEE80211_IOCTL_RADAR,
	  0, 0, "doth_radar" },
#ifdef AR_DEBUG
	{ IEEE80211_IOCTL_HALMAP,
	  0, 0, "dump_hal_map" },
#endif
	/*
	 * These depends on sub-ioctl support which added in version 12.
	 */
	/* sub-ioctl handlers */
	{ IEEE80211_WMMPARAMS_CWMIN,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 3, 0,"cwmin" },
	{ IEEE80211_WMMPARAMS_CWMIN,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,   "get_cwmin" },
	{ IEEE80211_WMMPARAMS_CWMAX,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 3, 0,"cwmax" },
	{ IEEE80211_WMMPARAMS_CWMAX,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,   "get_cwmax" },
	{ IEEE80211_WMMPARAMS_AIFS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 3, 0,"aifs" },
	{ IEEE80211_WMMPARAMS_AIFS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,   "get_aifs" },
	{ IEEE80211_WMMPARAMS_TXOPLIMIT,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 3, 0,"txoplimit" },
	{ IEEE80211_WMMPARAMS_TXOPLIMIT,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,   "get_txoplimit" },
	{ IEEE80211_WMMPARAMS_ACM,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 3, 0,"acm" },
	{ IEEE80211_WMMPARAMS_ACM,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,   "get_acm" },
	{ IEEE80211_WMMPARAMS_NOACKPOLICY,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 3, 0,"noackpolicy" },
	{ IEEE80211_WMMPARAMS_NOACKPOLICY,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,   "get_noackpolicy" },

	{ IEEE80211_IOCTL_SETPARAM,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0, "setparam" },
	/*
	 * These depends on sub-ioctl support which added in version 12.
	 */
	{ IEEE80211_IOCTL_GETPARAM,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1,"getparam" },

	/* sub-ioctl handlers */
	{ IEEE80211_IOCTL_SETPARAM,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "" },
	{ IEEE80211_IOCTL_GETPARAM,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "" },

	/* sub-ioctl definitions */
	{ IEEE80211_PARAM_AUTHMODE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "authmode" },
	{ IEEE80211_PARAM_AUTHMODE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_authmode" },
	{ IEEE80211_PARAM_PROTMODE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "protmode" },
	{ IEEE80211_PARAM_PROTMODE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_protmode" },
	{ IEEE80211_PARAM_PROTMODE_RSSI,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "protrssi" },
	{ IEEE80211_PARAM_PROTMODE_RSSI,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_protrssi" },
	{ IEEE80211_PARAM_PROTMODE_TIMEOUT,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "prottime" },
	{ IEEE80211_PARAM_PROTMODE_TIMEOUT,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_prottime" },
	{ IEEE80211_PARAM_MCASTCIPHER,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "mcastcipher" },
	{ IEEE80211_PARAM_MCASTCIPHER,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_mcastcipher" },
	{ IEEE80211_PARAM_MCASTKEYLEN,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "mcastkeylen" },
	{ IEEE80211_PARAM_MCASTKEYLEN,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_mcastkeylen" },
	{ IEEE80211_PARAM_UCASTCIPHERS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "ucastciphers" },
	{ IEEE80211_PARAM_UCASTCIPHERS,
	/*
	 * NB: can't use "get_ucastciphers" due to iwpriv command names
	 *     must be <IFNAMESIZ which is 16.
	 */
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_uciphers" },
	{ IEEE80211_PARAM_UCASTCIPHER,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "ucastcipher" },
	{ IEEE80211_PARAM_UCASTCIPHER,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_ucastcipher" },
	{ IEEE80211_PARAM_UCASTKEYLEN,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "ucastkeylen" },
	{ IEEE80211_PARAM_UCASTKEYLEN,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_ucastkeylen" },
	{ IEEE80211_PARAM_KEYMGTALGS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "keymgtalgs" },
	{ IEEE80211_PARAM_KEYMGTALGS,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_keymgtalgs" },
	{ IEEE80211_PARAM_RSNCAPS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "rsncaps" },
	{ IEEE80211_PARAM_RSNCAPS,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_rsncaps" },
	{ IEEE80211_PARAM_ROAMING,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "hostroaming" },
	{ IEEE80211_PARAM_ROAMING,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_hostroaming" },
	{ IEEE80211_PARAM_PRIVACY,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "privacy" },
	{ IEEE80211_PARAM_PRIVACY,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_privacy" },
	{ IEEE80211_PARAM_COUNTERMEASURES,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "countermeasures" },
	{ IEEE80211_PARAM_COUNTERMEASURES,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_countermeas" },
	{ IEEE80211_PARAM_DROPUNENCRYPTED,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "dropunencrypted" },
	{ IEEE80211_PARAM_DROPUNENCRYPTED,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_dropunencry" },
	{ IEEE80211_PARAM_WPA,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "wpa" },
	{ IEEE80211_PARAM_WPA,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_wpa" },
	{ IEEE80211_PARAM_DRIVER_CAPS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "driver_caps" },
	{ IEEE80211_PARAM_DRIVER_CAPS,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_driver_caps" },
	{ IEEE80211_PARAM_MACCMD,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "maccmd" },
	{ IEEE80211_PARAM_WMM,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "wmm" },
	{ IEEE80211_PARAM_WMM,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_wmm" },
	{ IEEE80211_PARAM_HIDESSID,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "hide_ssid" },
	{ IEEE80211_PARAM_HIDESSID,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_hide_ssid" },
	{ IEEE80211_PARAM_APBRIDGE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "ap_bridge" },
	{ IEEE80211_PARAM_APBRIDGE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_ap_bridge" },
	{ IEEE80211_PARAM_INACT,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "inact" },
	{ IEEE80211_PARAM_INACT,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_inact" },
	{ IEEE80211_PARAM_INACT_AUTH,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "inact_auth" },
	{ IEEE80211_PARAM_INACT_AUTH,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_inact_auth" },
	{ IEEE80211_PARAM_INACT_INIT,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "inact_init" },
	{ IEEE80211_PARAM_INACT_INIT,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_inact_init" },
	{ IEEE80211_PARAM_ABOLT,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "abolt" },
	{ IEEE80211_PARAM_ABOLT,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_abolt" },
	{ IEEE80211_PARAM_DTIM_PERIOD,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "dtim_period" },
	{ IEEE80211_PARAM_DTIM_PERIOD,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_dtim_period" },
	{IEEE80211_PARAM_RTP_PRINTS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "rtp_prints"},
	/* XXX bintval chosen to avoid 16-char limit */
	{ IEEE80211_PARAM_BEACON_INTERVAL,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_bintval" },
	{IEEE80211_PARAM_AMPDU,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "ampdu_set"},
	{IEEE80211_PARAM_AMPDU,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_ampdu"},
	{IEEE80211_PARAM_AMPDU_LIMIT,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_ampdu_limit"},
	{IEEE80211_PARAM_AMPDU_LIMIT,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_ampdu_limit"},
	{IEEE80211_PARAM_AMPDU_DENSITY,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_ampdu_dens"},
	{IEEE80211_PARAM_AMPDU_DENSITY,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_ampdu_dens"},
	{IEEE80211_PARAM_AMSDU,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "amsdu_set"},
	{IEEE80211_PARAM_AMSDU,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_amsdu"},
	{IEEE80211_PARAM_AMSDU_LIMIT,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_amsdu_limit"},
	{IEEE80211_PARAM_AMSDU_LIMIT,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_ampsu_limit"},
	{IEEE80211_PARAM_SMPS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_smps"},
	{IEEE80211_PARAM_SMPS,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_smps"},
	{IEEE80211_PARAM_HTCONF,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_htconf"},
	{IEEE80211_PARAM_HTCONF,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_htconf"},
	{IEEE80211_PARAM_HTPROTMODE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_htprotection"},
	{IEEE80211_PARAM_HTPROTMODE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_htprotection"},
	{IEEE80211_PARAM_DOTD,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_80211d"},
	{IEEE80211_PARAM_DOTD,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_80211d"},
	{IEEE80211_PARAM_DFS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_dfs"},
	{IEEE80211_PARAM_DFS,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_dfs"},
	{IEEE80211_PARAM_TSN,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_tsn"},
	{IEEE80211_PARAM_TSN,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_tsn"},
	{IEEE80211_PARAM_WPS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_wps"},
	{IEEE80211_PARAM_WPS,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_wps"},
	{IEEE80211_PARAM_HTCOMPAT,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_htcompat"},
	{IEEE80211_PARAM_HTCOMPAT,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_htcompat"},
	{IEEE80211_PARAM_PUREN,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_pure11n"},
	{IEEE80211_PARAM_PUREN,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_pure11n"},
	{IEEE80211_PARAM_TXPOWER,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_txpower"},
	{IEEE80211_PARAM_TXPOWER,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_txpower"},
	
	{ IEEE80211_PARAM_BEACON_MISS_THRESH_MS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "bmiss_ms" },
	{ IEEE80211_PARAM_BEACON_MISS_THRESH_MS,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_bmiss_ms" },
	{ IEEE80211_PARAM_BEACON_MISS_THRESH,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "bmiss" },
	{ IEEE80211_PARAM_BEACON_MISS_THRESH,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_bmiss" },
	{ IEEE80211_PARAM_DOTH,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "doth" },
	{ IEEE80211_PARAM_DOTH,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_doth" },
	{ IEEE80211_PARAM_PWRTARGET,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "doth_pwrtgt" },
	{ IEEE80211_PARAM_PWRTARGET,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_doth_pwrtgt" },
	{ IEEE80211_PARAM_GENREASSOC,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "doth_reassoc" },
	{ IEEE80211_PARAM_DOTH_ALGORITHM,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "doth_algo" },
	{ IEEE80211_PARAM_DOTH_ALGORITHM,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_doth_algo" },
	{ IEEE80211_PARAM_DOTH_MINCOM,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "doth_mincom" },
	{ IEEE80211_PARAM_DOTH_MINCOM,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_doth_mincom" },
	{ IEEE80211_PARAM_DOTH_SLCG,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "doth_slcg" },
	{ IEEE80211_PARAM_DOTH_SLCG,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_doth_slcg" },
	{ IEEE80211_PARAM_DOTH_SLDG,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "doth_sldg" },
	{ IEEE80211_PARAM_DOTH_SLDG,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_doth_sldg" },
	/* continuous transmission (for regulatory agency testing) */
	{ IEEE80211_PARAM_TXCONT,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "txcont" },
	{ IEEE80211_PARAM_TXCONT,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_txcont" },
	{ IEEE80211_PARAM_TXCONT_RATE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "txcontrate" },
	{ IEEE80211_PARAM_TXCONT_RATE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_txcontrate" },
	{ IEEE80211_PARAM_TXCONT_POWER,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "txcontpower" },
	{ IEEE80211_PARAM_TXCONT_POWER,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_txcontpower" },
	{ IEEE80211_PARAM_DFS_TESTMODE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "dfstestmode" },
	{ IEEE80211_PARAM_DFS_TESTMODE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_dfstestmode" },
	{ IEEE80211_PARAM_DFS_CACTIME,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "dfscactime" },
	{ IEEE80211_PARAM_DFS_CACTIME,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_dfscactime" },
	{ IEEE80211_PARAM_DFS_EXCLPERIOD,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "dfsexcltim" },
	{ IEEE80211_PARAM_DFS_EXCLPERIOD,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_dfsexcltim" },
	{ IEEE80211_PARAM_COMPRESSION,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "compression" },
	{ IEEE80211_PARAM_COMPRESSION, 0,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_compression" },
	{ IEEE80211_PARAM_FF,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "ff" },
	{ IEEE80211_PARAM_FF, 0,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_ff" },
	{ IEEE80211_PARAM_TURBO,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "turbo" },
	{ IEEE80211_PARAM_TURBO, 0,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_turbo" },
	{ IEEE80211_PARAM_XR,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "xr" },
	{ IEEE80211_PARAM_XR, 0,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_xr" },
	{ IEEE80211_PARAM_BURST,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "burst" },
	{ IEEE80211_PARAM_BURST, 0,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_burst" },
	{ IEEE80211_IOCTL_CHANSWITCH,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 2, 0,"doth_chanswitch" },
	{ IEEE80211_PARAM_PUREG,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "pureg" },
	{ IEEE80211_PARAM_PUREG, 0,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_pureg" },
	{ IEEE80211_PARAM_AR,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "ar" },
	{ IEEE80211_PARAM_AR, 0,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_ar" },
	{ IEEE80211_PARAM_WDS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "wds" },
	{ IEEE80211_PARAM_WDS, 0,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_wds" },
	{ IEEE80211_PARAM_BGSCAN,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "bgscan" },
	{ IEEE80211_PARAM_BGSCAN, 0,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_bgscan" },
	{ IEEE80211_PARAM_BGSCAN_IDLE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "bgscanidle" },
	{ IEEE80211_PARAM_BGSCAN_IDLE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_bgscanidle" },
	{ IEEE80211_PARAM_BGSCAN_INTERVAL,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "bgscanintvl" },
	{ IEEE80211_PARAM_BGSCAN_INTERVAL,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_bgscanintvl" },
	{ IEEE80211_PARAM_BGSCAN_THRESH,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "bgscanthr" },
	{ IEEE80211_PARAM_BGSCAN_THRESH,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_bgscanthr" },
	{ IEEE80211_PARAM_MCAST_RATE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "mcast_rate" },
	{ IEEE80211_PARAM_MCAST_RATE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_mcast_rate" },
	{ IEEE80211_PARAM_COVERAGE_CLASS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "coverageclass" },
	{ IEEE80211_PARAM_COVERAGE_CLASS,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_coveragecls" },
	{ IEEE80211_PARAM_COUNTRY_IE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "countryie" },
	{ IEEE80211_PARAM_COUNTRY_IE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_countryie" },

	{ IEEE80211_PARAM_CHANGE_COUNTRY_IE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED| 1,  0, "change_country" }, /* changes country IE */
	{ IEEE80211_PARAM_SHORT_GI,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "short_gi" },       /* sets short GI */
	{ IEEE80211_PARAM_SHORT_GI,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_short_gi" },
	{ IEEE80211_PARAM_RIFS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "rifs" },           /* sets RIFS */
	{ IEEE80211_PARAM_RIFS,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_rifs" },
	{ IEEE80211_PARAM_GREEN_FIELD,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "green_field" },    /* sets Green field */
	{ IEEE80211_PARAM_GREEN_FIELD,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_green_field" },
	{ IEEE80211_PARAM_AP_IOSOLATION,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "ap_isol" },        /* sets ip isolation */
	{ IEEE80211_PARAM_AP_IOSOLATION,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_ap_isol" },

	{ IEEE80211_PARAM_SCANVALID,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "scanvalid" },
	{ IEEE80211_PARAM_SCANVALID,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_scanvalid" },
	{ IEEE80211_PARAM_REGCLASS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "regclass" },
	{ IEEE80211_PARAM_REGCLASS,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_regclass" },
	{ IEEE80211_PARAM_DROPUNENC_EAPOL,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "dropunenceapol" },
	{ IEEE80211_PARAM_DROPUNENC_EAPOL,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_dropunencea" },
	{ IEEE80211_PARAM_SHPREAMBLE,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "shpreamble" },
	{ IEEE80211_PARAM_SHPREAMBLE,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_shpreamble" },
	/*
	 * NB: these should be roamrssi* etc, but iwpriv usurps all
	 *     strings that start with roam!
	 */
	{ IEEE80211_PARAM_ROAM_RSSI_11A,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "rssi11a" },
	{ IEEE80211_PARAM_ROAM_RSSI_11A,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_rssi11a" },
	{ IEEE80211_PARAM_ROAM_RSSI_11B,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "rssi11b" },
	{ IEEE80211_PARAM_ROAM_RSSI_11B,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_rssi11b" },
	{ IEEE80211_PARAM_ROAM_RSSI_11G,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "rssi11g" },
	{ IEEE80211_PARAM_ROAM_RSSI_11G,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_rssi11g" },
	{ IEEE80211_PARAM_ROAM_RATE_11A,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "rate11a" },
	{ IEEE80211_PARAM_ROAM_RATE_11A,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_rate11a" },
	{ IEEE80211_PARAM_ROAM_RATE_11B,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "rate11b" },
	{ IEEE80211_PARAM_ROAM_RATE_11B,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_rate11b" },
	{ IEEE80211_PARAM_ROAM_RATE_11G,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "rate11g" },
	{ IEEE80211_PARAM_ROAM_RATE_11G,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_rate11g" },
	{ IEEE80211_PARAM_RSSI_DIS_THR,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "rssi_disthr" },
	{ IEEE80211_PARAM_RSSI_DIS_THR,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_rssi_disthr" },
	{ IEEE80211_PARAM_RSSI_DIS_COUNT,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "rssi_discnt" },
	{ IEEE80211_PARAM_RSSI_DIS_COUNT,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_rssi_discnt" },
	{ IEEE80211_PARAM_UAPSDINFO,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "uapsd" },
	{ IEEE80211_PARAM_UAPSDINFO,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_uapsd" },
	{ IEEE80211_PARAM_SLEEP,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "sleep" },
	{ IEEE80211_PARAM_SLEEP,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_sleep" },
	{ IEEE80211_PARAM_QOSNULL,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "qosnull" },
	{ IEEE80211_PARAM_PSPOLL,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "pspoll" },
	{ IEEE80211_PARAM_EOSPDROP,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "eospdrop" },
	{ IEEE80211_PARAM_EOSPDROP,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_eospdrop" },
	{ IEEE80211_PARAM_MARKDFS,
	  IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "markdfs" },
	{ IEEE80211_PARAM_MARKDFS,
	  0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_markdfs" },
	{ IEEE80211_IOCTL_SET_APPIEBUF,
	  IW_PRIV_TYPE_APPIEBUF, 0, "setiebuf" },
	{ IEEE80211_IOCTL_GET_APPIEBUF,
	  0, IW_PRIV_TYPE_APPIEBUF, "getiebuf" },
	{ IEEE80211_IOCTL_FILTERFRAME,
	  IW_PRIV_TYPE_FILTER , 0, "setfilter" },
	{IEEE80211_PARAM_MAXRATE,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "maxrate"},
	{IEEE80211_PARAM_MAXRATE,
	 0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_maxrate"},
	{IEEE80211_PARAM_MINRATE,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "minrate"},
	{IEEE80211_PARAM_MINRATE,
	 0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_minrate"},
	{ IEEE80211_IOCTL_SETSCANLIST,
	 IW_PRIV_TYPE_CHAR | 255, 0, "setscanlist"},
	{ IEEE80211_PARAM_WDS_SEP,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "wdssep"},
	{ IEEE80211_PARAM_WDS_SEP,
	 0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_wdssep"},
	{ IEEE80211_PARAM_MAXASSOC,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "maxassoc"},
	{ IEEE80211_PARAM_MAXASSOC,
	 0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_maxassoc"},
	{ IEEE80211_PARAM_PROBEREQ,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "probereq"},
	{ IEEE80211_PARAM_PROBEREQ,
	 0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_probereq"},
	{ IEEE80211_KEEP_ALIVE_PERIOD,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "keep_alive"},
	{ IEEE80211_PARAM_DBG_ZONE,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "set_dbg_zone"},	
	{ IEEE80211_PARAM_DBG_ZONE,
	 0, IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, "get_dbg_zone"},	
	{ IEEE80211_DFS_CHAN_TO_SWITCH,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "dfs_chan_switch"},	
	{ IEEE80211_DEFAULT_MGMT_RATE,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 1, 0, "mgmt_rate"},	
#ifdef IEEE80211K 
	{ IEEE80211_IOCTL_11K_MSRMNT_REQ,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 4, 0, "msrmnt_req"},	
	{ IEEE80211_IOCTL_11K_SM_MSRMNT_REQ,
	 IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 4, 0, "sm_msrmnt_req"},	
  { IEEE80211_IOCTL_11K_MSRMNT_RPT,
	 0,IW_PRIV_TYPE_INT | IW_PRIV_SIZE_FIXED | 0, "get_msrmnt_rpt"},	
#endif
};

#if WIRELESS_EXT >= 17
static struct iw_statistics *ieee80211_iw_getstats(struct net_device *dev)
{
	/* Implementation pending to indicate signal, noise levels, rssi etc */
#if KERNEL_VERSION_BTWN_2_6_(18 ,22)
	struct ieee80211vap *vap = dev->priv;
#else
	struct ieee80211vap *vap = netdev_priv(dev);
#endif

	uint16_t ii = 0;
	uint8_t qual = 0;
	vap->stats.status = ii;
	/* quality = 2 *(rssi (-dbm) + 100) */
 	vap->stats.qual.level = -abs(vap->hal_priv_vap->rssi);
	qual = 2*(-abs(vap->hal_priv_vap->rssi) + 100);	
	if (qual > 80)
		qual = 80;
	vap->stats.qual.qual = qual;
 	vap->stats.qual.noise = 0;	
 	
	if(vap->iv_opmode == IEEE80211_M_HOSTAP) {
		vap->stats.qual.updated = IW_QUAL_LEVEL_UPDATED | IW_QUAL_QUAL_UPDATED | IW_QUAL_DBM | IW_QUAL_LEVEL_INVALID;
	} else {
		vap->stats.qual.updated = IW_QUAL_LEVEL_UPDATED | IW_QUAL_QUAL_UPDATED | IW_QUAL_DBM;
	}
	
 	
	vap->stats.discard.nwid = 0;    	
 	vap->stats.discard.code = 0;    	
	vap->stats.discard.fragment = 0;	
	vap->stats.discard.retries = 0; 	
	vap->stats.discard.misc = 0;    	
 	vap->stats.miss.beacon = 0;	
	return (struct iw_statistics *)&vap->stats;
}
#endif

struct iw_handler_def ieee80211_iw_handler_def = 
{
	.standard = (iw_handler *) ieee80211_handlers,
	.num_standard = ARRAY_SIZE(ieee80211_handlers),
#ifdef CONFIG_WEXT_PRIV
	.private = (iw_handler *) ieee80211_priv_handlers,
	.num_private = ARRAY_SIZE(ieee80211_priv_handlers),
	.private_args = (struct iw_priv_args *) ieee80211_priv_args,
	.num_private_args = ARRAY_SIZE(ieee80211_priv_args),
#endif
#if WIRELESS_EXT >= 17
	.get_wireless_stats	= ieee80211_iw_getstats,
#endif
};

int ishtrate(struct ieee80211com *ic, int ucastrate)
{
	struct ieee80211_htrateset *rates = 
	(struct ieee80211_htrateset *)ieee80211_get_suphtrates(ic,(const struct ieee80211_channel *)ic->ic_curchan); /* Jst some channel */
	int ii = 0;

	if (ucastrate == IEEE80211_FIXED_RATE_NONE)
	{
		return 1;
	}
	for (ii = 0; ii < rates->rs_nrates; ii++) 
	{
		if (ucastrate == ieee80211_htrates[rates->rs_rates[ii]].ht20_rate_800ns)
		{
			return 1;
		}
	}
	return 0;
}

int check_band(int mode)
{
	if ((mode == IEEE80211_MODE_11A) || 
	   (mode == IEEE80211_MODE_11NA) || 
	   (mode == IEEE80211_MODE_TURBO_A))
	{
		return BAND_5GHZ;
	}
	return BAND_2_4_GHZ;
}

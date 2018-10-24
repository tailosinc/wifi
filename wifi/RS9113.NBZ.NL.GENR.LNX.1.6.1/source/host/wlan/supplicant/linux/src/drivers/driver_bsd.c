/*
 * WPA Supplicant - driver interaction with BSD net80211 layer
 * Copyright (c) 2004, Sam Leffler <sam@errno.com>
 * Copyright (c) 2004, 2Wire, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Alternatively, this software may be distributed under the terms of BSD
 * license.
 *
 * See README and COPYING for more details.
 */

#include "includes.h"
#include <sys/ioctl.h>
#include <sys/sysctl.h>
#include <sys/stat.h>
#include <net/if_arp.h>

#include "common.h"
#include "driver.h"
#include "eloop.h"
#include "common/ieee802_11_defs.h"
#include "priv_netlink.h"

#include "if.h"
#include "if_media.h"
#if 0
#ifdef __NetBSD__
#include <net/if_ether.h>
#else
#include <net/ethernet.h>
#endif
#include <net/route.h>

#ifdef __DragonFly__
#include <netproto/802_11/ieee80211_ioctl.h>
#include <netproto/802_11/ieee80211_dragonfly.h>
#else /* __DragonFly__ */
#ifdef __GLIBC__
#include <netinet/ether.h>
#endif /* __GLIBC__ */
#endif /* __DragonFly__ || __GLIBC__ */
#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
#include <net80211/ieee80211_freebsd.h>
#endif
#if __NetBSD__
#include <net80211/ieee80211_netbsd.h>
#endif
#endif
#include "ieee80211.h"
#include "ieee80211_ioctl.h"
#include "ieee80211_crypto.h"
#include "ioctl.h"
#include "l2_packet/l2_packet.h"
#include "wireless_copy.h"

/* These values should be equal with what are being used in driver for encoding */
#define DRIVER_EVENT_REMAIN_ON_CHANNEL        0
#define DRIVER_EVENT_CANCEL_REMAIN_ON_CHANNEL 1
#define DRIVER_EVENT_MGMT                     2
#define DRIVER_EVENT_ASSOC                    3
#ifdef RSI_CCX
#define DRIVER_EVENT_ASSOC_RESP_CCKM          4
#endif
static int
bsd_ctrl_iface(void *priv, int enable);
#define IEEE80211_IOC_APPIE 95
#define IEEE80211_IOC_WPS 96

struct bsd_driver_data {
	struct hostapd_data *hapd;	/* back pointer */

	int	sock;			/* open socket for 802.11 ioctls */
	struct l2_packet_data *sock_xmit;/* raw packet xmit socket */
	int	route;			/* routing socket for events */
	char	ifname[IFNAMSIZ+1];	/* interface name */
	unsigned int ifindex;		/* interface index */
	void	*ctx;
	struct wpa_driver_capa capa;	/* driver capability */
	int	is_ap;			/* Access point mode */
	int	prev_roaming;	/* roaming state to restore on deinit */
	int	prev_privacy;	/* privacy state to restore on deinit */
	int	prev_wpa;	/* wpa state to restore on deinit */
	enum ieee80211_opmode opmode;	/* operation mode */
	char	*event_buf;
	size_t	event_buf_len;
	unsigned char acct_mac[ETH_ALEN];
	int we_version;
	int mode_11j;
};

/* Generic functions for hostapd and wpa_supplicant */

#define IEEE80211_APPIE_WPA \
  (IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_BEACON | \
   IEEE80211_FC0_SUBTYPE_PROBE_RESP)
static int
bsd_set80211(void *priv, int op, int val, const void *arg, int arg_len)
{
	struct bsd_driver_data *drv = priv;
	struct ieee80211req ireq;
	char mode_11j;

	os_memset(&ireq, 0, sizeof(ireq));
	os_strlcpy(ireq.i_name, drv->ifname, sizeof(ireq.i_name));
	ireq.i_type = op;
	ireq.i_val = val;
	ireq.i_data = (void *) arg;
	ireq.i_len = arg_len;

	if(op == IEEE80211_IOC_CHANNEL )
	{
		mode_11j = drv->mode_11j;
		ireq.i_data = &mode_11j;
	}

		wpa_printf(MSG_DEBUG,"IOCTL: %s %d ioctl_op = %d\n", __func__, __LINE__, ireq.i_type);
	if (ioctl(drv->sock, SIOCS80211, &ireq) < 0) {
		wpa_printf(MSG_ERROR, "ioctl[SIOCS80211, op=%u, val=%u, "
			   "arg_len=%u]: %s", op, val, arg_len,
			   strerror(errno));
		return -1;
	}
	return 0;
}

static int
bsd_get80211(void *priv, struct ieee80211req *ireq, int op, void *arg,
	     int arg_len)
{
	struct bsd_driver_data *drv = priv;

	os_memset(ireq, 0, sizeof(*ireq));
	os_strlcpy(ireq->i_name, drv->ifname, sizeof(ireq->i_name));
	ireq->i_type = op;
	ireq->i_len = arg_len;
	ireq->i_data = arg;

	if (ioctl(drv->sock, SIOCG80211, ireq) < 0) {
		wpa_printf(MSG_ERROR, "ioctl[SIOCG80211, op=%u, "
			   "arg_len=%u]: %s", op, arg_len, strerror(errno));
		return -1;
	}
	return 0;
}

static int
get80211var(struct bsd_driver_data *drv, int op, void *arg, int arg_len)
{
	struct ieee80211req ireq;

	if (bsd_get80211(drv, &ireq, op, arg, arg_len) < 0)
		return -1;
	return ireq.i_len;
}

static int
set80211var(struct bsd_driver_data *drv, int op, const void *arg, int arg_len)
{
	return bsd_set80211(drv, op, 0, arg, arg_len);
}

static int
set80211param(struct bsd_driver_data *drv, int op, int arg)
{
	return bsd_set80211(drv, op, arg, NULL, 0);
}

static int
bsd_get_ssid(void *priv, u8 *ssid, int len)
{
	struct bsd_driver_data *drv = priv;
#ifdef SIOCG80211NWID
	struct ieee80211_nwid nwid;
	struct ifreq ifr;

	os_memset(&ifr, 0, sizeof(ifr));
	os_strlcpy(ifr.ifr_name, drv->ifname, sizeof(ifr.ifr_name));
	ifr.ifr_data = (void *)&nwid;
	if (ioctl(drv->sock, SIOCG80211NWID, &ifr) < 0 ||
	    nwid.i_len > IEEE80211_NWID_LEN)
		return -1;
	os_memcpy(ssid, nwid.i_nwid, nwid.i_len);
	return nwid.i_len;
#else
	return get80211var(drv, IEEE80211_IOC_SSID, ssid, IEEE80211_NWID_LEN);
#endif
}

static int
bsd_set_ssid(void *priv, const u8 *ssid, int ssid_len)
{
	struct bsd_driver_data *drv = priv;
#ifdef SIOCS80211NWID
	struct ieee80211_nwid nwid;
	struct ifreq ifr;

	os_memcpy(nwid.i_nwid, ssid, ssid_len);
	nwid.i_len = ssid_len;
	os_memset(&ifr, 0, sizeof(ifr));
	os_strlcpy(ifr.ifr_name, drv->ifname, sizeof(ifr.ifr_name));
	ifr.ifr_data = (void *)&nwid;
		wpa_printf(MSG_DEBUG,"IOCTL: %s %d set_ssid\n", __func__, __LINE__);
	return ioctl(drv->sock, SIOCS80211NWID, &ifr);
#else
	return set80211var(drv, IEEE80211_IOC_SSID, ssid, ssid_len);
#endif
}

static int
bsd_get_if_media(void *priv)
{
	struct bsd_driver_data *drv = priv;
	struct ifmediareq ifmr;

	os_memset(&ifmr, 0, sizeof(ifmr));
	os_strlcpy(ifmr.ifm_name, drv->ifname, sizeof(ifmr.ifm_name));

	if (ioctl(drv->sock, SIOCGIFMEDIA, &ifmr) < 0) {
		wpa_printf(MSG_ERROR, "%s: SIOCGIFMEDIA %s", __func__,
			   strerror(errno));
		return -1;
	}

	return ifmr.ifm_current;
}

static int
bsd_set_if_media(void *priv, int media)
{
	struct bsd_driver_data *drv = priv;
	struct ifreq ifr;

	os_memset(&ifr, 0, sizeof(ifr));
	os_strlcpy(ifr.ifr_name, drv->ifname, sizeof(ifr.ifr_name));
	ifr.ifr_ifru.ifru_ivalue = media;

	if (ioctl(drv->sock, SIOCSIFMEDIA, &ifr) < 0) {
		wpa_printf(MSG_ERROR, "%s: SIOCSIFMEDIA %s", __func__,
			   strerror(errno));
		return -1;
	}

	return 0;
}

static int
bsd_set_mediaopt(void *priv, uint32_t mask, uint32_t mode)
{
	int media = bsd_get_if_media(priv);

	if (media < 0)
		return -1;
	media &= ~mask;
	media |= mode;
	if (bsd_set_if_media(priv, media) < 0)
		return -1;
	return 0;
}

static int
bsd_del_key(void *priv, const u8 *addr, int key_idx)
{
	struct ieee80211req_del_key wk;

	os_memset(&wk, 0, sizeof(wk));
	if (addr == NULL) {
		wpa_printf(MSG_DEBUG, "%s: key_idx=%d", __func__, key_idx);
		wk.idk_keyix = key_idx;
	} else {
		wpa_printf(MSG_DEBUG, "%s: addr=" MACSTR, __func__,
			   MAC2STR(addr));
		os_memcpy(wk.idk_macaddr, addr, IEEE80211_ADDR_LEN);
		wk.idk_keyix = (u_int8_t) IEEE80211_KEYIX_NONE;	/* XXX */
	}

	return set80211var(priv, IEEE80211_IOC_DELKEY, &wk, sizeof(wk));
}

static int
bsd_send_mlme_param(void *priv, const u8 op, const u16 reason, const u8 *addr)
{
	struct ieee80211req_mlme mlme;

	os_memset(&mlme, 0, sizeof(mlme));
	mlme.im_op = op;
	mlme.im_reason = reason;
	os_memcpy(mlme.im_macaddr, addr, IEEE80211_ADDR_LEN);
	return set80211var(priv, IEEE80211_IOC_MLME, &mlme, sizeof(mlme));
}

static int
bsd_ctrl_iface(void *priv, int enable)
{
	struct bsd_driver_data *drv = priv;
	struct ifreq ifr;

	os_memset(&ifr, 0, sizeof(ifr));
	os_strlcpy(ifr.ifr_name, drv->ifname, sizeof(ifr.ifr_name));

	wpa_printf(MSG_DEBUG,"IOCTL: %s %d enable = %d\n", __func__, __LINE__, enable);
	if (ioctl(drv->sock, SIOCGIFFLAGS, &ifr) < 0) {
		perror("ioctl[SIOCGIFFLAGS]");
		return -1;
	}

	if (enable) {
		if (ifr.ifr_flags & IFF_UP)
			return 0;
		ifr.ifr_flags |= IFF_UP;
	} else {
		if (!(ifr.ifr_flags & IFF_UP))
			return 0;
		ifr.ifr_flags &= ~IFF_UP;
	}

	if (ioctl(drv->sock, SIOCSIFFLAGS, &ifr) < 0) {
		perror("ioctl[SIOCSIFFLAGS]");
		return -1;
	}

	return 0;
}

static int
bsd_set_key(const char *ifname, void *priv, enum wpa_alg alg,
	    const unsigned char *addr, int key_idx, int set_tx, const u8 *seq,
	    size_t seq_len, const u8 *key, size_t key_len)
{
	struct ieee80211req_key wk;
#ifdef IEEE80211_KEY_NOREPLAY
	struct bsd_driver_data *drv = priv;
#endif /* IEEE80211_KEY_NOREPLAY */

	wpa_printf(MSG_DEBUG, "%s: alg=%d addr=%p key_idx=%d set_tx=%d "
		   "seq_len=%zu key_len=%zu", __func__, alg, addr, key_idx,
		   set_tx, seq_len, key_len);

	if (alg == WPA_ALG_NONE) {
#ifndef HOSTAPD
		if (addr == NULL ||
		    os_memcmp(addr, "\xff\xff\xff\xff\xff\xff",
			      IEEE80211_ADDR_LEN) == 0)
			return bsd_del_key(priv, NULL, key_idx);
		else
#endif /* HOSTAPD */
			return bsd_del_key(priv, addr, key_idx);
	}

	os_memset(&wk, 0, sizeof(wk));
	switch (alg) {
	case WPA_ALG_WEP:
		wk.ik_type = IEEE80211_CIPHER_WEP;
		break;
	case WPA_ALG_TKIP:
		wk.ik_type = IEEE80211_CIPHER_TKIP;
		break;
	case WPA_ALG_CCMP:
		wk.ik_type = IEEE80211_CIPHER_AES_CCM;
		break;
#ifdef CONFIG_IEEE80211W
	case WPA_ALG_IGTK:
		wk.ik_type = IEEE80211_CIPHER_AES_CMAC;
		break;
#endif
	default:
		wpa_printf(MSG_ERROR, "%s: unknown alg=%d", __func__, alg);
		return -1;
	}

	wk.ik_flags = IEEE80211_KEY_RECV;
	if (set_tx)
		wk.ik_flags |= IEEE80211_KEY_XMIT;

	if (addr == NULL) {
		os_memset(wk.ik_macaddr, 0xff, IEEE80211_ADDR_LEN);
		wk.ik_keyix = key_idx;
	} else {
		os_memcpy(wk.ik_macaddr, addr, IEEE80211_ADDR_LEN);
	}
		/*
		 * Deduce whether group/global or unicast key by checking
		 * the address (yech).  Note also that we can only mark global
		 * keys default; doing this for a unicast key is an error.
		 */
		if (os_memcmp(wk.ik_macaddr, "\xff\xff\xff\xff\xff\xff",
			      IEEE80211_ADDR_LEN) == 0) {
			wk.ik_flags |= IEEE80211_KEY_GROUP;
			wk.ik_keyix = key_idx;
		} else {
			wk.ik_keyix = key_idx == 0 ? IEEE80211_KEYIX_NONE :
				key_idx;
		}
	if (wk.ik_keyix != IEEE80211_KEYIX_NONE && set_tx)
		wk.ik_flags |= IEEE80211_KEY_DEFAULT;
#ifndef HOSTAPD
#ifdef IEEE80211_KEY_NOREPLAY
	/*
	 * Ignore replay failures in IBSS and AHDEMO mode.
	 */
	if (drv->opmode == IEEE80211_M_IBSS ||
	    drv->opmode == IEEE80211_M_AHDEMO)
		wk.ik_flags |= IEEE80211_KEY_NOREPLAY;
#endif /* IEEE80211_KEY_NOREPLAY */
#endif /* HOSTAPD */
	wk.ik_keylen = key_len;
	os_memcpy(&wk.ik_keyrsc, seq, seq_len);
	os_memcpy(wk.ik_keydata, key, key_len);

	return set80211var(priv, IEEE80211_IOC_WPAKEY, &wk, sizeof(wk));
}

static int
bsd_configure_wpa(void *priv, struct wpa_bss_params *params)
{
#ifndef IEEE80211_IOC_APPIE
	static const char *ciphernames[] =
		{ "WEP", "TKIP", "AES-OCB", "AES-CCM", "CKIP", "NONE" };
	int v;

	switch (params->wpa_group) {
	case WPA_CIPHER_CCMP:
		v = IEEE80211_CIPHER_AES_CCM;
		break;
	case WPA_CIPHER_TKIP:
		v = IEEE80211_CIPHER_TKIP;
		break;
	case WPA_CIPHER_WEP104:
		v = IEEE80211_CIPHER_WEP;
		break;
	case WPA_CIPHER_WEP40:
		v = IEEE80211_CIPHER_WEP;
		break;
	case WPA_CIPHER_NONE:
		v = IEEE80211_CIPHER_NONE;
		break;
	default:
		wpa_printf(MSG_ERROR, "Unknown group key cipher %u\n",
				params->wpa_group);
		return -1;
	}
	wpa_printf(MSG_DEBUG, "%s: group key cipher=%s (%u)",
		   __func__, ciphernames[v], v);
	if (set80211param(priv, IEEE80211_IOC_MCASTCIPHER, v)) {
		wpa_printf(MSG_DEBUG, "Unable to set group key cipher to %u (%s)\n",
			v, ciphernames[v]);
		return -1;
	}
	if (v == IEEE80211_CIPHER_WEP) {
		/* key length is done only for specific ciphers */
		v = (params->wpa_group == WPA_CIPHER_WEP104 ? 13 : 5);
		if (set80211param(priv, IEEE80211_IOC_MCASTKEYLEN, v)) {
			wpa_printf(MSG_DEBUG, "Unable to set group key length to %u\n", v);
			return -1;
		}
	}

	v = 0;
	if (params->wpa_pairwise & WPA_CIPHER_CCMP)
		v |= 1<<IEEE80211_CIPHER_AES_CCM;
	if (params->wpa_pairwise & WPA_CIPHER_TKIP)
		v |= 1<<IEEE80211_CIPHER_TKIP;
	if (params->wpa_pairwise & WPA_CIPHER_NONE)
		v |= 1<<IEEE80211_CIPHER_NONE;
	wpa_printf(MSG_DEBUG, "%s: pairwise key ciphers=0x%x", __func__, v);
	if (set80211param(priv, IEEE80211_IOC_UCASTCIPHERS, v)) {
		wpa_printf(MSG_DEBUG, "Unable to set pairwise key ciphers to 0x%x\n", v);
		return -1;
	}

	wpa_printf(MSG_DEBUG, "%s: key management algorithms=0x%x",
		   __func__, params->wpa_key_mgmt);
	if (set80211param(priv, IEEE80211_IOC_KEYMGTALGS,
			  params->wpa_key_mgmt)) {
		wpa_printf(MSG_DEBUG, "Unable to set key management algorithms to 0x%x\n",
			params->wpa_key_mgmt);
		return -1;
	}

	v = 0;
	if (params->rsn_preauth)
		v |= BIT(0);
	wpa_printf(MSG_DEBUG, "%s: rsn capabilities=0x%x",
		   __func__, params->rsn_preauth);
	if (set80211param(priv, IEEE80211_IOC_RSNCAPS, v)) {
		wpa_printf(MSG_DEBUG,"Unable to set RSN capabilities to 0x%x\n", v);
		return -1;
	}

#ifdef CONFIG_IEEE80211W
	if (params->ieee80211w != NO_MGMT_FRAME_PROTECTION) {
		v |= BIT(7);
		if (params->ieee80211w == MGMT_FRAME_PROTECTION_REQUIRED)
			v |= BIT(6);
	}
#endif /* CONFIG_IEEE80211W */

#endif /* IEEE80211_IOC_APPIE */

	wpa_printf(MSG_DEBUG, "%s: enable WPA= 0x%x", __func__, params->wpa);
	if (set80211param(priv, IEEE80211_IOC_WPA, params->wpa)) {
		wpa_printf(MSG_DEBUG, "Unable to set WPA to %u\n", params->wpa);
		return -1;
	}
	return 0;
}

static int
bsd_set_ieee8021x(void *priv, struct wpa_bss_params *params)
{
	wpa_printf(MSG_DEBUG, "%s: enabled=%d", __func__, params->enabled);

	if (!params->enabled) {
		/* XXX restore state */
		return set80211param(priv, IEEE80211_IOC_AUTHMODE,
				     IEEE80211_AUTH_AUTO);
	}
	if (!params->wpa && !params->ieee802_1x) {
		wpa_printf(MSG_ERROR, "%s: No 802.1X or WPA enabled",
			   __func__);
		return -1;
	}
	if (params->wpa && bsd_configure_wpa(priv, params) != 0) {
		wpa_printf(MSG_ERROR, "%s: Failed to configure WPA state",
			   __func__);
		return -1;
	}
	if (set80211param(priv, IEEE80211_IOC_AUTHMODE,
		(params->wpa ? IEEE80211_AUTH_WPA : IEEE80211_AUTH_8021X))) {
		wpa_printf(MSG_ERROR, "%s: Failed to enable WPA/802.1X",
			   __func__);
		return -1;
	}
	return 0;
	//bsd_ctrl_iface(priv, 1);
}

#if 1

static void
bsd_new_sta(void *priv, void *ctx, u8 addr[IEEE80211_ADDR_LEN])
{
	struct ieee80211req_wpaie ie;
	int ielen = 0;
	u8 *iebuf = NULL;

	/*
	 * Fetch and validate any negotiated WPA/RSN parameters.
	 */
	memset(&ie, 0, sizeof(ie));
	memcpy(ie.wpa_macaddr, addr, IEEE80211_ADDR_LEN);
	if (get80211var(priv, IEEE80211_IOC_WPAIE, &ie, sizeof(ie)) < 0) {
		wpa_printf(MSG_DEBUG, "Failed to get WPA/RSN information element.\n");
		goto no_ie;
	}
	iebuf = ie.wpa_ie;
	ielen = ie.wpa_ie[1];
	if (ielen == 0)
		iebuf = NULL;
	else
		ielen += 2;

no_ie:
	drv_event_assoc(ctx, addr, iebuf, ielen, 0);
}

static int
bsd_send_eapol(void *priv, const u8 *addr, const u8 *data, size_t data_len,
	       int encrypt, const u8 *own_addr, u32 flags)
{
	struct bsd_driver_data *drv = priv;

	wpa_hexdump(MSG_MSGDUMP, "TX EAPOL", data, data_len);

	return l2_packet_send(drv->sock_xmit, addr, ETH_P_EAPOL, data,
			      data_len);
}
#endif

static int
bsd_set_freq(void *priv, u16 channel)
{
	struct bsd_driver_data *drv = priv;
#ifdef SIOCS80211CHANNEL
	struct ieee80211chanreq creq;
#endif /* SIOCS80211CHANNEL */
	u32 mode, ret;

	if( drv->mode_11j )
			mode = IFM_IEEE80211_11NA;
	else if (channel > 0 && channel < 14)
		mode = IFM_IEEE80211_11NG;
	else if (channel == 0)
		mode = IFM_AUTO;
	else if (channel == 14)
		mode = IFM_IEEE80211_11B;
	else
		mode = IFM_IEEE80211_11NA;
	if (bsd_set_mediaopt(drv, IFM_MMASK, mode) < 0) {
		wpa_printf(MSG_ERROR, "%s: failed to set modulation mode",
			   __func__);
		return -1;
	}

#ifdef SIOCS80211CHANNEL
	os_memset(&creq, 0, sizeof(creq));
	os_strlcpy(creq.i_name, drv->ifname, sizeof(creq.i_name));
	creq.i_channel = channel;
	return ioctl(drv->sock, SIOCS80211CHANNEL, &creq);
#else /* SIOCS80211CHANNEL */
    {
	  struct iwreq ireq;
	  char str[6];
	  os_memset(&ireq, 0, sizeof(ireq));
	  os_strlcpy(ireq.ifr_name, drv->ifname, sizeof(ireq.ifr_name));
	  if (mode == IFM_IEEE80211_11B)
		strcpy(str, "11B");
	  else if (mode == IFM_IEEE80211_11NG)
		strcpy(str, "11N");
	  else if (mode == IFM_AUTO)
		strcpy(str, "AUTO");
	  else
		strcpy(str, "11AN");
	  str[4]= '\0';
	  ireq.u.data.pointer = str;
	  ireq.u.data.length  = strlen(str);
#define SIOCSISETMODE 0x8Be0 + 0x02
	  ioctl(drv->sock, SIOCSISETMODE, &ireq);
	  ret = set80211param(priv, IEEE80211_IOC_CHANNEL, channel);
	  return ret;
	}
#endif /* SIOCS80211CHANNEL */
}

static int
bsd_set_opt_ie(void *priv, const u8 *ie, size_t ie_len)
{
#ifdef IEEE80211_IOC_APPIE
	wpa_printf(MSG_DEBUG, "%s: set WPA+RSN ie (len %lu)", __func__,
		   (unsigned long)ie_len);
	return bsd_set80211(priv, IEEE80211_IOC_APPIE, IEEE80211_APPIE_WPA,
			    ie, ie_len);
#endif /* IEEE80211_IOC_APPIE */
	return 0;
}

static size_t
rtbuf_len(void)
{
	size_t len;

	int mib[6] = {CTL_NET, AF_ROUTE, 0, AF_INET, 0, 0};

	if (sysctl(mib, 6, NULL, &len, NULL, 0) < 0) {
		wpa_printf(MSG_WARNING, "%s failed: %s\n", __func__,
			   strerror(errno));
		len = 2048;
	}

	return len;
}

static int
bsd_set_privacy(void *priv, int enabled)
{
	wpa_printf(MSG_DEBUG, "%s: enabled=%d", __func__, enabled);
	return set80211param(priv, IEEE80211_IOC_PRIVACY, enabled);
}

static int
bsd_set_ap_wps_ie(void *priv, const struct wpabuf *beacon,
		      const struct wpabuf *proberesp, const struct wpabuf *assocresp)
{
	if(bsd_set80211(priv, IEEE80211_IOC_WPS, 1, NULL, 0))
	{
		wpa_printf(MSG_DEBUG, "Failed to set IEEE80211_IOC_WPS ioctl\n");
		return -1;
	}
	if(beacon)
	{
		wpa_printf(MSG_DEBUG, "Setting WPS IE FOR BEACON\n");
		if(bsd_set80211(priv, IEEE80211_IOC_APPIE, (IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_BEACON),
						wpabuf_head(beacon), wpabuf_len(beacon)))
			return -1;
	}
	if(proberesp)
	{
		wpa_printf(MSG_DEBUG, "Setting WPS IE FOR PROBE RESP\n");
		if(bsd_set80211(priv, IEEE80211_IOC_APPIE, (IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_PROBE_RESP),
						wpabuf_head(proberesp), wpabuf_len(proberesp)))
			return -1;
	}
	if(assocresp)
	{
		wpa_printf(MSG_DEBUG, "Setting WPS IE FOR PROBE RESP\n");
		return bsd_set80211(priv, IEEE80211_IOC_APPIE, (IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_ASSOC_RESP),
						wpabuf_head(assocresp), wpabuf_len(assocresp));
	}
	return 0;
}

#ifdef HOSTAPD

/*
 * Avoid conflicts with hostapd definitions by undefining couple of defines
 * from net80211 header files.
 */
#undef RSN_VERSION
#undef WPA_VERSION
#undef WPA_OUI_TYPE

static int bsd_sta_deauth(void *priv, const u8 *own_addr, const u8 *addr,
			  int reason_code);

static const char *
ether_sprintf(const u8 *addr)
{
	static char buf[sizeof(MACSTR)];

	if (addr != NULL)
		snprintf(buf, sizeof(buf), MACSTR, MAC2STR(addr));
	else
		snprintf(buf, sizeof(buf), MACSTR, 0,0,0,0,0,0);
	return buf;
}

static int
bsd_set_privacy(void *priv, int enabled)
{
	wpa_printf(MSG_DEBUG, "%s: enabled=%d", __func__, enabled);

	return set80211param(priv, IEEE80211_IOC_PRIVACY, enabled);
}

static int
bsd_get_seqnum(const char *ifname, void *priv, const u8 *addr, int idx,
	       u8 *seq)
{
	struct ieee80211req_key wk;

	wpa_printf(MSG_DEBUG, "%s: addr=%s idx=%d",
		   __func__, ether_sprintf(addr), idx);

	memset(&wk, 0, sizeof(wk));
	if (addr == NULL)
		memset(wk.ik_macaddr, 0xff, IEEE80211_ADDR_LEN);
	else
		memcpy(wk.ik_macaddr, addr, IEEE80211_ADDR_LEN);
	wk.ik_keyix = idx;

	if (get80211var(priv, IEEE80211_IOC_WPAKEY, &wk, sizeof(wk)) < 0) {
		wpa_printf(MSG_DEBUG, "Failed to get encryption.\n");
		return -1;
	}

#ifdef WORDS_BIGENDIAN
	{
		/*
		 * wk.ik_keytsc is in host byte order (big endian), need to
		 * swap it to match with the byte order used in WPA.
		 */
		int i;
		u8 tmp[WPA_KEY_RSC_LEN];
		memcpy(tmp, &wk.ik_keytsc, sizeof(wk.ik_keytsc));
		for (i = 0; i < WPA_KEY_RSC_LEN; i++) {
			seq[i] = tmp[WPA_KEY_RSC_LEN - i - 1];
		}
	}
#else /* WORDS_BIGENDIAN */
	memcpy(seq, &wk.ik_keytsc, sizeof(wk.ik_keytsc));
#endif /* WORDS_BIGENDIAN */
	return 0;
}


static int 
bsd_flush(void *priv)
{
	u8 allsta[IEEE80211_ADDR_LEN];

	memset(allsta, 0xff, IEEE80211_ADDR_LEN);
	return bsd_sta_deauth(priv, NULL, allsta, IEEE80211_REASON_AUTH_LEAVE);
}


static int
bsd_read_sta_driver_data(void *priv, struct hostap_sta_driver_data *data,
			 const u8 *addr)
{
	struct ieee80211req_sta_stats stats;

	memcpy(stats.is_u.macaddr, addr, IEEE80211_ADDR_LEN);
	if (get80211var(priv, IEEE80211_IOC_STA_STATS, &stats, sizeof(stats))
	    > 0) {
		/* XXX? do packets counts include non-data frames? */
		data->rx_packets = stats.is_stats.ns_rx_data;
		data->rx_bytes = stats.is_stats.ns_rx_bytes;
		data->tx_packets = stats.is_stats.ns_tx_data;
		data->tx_bytes = stats.is_stats.ns_tx_bytes;
	}
	return 0;
}

#endif
static int
bsd_sta_deauth(void *priv, const u8 *own_addr, const u8 *addr, int reason_code)
{
	return bsd_send_mlme_param(priv, IEEE80211_MLME_DEAUTH, reason_code,
				   addr);
}

static void
handle_read(void *ctx, const u8 *src_addr, const u8 *buf, size_t len)
{
	struct bsd_driver_data *drv = ctx;
	drv_event_eapol_rx(drv->ctx, src_addr, buf, len);
}

#ifdef HOSTAPD

static int
bsd_sta_disassoc(void *priv, const u8 *own_addr, const u8 *addr,
		 int reason_code)
{
	return bsd_send_mlme_param(priv, IEEE80211_MLME_DISASSOC, reason_code,
				   addr);
}

static void
bsd_wireless_event_receive(int sock, void *ctx, void *sock_ctx)
{
	struct bsd_driver_data *drv = ctx;
	char buf[2048];
	struct if_announcemsghdr *ifan;
	struct rt_msghdr *rtm;
	struct ieee80211_michael_event *mic;
	struct ieee80211_join_event *join;
	struct ieee80211_leave_event *leave;
	int n;
	union wpa_event_data data;


#if 0
	n = read(sock, buf, sizeof(buf));
	if (n < 0) {
		if (errno != EINTR && errno != EAGAIN)
			perror("read(PF_ROUTE)");
		return;
	}

	rtm = (struct rt_msghdr *) buf;
	if (rtm->rtm_version != RTM_VERSION) {
		wpa_printf(MSG_DEBUG, "Routing message version %d not "
			"understood\n", rtm->rtm_version);
		return;
	}
	ifan = (struct if_announcemsghdr *) rtm;
	switch (rtm->rtm_type) {
	case RTM_IEEE80211:
		switch (ifan->ifan_what) {
		case RTM_IEEE80211_ASSOC:
		case RTM_IEEE80211_REASSOC:
		case RTM_IEEE80211_DISASSOC:
		case RTM_IEEE80211_SCAN:
			break;
		case RTM_IEEE80211_LEAVE:
			leave = (struct ieee80211_leave_event *) &ifan[1];
			drv_event_disassoc(drv->hapd, leave->iev_addr);
			break;
		case RTM_IEEE80211_JOIN:
#ifdef RTM_IEEE80211_REJOIN
		case RTM_IEEE80211_REJOIN:
#endif
			join = (struct ieee80211_join_event *) &ifan[1];
			bsd_new_sta(drv, drv->hapd, join->iev_addr);
			break;
		case RTM_IEEE80211_REPLAY:
			/* ignore */
			break;
		case RTM_IEEE80211_MICHAEL:
			mic = (struct ieee80211_michael_event *) &ifan[1];
			wpa_printf(MSG_DEBUG,
				"Michael MIC failure wireless event: "
				"keyix=%u src_addr=" MACSTR, mic->iev_keyix,
				MAC2STR(mic->iev_src));
			os_memset(&data, 0, sizeof(data));
			data.michael_mic_failure.unicast = 1;
			data.michael_mic_failure.src = mic->iev_src;
			wpa_supplicant_event(drv->hapd,
					     EVENT_MICHAEL_MIC_FAILURE, &data);
			break;
		}
		break;
	}
#endif
}

static void *
bsd_init(struct hostapd_data *hapd, struct wpa_init_params *params)
{
	struct bsd_driver_data *drv;

	drv = os_zalloc(sizeof(struct bsd_driver_data));
	if (drv == NULL) {
		wpa_printf(MSG_ERROR, "Could not allocate memory for bsd driver data");
		return NULL;
	}

	drv->event_buf_len = rtbuf_len();

	drv->event_buf = os_malloc(drv->event_buf_len);
	if (drv->event_buf == NULL) {
		wpa_printf(MSG_ERROR, "%s: os_malloc() failed", __func__);
		goto bad;
	}

	drv->hapd = hapd;
	drv->sock = socket(PF_INET, SOCK_DGRAM, 0);
	if (drv->sock < 0) {
		perror("socket[PF_INET,SOCK_DGRAM]");
		goto bad;
	}
	os_strlcpy(drv->ifname, params->ifname, sizeof(drv->ifname));

	drv->sock_xmit = l2_packet_init(drv->ifname, NULL, ETH_P_EAPOL,
					handle_read, drv, 0);
	if (drv->sock_xmit == NULL)
		goto bad;
	if (l2_packet_get_own_addr(drv->sock_xmit, params->own_addr))
		goto bad;

	/* mark down during setup */
	if (bsd_ctrl_iface(drv, 0) < 0)
		goto bad;

	drv->route = socket(PF_ROUTE, SOCK_RAW, 0);
	if (drv->route < 0) {
		perror("socket(PF_ROUTE,SOCK_RAW)");
		goto bad;
	}
	eloop_register_read_sock(drv->route, bsd_wireless_event_receive, drv,
				 NULL);

	if (bsd_set_mediaopt(drv, IFM_OMASK, IFM_IEEE80211_HOSTAP) < 0) {
		wpa_printf(MSG_ERROR, "%s: failed to set operation mode",
			   __func__);
		goto bad;
	}

	return drv;
bad:
	if (drv->sock_xmit != NULL)
		l2_packet_deinit(drv->sock_xmit);
	if (drv->sock >= 0)
		close(drv->sock);
	os_free(drv->event_buf);
	if (drv != NULL)
		os_free(drv);
	return NULL;
}


static void
bsd_deinit(void *priv)
{
	struct bsd_driver_data *drv = priv;

	if (drv->route >= 0) {
		eloop_unregister_read_sock(drv->route);
		close(drv->route);
	}
	bsd_ctrl_iface(drv, 0);
	if (drv->sock >= 0)
		close(drv->sock);
	if (drv->sock_xmit != NULL)
		l2_packet_deinit(drv->sock_xmit);
	os_free(drv->event_buf);
	os_free(drv);
}

#else /* HOSTAPD */
static int 
bsd_set_sta_authorized(void *priv, const u8 *addr,
		       int total_flags, int flags_or, int flags_and)
{
	int authorized = -1;

	/* For now, only support setting Authorized flag */
	if (flags_or & WPA_STA_AUTHORIZED)
		authorized = 1;
	if (!(flags_and & WPA_STA_AUTHORIZED))
		authorized = 0;

	if (authorized < 0)
		return 0;

	return bsd_send_mlme_param(priv, authorized ?
				   IEEE80211_MLME_AUTHORIZE :
				   IEEE80211_MLME_UNAUTHORIZE, 0, addr);
}

static int
get80211param(struct bsd_driver_data *drv, int op)
{
	struct ieee80211req ireq;

	if (bsd_get80211(drv, &ireq, op, NULL, 0) < 0)
		return -1;
	return ireq.i_val;
}

static int
wpa_driver_bsd_get_bssid(void *priv, u8 *bssid)
{
	struct bsd_driver_data *drv = priv;
#ifdef SIOCG80211BSSID
	struct ieee80211_bssid bs;

	os_strlcpy(bs.i_name, drv->ifname, sizeof(bs.i_name));
	if (ioctl(drv->sock, SIOCG80211BSSID, &bs) < 0)
		return -1;
	os_memcpy(bssid, bs.i_bssid, sizeof(bs.i_bssid));
	return 0;
#else
	return get80211var(drv, IEEE80211_IOC_BSSID,
		bssid, IEEE80211_ADDR_LEN) < 0 ? -1 : 0;
#endif
}

static int
wpa_driver_bsd_get_ssid(void *priv, u8 *ssid)
{
	struct bsd_driver_data *drv = priv;
	return bsd_get_ssid(drv, ssid, 0);
}

static int
wpa_driver_bsd_set_wpa_ie(struct bsd_driver_data *drv, const u8 *wpa_ie,
			  size_t wpa_ie_len)
{
#ifdef IEEE80211_IOC_APPIE
	return bsd_set_opt_ie(drv, wpa_ie, wpa_ie_len);
#else /* IEEE80211_IOC_APPIE */
	return set80211var(drv, IEEE80211_IOC_OPTIE, wpa_ie, wpa_ie_len);
#endif /* IEEE80211_IOC_APPIE */
}

static int
wpa_driver_bsd_set_wpa_internal(void *priv, int wpa, int privacy)
{
	int ret = 0;

	wpa_printf(MSG_DEBUG, "%s: wpa=%d privacy=%d",
		__FUNCTION__, wpa, privacy);

	if (!wpa && wpa_driver_bsd_set_wpa_ie(priv, NULL, 0) < 0)
		ret = -1;
	if (set80211param(priv, IEEE80211_IOC_PRIVACY, privacy) < 0)
		ret = -1;
	if (set80211param(priv, IEEE80211_IOC_WPA, wpa) < 0)
		ret = -1;

	return ret;
}

static int
wpa_driver_bsd_set_wpa(void *priv, int enabled)
{
	wpa_printf(MSG_DEBUG, "%s: enabled=%d", __FUNCTION__, enabled);

	return wpa_driver_bsd_set_wpa_internal(priv, enabled ? 3 : 0, enabled);
}

static int
wpa_driver_bsd_set_countermeasures(void *priv, int enabled)
{
	wpa_printf(MSG_DEBUG, "%s: enabled=%d", __func__, enabled);
	return set80211param(priv, IEEE80211_IOC_COUNTERMEASURES, enabled);
}


static int
wpa_driver_bsd_set_drop_unencrypted(void *priv, int enabled)
{
	wpa_printf(MSG_DEBUG, "%s: enabled=%d", __func__, enabled);
	return set80211param(priv, IEEE80211_IOC_DROPUNENCRYPTED, enabled);
}

static int
wpa_driver_bsd_deauthenticate(void *priv, const u8 *addr, int reason_code)
{
	return bsd_send_mlme_param(priv, IEEE80211_MLME_DEAUTH, reason_code,
				   addr);
}

#if 0
static int
wpa_driver_bsd_disassociate(void *priv, const u8 *addr, int reason_code)
{
	return bsd_send_mlme_param(priv, IEEE80211_MLME_DISASSOC, reason_code,
				   addr);
}
#endif

static int
wpa_driver_bsd_set_auth_alg(void *priv, int auth_alg)
{
	int authmode;

	if ((auth_alg & WPA_AUTH_ALG_OPEN) &&
	    (auth_alg & WPA_AUTH_ALG_SHARED))
		authmode = IEEE80211_AUTH_AUTO;
	else if (auth_alg & WPA_AUTH_ALG_SHARED)
		authmode = IEEE80211_AUTH_SHARED;
	else
		authmode = IEEE80211_AUTH_OPEN;

	return set80211param(priv, IEEE80211_IOC_AUTHMODE, authmode);
}

static int
wpa_driver_bsd_associate(void *priv, struct wpa_driver_associate_params *params)
{
	struct bsd_driver_data *drv = priv;
	struct ieee80211req_mlme mlme;
	u32 mode;
	u16 channel;
	int privacy;
	int ret = 0;
#ifdef CONFIG_P2P
	int loop_count = 0;
	int status;
#endif
	wpa_printf(MSG_DEBUG,
		"%s: ssid '%.*s' wpa ie len %u pairwise %u group %u key mgmt %u"
		, __func__
		   , (unsigned int) params->ssid_len, params->ssid
		, (unsigned int) params->wpa_ie_len
		, params->pairwise_suite
		, params->group_suite
		, params->key_mgmt_suite
	);

	printf("%s %d freq = %d mode = %d\n", __func__, __LINE__, params->freq, params->mode);
	switch (params->mode) {
	case IEEE80211_MODE_INFRA:
		mode = 0 /* STA */;
		break;
	case IEEE80211_MODE_IBSS:
		mode = IFM_IEEE80211_IBSS;
		break;
	case IEEE80211_MODE_AP:
	case IEEE80211_MODE_P2P_GO:
		mode = IFM_IEEE80211_HOSTAP;
		break;
	default:
		wpa_printf(MSG_ERROR, "%s: unknown operation mode", __func__);
		return -1;
	}

#ifdef CONFIG_P2P
	if ((drv->capa.flags & WPA_DRIVER_FLAGS_P2P_CAPABLE) && (mode == IFM_IEEE80211_HOSTAP)) {

		/* Bring the vap to INIT state before changing it to IEEEE80211_M_HOSTAP mode */
		os_memset(&mlme, 0, sizeof(mlme));
		mlme.im_op = IEEE80211_MLME_DEAUTH;
		/* If already in HOSTAP mode, this may return null value. Hence not caring for return value */
		set80211var(drv, IEEE80211_IOC_MLME, &mlme, sizeof(mlme));

		/* Reset all app ie's */
		bsd_set80211(priv, IEEE80211_IOC_APPIE, (IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_PROBE_REQ), NULL, 0);
		bsd_set80211(priv, IEEE80211_IOC_APPIE, (IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_PROBE_RESP), NULL, 0);
		bsd_set80211(priv, IEEE80211_IOC_APPIE, (IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_BEACON), NULL, 0);
		bsd_set80211(priv, IEEE80211_IOC_APPIE, (IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_ASSOC_REQ), NULL, 0);
		bsd_set80211(priv, IEEE80211_IOC_APPIE, (IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_ASSOC_RESP), NULL, 0);

		status = set80211param(priv, IEEE80211_IOC_CHANGE_VAP_MODE, mode);
		wpa_printf(MSG_DEBUG, "change_vap_mode: status returned = %d\n", status);
		//while (set80211param(priv, IEEE80211_IOC_CHANGE_VAP_MODE, mode)) {
		while(status) {
			status = set80211param(priv, IEEE80211_IOC_CHANGE_VAP_MODE, mode);
			loop_count++;
			wpa_printf(MSG_DEBUG, "loop_count = %d\n", loop_count);
			if(loop_count >= 10)
			{
				wpa_printf(MSG_DEBUG, "Unable to set mode = %d\n", mode);
				return -1;
			}
			/* Sleeping for 1 milli second before retrying */
			os_sleep(0, 1000);
		}
	}
#endif


	if (bsd_set_mediaopt(drv, IFM_OMASK, mode) < 0) {
		wpa_printf(MSG_ERROR, "%s: failed to set operation mode",
			   __func__);
		return -1;
	}

	//drv->sock_xmit = l2_packet_init(drv->ifname, NULL, ETH_P_EAPOL,
						//handle_read, drv, 0);
		//if (drv->sock_xmit == NULL)
			//return -1;
			
	drv->mode_11j = 0;

	if ((params->mode == IEEE80211_MODE_AP) || (params->mode == IEEE80211_MODE_P2P_GO)) {
		if (params->freq >= 2412 && params->freq <= 2472)
			channel = (params->freq - 2407) / 5;
		else if (params->freq == 2484)
			channel = 14;
		else if ((params->freq >= 5180 && params->freq <= 5320) ||
			 (params->freq >= 5500 && params->freq <= 5700) ||
			 (params->freq >= 5745 && params->freq <= 5825))
			channel = (params->freq - 5000) / 5;
		else if ((params->freq >= 5040 && params->freq <= 5080))
		{
				drv->mode_11j = 1;
				channel = (params->freq - 5000) / 5;
		}
		else if ((params->freq >= 4920 && params->freq <= 4980))
		{
				drv->mode_11j = 1;
				channel = (params->freq - 4000) / 5;
		}
		else
			channel = 0;
		if (bsd_set_freq(drv, channel) < 0)
			return -1;
				drv->is_ap = 1;

		wpa_printf(MSG_DEBUG, "In %s %d drv->sock_xmit = %p alg %d \n", __func__, __LINE__, drv->sock_xmit, params->auth_alg);
		/* Ideally this will be done in the initialization itself 
		 * But for some reason its getting NULL again we need to see why 
		 * that's happening and resolve this with a permanent fix */ //TODO
		if (wpa_driver_bsd_set_auth_alg(drv, params->auth_alg) < 0)
			ret = -1;
		if(drv->sock_xmit == NULL)
		{
			drv->sock_xmit = l2_packet_init(drv->ifname, NULL, ETH_P_EAPOL,
						handle_read, drv, 0);
			wpa_printf(MSG_DEBUG, "In %s %d drv->sock_xmit = %p\n", __func__, __LINE__, drv->sock_xmit);
			if (drv->sock_xmit == NULL)
				return -1;
		drv->is_ap = 1;
		}

		return 0;
	}
	if (wpa_driver_bsd_set_drop_unencrypted(drv, params->drop_unencrypted)
	    < 0)
		ret = -1;
	
	if (wpa_driver_bsd_set_auth_alg(drv, params->auth_alg) < 0)
		ret = -1;
	/* XXX error handling is wrong but unclear what to do... */
	if (wpa_driver_bsd_set_wpa_ie(drv, params->wpa_ie, params->wpa_ie_len) < 0)
		return -1;


	privacy = !(params->pairwise_suite == 1 &&
	    params->group_suite == 1 &&
	    params->key_mgmt_suite == 4 &&
	    params->wpa_ie_len == 0);
	wpa_printf(MSG_DEBUG, "%s: set PRIVACY %u", __func__, privacy);

	if (set80211param(drv, IEEE80211_IOC_PRIVACY, privacy) < 0)
		return -1;

	if (params->wpa_ie_len &&
	    set80211param(drv, IEEE80211_IOC_WPA,
			  params->wpa_ie[0] == WLAN_EID_RSN ? 2 : 1) < 0)
		return -1;

	if ((params->ssid != NULL) && (set80211var(drv, IEEE80211_IOC_SSID, params->ssid, params->ssid_len)))
	{
		return -1;
	}

	os_memset(&mlme, 0, sizeof(mlme));
	mlme.im_op = IEEE80211_MLME_ASSOC;
	if (params->ssid != NULL)
	{
		os_memcpy(mlme.im_ssid, params->ssid, params->ssid_len);
	}
	mlme.im_ssid_len = params->ssid_len;
#ifdef RSI_CCX
	mlme.ccx_capable = params->ccx_capable;
	mlme.tsf = params->tsf;
	mlme.rn = params->rn;
	mlme.mic = params->mic;
#endif	

	if (params->bssid != NULL)
		os_memcpy(mlme.im_macaddr, params->bssid, IEEE80211_ADDR_LEN);

	wpa_printf(MSG_DEBUG, "ASSOCIATE addr : %02x:%02x:%02x:%02x:%02x:%02x\n", mlme.im_macaddr[0], mlme.im_macaddr[1], mlme.im_macaddr[2], mlme.im_macaddr[3], mlme.im_macaddr[4], mlme.im_macaddr[5]);
	wpa_printf(MSG_DEBUG, "ASSOCIATE: ssid : %s\n", mlme.im_ssid);

	if (set80211var(drv, IEEE80211_IOC_MLME, &mlme, sizeof(mlme)) < 0)
		return -1;
	return ret;
}

/**
 * wpa_driver_wext_scan_timeout - Scan timeout to report scan completion
 * @eloop_ctx: Unused
 * @timeout_ctx: ctx argument given to wpa_driver_wext_init()
 *
 * This function can be used as registered timeout when starting a scan to
 * generate a scan completed event if the driver does not report this.
 */
void wpa_driver_bsd_scan_timeout(void *eloop_ctx, void *timeout_ctx)
{
	wpa_printf(MSG_DEBUG, "Scan timeout - try to get results");
	wpa_supplicant_event(timeout_ctx, EVENT_SCAN_RESULTS, NULL);
}

static int
wpa_driver_bsd_scan(void *priv, struct wpa_driver_scan_params *params)
{
	struct bsd_driver_data *drv = priv;
	int timeout;
#ifdef IEEE80211_IOC_SCAN_MAX_SSID
	struct ieee80211_scan_req sr;
	int i;
#endif /* IEEE80211_IOC_SCAN_MAX_SSID */

	if(drv->is_ap)
	{
		if (bsd_set_mediaopt(drv, IFM_OMASK, 512 /* ACS */) < 0) 
		{
			wpa_printf(MSG_ERROR, "%s: failed to set operation mode",
				       __func__);
			return -1;
		}
	}
	else
	{
		if (bsd_set_mediaopt(drv, IFM_OMASK, 0 /* STA */) < 0) 
		{
			wpa_printf(MSG_ERROR, "%s: failed to set operation mode",
				       __func__);
			return -1;
		}
	}

	if (bsd_set_mediaopt(drv, IFM_MMASK, 0) < 0) {
		wpa_printf(MSG_ERROR, "%s: failed to set modulation mode",
			   __func__);
		return -1;
	}

	if (set80211param(drv, IEEE80211_IOC_ROAMING,
			  IEEE80211_ROAMING_MANUAL) < 0) {
		wpa_printf(MSG_ERROR, "%s: failed to set "
			   "wpa_supplicant-based roaming: %s", __func__,
			   strerror(errno));
		return -1;
	}

	if (wpa_driver_bsd_set_wpa(drv, 1) < 0) {
		wpa_printf(MSG_ERROR, "%s: failed to set wpa: %s", __func__,
			   strerror(errno));
		return -1;
	}

	/* NB: interface must be marked UP to do a scan */
	if (bsd_ctrl_iface(drv, 1) < 0)
		return -1;
	/* Configure WPS/P2P ie for probe req before scanning */
	if(params->extra_ies && params->extra_ies_len)
	{
		wpa_printf(MSG_DEBUG, " Setting WPS/P2P IE FOR PROBE REQ \n");
		bsd_set80211(priv, IEEE80211_IOC_APPIE, (IEEE80211_FC0_TYPE_MGT | IEEE80211_FC0_SUBTYPE_PROBE_REQ),
						params->extra_ies, params->extra_ies_len);
	}
#ifdef IEEE80211_IOC_SCAN_MAX_SSID
	os_memset(&sr, 0, sizeof(sr));
	
	if(drv->is_ap)
	{
		sr.sr_flags = IEEE80211_IOC_SCAN_ONCE;
	}
	else
	{
		sr.sr_flags = IEEE80211_IOC_SCAN_ACTIVE | IEEE80211_IOC_SCAN_ONCE |
					  IEEE80211_IOC_SCAN_NOJOIN;
	}
	sr.sr_duration = IEEE80211_IOC_SCAN_FOREVER;
	if (params->num_ssids > 0) {
		sr.sr_nssid = params->num_ssids;
#if 1
		/* Boundary check is done by upper layer as supplicant supports upto 16 SSIDs to Scan while driver supports only 3. So in case if supplicant gives more than 3 SSIDs then we will pick first 3 SSIDs.*/
		if (sr.sr_nssid > IEEE80211_IOC_SCAN_MAX_SSID)
			sr.sr_nssid = IEEE80211_IOC_SCAN_MAX_SSID;
#endif

		/* NB: check scan cache first */
		sr.sr_flags |= IEEE80211_IOC_SCAN_CHECK;
	}
	for (i = 0; i < sr.sr_nssid; i++) {
		sr.sr_ssid[i].len = params->ssids[i].ssid_len;
		os_memcpy(sr.sr_ssid[i].ssid, params->ssids[i].ssid,
			  sr.sr_ssid[i].len);
	}

 	timeout = 5;
	wpa_printf(MSG_DEBUG, "Scan requested - scan timeout %d "
		   "seconds", timeout);
	eloop_cancel_timeout(wpa_driver_bsd_scan_timeout, drv, drv->ctx);
	eloop_register_timeout(timeout, 0, wpa_driver_bsd_scan_timeout, drv,
			       drv->ctx);
	if(params->freqs)
	{
		int count = 0;
		while(params->freqs[count])
		{
			wpa_printf(MSG_DEBUG, "scanning selected channel freq = %d\n", params->freqs[count]);
			sr.freqs[count] = params->freqs[count];
			count++;
			if(count == IEEE80211_MAX_FREQS_ALLOWED)
				break;
		}
		sr.num_freqs = count;
	}
	else
	{
		wpa_printf(MSG_DEBUG, "scanning All channels\n");
		sr.num_freqs = 0;
	}

	/* NB: net80211 delivers a scan complete event so no need to poll */
	return set80211var(drv, IEEE80211_IOC_SCAN_REQ, &sr, sizeof(sr));
#else /* IEEE80211_IOC_SCAN_MAX_SSID */
	/* set desired ssid before scan */
	if (bsd_set_ssid(drv, params->ssids[0].ssid,
			 params->ssids[0].ssid_len) < 0)
		return -1;

	/* NB: net80211 delivers a scan complete event so no need to poll */
	return set80211param(drv, IEEE80211_IOC_SCAN_REQ, 0);
#endif /* IEEE80211_IOC_SCAN_MAX_SSID */
}
#define    RTM_VERSION             5 
#define    RTM_IFANNOUNCE          0x11
#define    RTM_IEEE80211           0x12
#define    RTM_IEEE80211_ASSOC     100
#define    RTM_IEEE80211_REASSOC   101
#define    RTM_IEEE80211_DISASSOC  102
#define    RTM_IEEE80211_SCAN      105
#define    RTM_IEEE80211_LEAVE     104
#define    RTM_IEEE80211_JOIN      103
#define    RTM_IEEE80211_REPLAY    106     /* sequence counter replay detected */
#define    RTM_IEEE80211_MICHAEL   107
#define    RTM_IFINFO              0xe
#define    RTF_UP                  0x1  

static void
bsd_wireless_event_wireless_custom(struct bsd_driver_data *drv,
				       char *custom, uint16_t len)
{
	wpa_printf(MSG_DEBUG, "Custom wireless event len = %d", len);

	if (strncmp(custom, "MLME-MICHAELMICFAILURE.indication", 33) == 0) {
		char *pos;
		u8 addr[ETH_ALEN];
		pos = strstr(custom, "addr=");
		if (pos == NULL) {
			wpa_printf(MSG_DEBUG,
				   "MLME-MICHAELMICFAILURE.indication "
				   "without sender address ignored");
			return;
		}
		pos += 5;
		if (hwaddr_aton(pos, addr) == 0) {
			wpa_supplicant_event(drv->ctx, EVENT_MICHAEL_MIC_FAILURE, (union wpa_event_data *)custom);
			//ieee80211_michael_mic_failure(drv->hapd, addr, 1); FIXME
		} else {
			wpa_printf(MSG_DEBUG,
				   "MLME-MICHAELMICFAILURE.indication "
				   "with invalid MAC address");
		}
	} 
	else /* The above event also should be done in the following way. But this is temporary patch hence left */
	{
		uint16_t custom_cmd_type = *(uint16_t *)custom;
		uint16_t recv_freq;
		uint16_t duration;
		union wpa_event_data data;

		os_memset(&data, 0, sizeof(data));
		switch(custom_cmd_type)
		{
			case DRIVER_EVENT_REMAIN_ON_CHANNEL:
			{
				recv_freq = *(uint16_t *)(custom + 2);
				duration  = *(uint16_t *)(custom + 4);
				data.remain_on_channel.freq = recv_freq;
				data.remain_on_channel.duration = duration;
				wpa_supplicant_event(drv->ctx, EVENT_REMAIN_ON_CHANNEL, &data);
			}
			break;
			case DRIVER_EVENT_CANCEL_REMAIN_ON_CHANNEL:
			{
				recv_freq = *(uint16_t *)(custom + 2);
				data.remain_on_channel.freq = recv_freq;
				wpa_supplicant_event(drv->ctx, EVENT_CANCEL_REMAIN_ON_CHANNEL, &data);
			}
			break;
			case DRIVER_EVENT_MGMT:
			{
				struct ieee80211_mgmt *rcv_pkt = (struct ieee80211_mgmt *)(custom + 4);

				recv_freq = *(uint16_t *)(custom + 2);

				switch(WLAN_FC_GET_STYPE(rcv_pkt->frame_control))
				{
					case WLAN_FC_STYPE_PROBE_REQ:
					{
						data.rx_probe_req.da = rcv_pkt->da;
						data.rx_probe_req.sa = rcv_pkt->sa;
						data.rx_probe_req.bssid = rcv_pkt->bssid;
						data.rx_probe_req.ie = &rcv_pkt->u.probe_req.variable[0];
						data.rx_probe_req.ie_len = len + (data.rx_probe_req.ie - (const u8 *)custom);
						wpa_supplicant_event(drv->ctx, EVENT_RX_PROBE_REQ, &data);
					}
					break;
					case WLAN_FC_STYPE_ACTION:
					{
						data.rx_mgmt.frame = (u8 *) (custom + 4);
						data.rx_mgmt.frame_len = len + ((&rcv_pkt->u.action.category + 1) - (const u8 *)custom);
						wpa_supplicant_event(drv->ctx, EVENT_RX_MGMT, &data);
					}
					break;
					default:
					{
						wpa_printf(MSG_DEBUG,"NON EXPECTING SUB TYPE = %0x", rcv_pkt->frame_control);
					}
					break;
				}
			}
			break;
			case DRIVER_EVENT_ASSOC:
			{
				uint8_t *addr, *ies;
				uint16_t ies_len, reassoc;

				reassoc = *(uint16_t *)(custom + 2);
				addr = (uint8_t *)(custom + 4);
				if(drv->is_ap)
				{
					ies_len = *(uint16_t *)(custom + 10);
					ies = (uint8_t *)(custom + 12);
				}
				else
				{
					ies_len = 0;
					ies = NULL;
				}
				drv_event_assoc(drv->ctx, addr, ies, ies_len, reassoc);
			}
			break;
#ifdef RSI_CCX
			case DRIVER_EVENT_ASSOC_RESP_CCKM:
			wpa_printf(MSG_DEBUG," Recvd Reassoc resp event");
			int len = *(uint16_t *)(custom + 2);;
			memcpy (&data, custom+4, len);
			wpa_supplicant_event(drv->ctx, EVENT_CCKM_REASSOC_RESP, &data);
			break;
#endif
			default:
			{
				wpa_printf(MSG_DEBUG,"Received unknown CUSTOM DRIVER_EVENT EVENT Value = %x\n", custom_cmd_type);
			}
			break;
		}
	}
}


static void
bsd_wireless_event_wireless(struct bsd_driver_data *drv,
					    char *data, int len)
{
	struct iw_event iwe_buf, *iwe = &iwe_buf;
	char *pos, *end, *custom, *buf;

	pos = data;
	end = data + len;

	/* This whole logic may not make any sense for now 
	 * Need to re-work on this when we have time*/ //TODO
	while (pos + IW_EV_LCP_LEN <= end) {
		/* Event data may be unaligned, so make a local, aligned copy
		 * before processing. */
		memcpy(&iwe_buf, pos, IW_EV_LCP_LEN);
		wpa_printf(MSG_MSGDUMP, "Wireless event: cmd=0x%x len=%d",
			   iwe->cmd, iwe->len);
		if (iwe->len <= IW_EV_LCP_LEN)
			return;

		custom = pos + IW_EV_POINT_LEN;
		if (/*drv->we_version > 18 && */
		    (iwe->cmd == IWEVMICHAELMICFAILURE ||
		     iwe->cmd == IWEVCUSTOM)) {
			/* WE-19 removed the pointer from struct iw_point */
			char *dpos = (char *) &iwe_buf.u.data.length;
			int dlen = dpos - (char *) &iwe_buf;
			memcpy(dpos, pos + IW_EV_LCP_LEN,
			       sizeof(struct iw_event) - dlen);
		} else {
			memcpy(&iwe_buf, pos, sizeof(struct iw_event));
			custom += IW_EV_POINT_OFF;
		}

		switch (iwe->cmd) {
		case IWEVEXPIRED:
			drv_event_disassoc(drv->ctx, (const u8 *)iwe->u.addr.sa_data);
			break;
		case IWEVREGISTERED:
			bsd_new_sta(drv, drv->ctx, (u8 *) iwe->u.addr.sa_data);
			break;
		case IWEVCUSTOM:
			if (custom + iwe->u.data.length > end)
				return;
			buf = malloc(iwe->u.data.length + 1);
			if (buf == NULL)
				return;		/* XXX */
			memcpy(buf, custom, iwe->u.data.length);
			buf[iwe->u.data.length] = '\0';
			bsd_wireless_event_wireless_custom(drv, buf, iwe->u.data.length);
			free(buf);
			break;
		case SIOCGIWSCAN:
			wpa_supplicant_event(drv->ctx, EVENT_SCAN_RESULTS, NULL);

			/* Cancelling the scan timer if registered any */
			eloop_cancel_timeout(wpa_driver_bsd_scan_timeout, drv, drv->ctx);

			break;
		}

		pos += iwe->len;
	}
}

static void
bsd_wireless_event_rtm_newlink(struct bsd_driver_data *drv,
					       struct nlmsghdr *h, int len)
{
	struct ifinfomsg *ifi;
	int attrlen, nlmsg_len, rta_len;
	struct rtattr * attr;

	if (len < (int) sizeof(*ifi))
		return;

	ifi = NLMSG_DATA(h);

	if (ifi->ifi_index != drv->ifindex)
		return;

	nlmsg_len = NLMSG_ALIGN(sizeof(struct ifinfomsg));

	attrlen = h->nlmsg_len - nlmsg_len;
	if (attrlen < 0)
		return;

	attr = (struct rtattr *) (((char *) ifi) + nlmsg_len);

	rta_len = RTA_ALIGN(sizeof(struct rtattr));
	while (RTA_OK(attr, attrlen)) {
		if (attr->rta_type == IFLA_WIRELESS) {
			bsd_wireless_event_wireless(
				drv, ((char *) attr) + rta_len,
				attr->rta_len - rta_len);
		}
		attr = RTA_NEXT(attr, attrlen);
	}
}


static void
wpa_driver_bsd_event_receive(int sock, void *ctx, void *sock_ctx)
{
	//char buf[512];
	#define BUF_SIZE 512
	char *buf;
	int left;
	struct sockaddr_nl from;
	socklen_t fromlen;
	struct nlmsghdr *h;
	struct bsd_driver_data *drv = ctx;

	buf = malloc(BUF_SIZE);
	fromlen = sizeof(from);
	left = recvfrom(sock, buf, BUF_SIZE, MSG_DONTWAIT,
			(struct sockaddr *) &from, &fromlen);
	if (left < 0) {
		if (errno != EINTR && errno != EAGAIN)
			perror("recvfrom(netlink)");
		return;
	}

	h = (struct nlmsghdr *) buf;
	while (left >= (int) sizeof(*h)) {
		int len, plen;

		len = h->nlmsg_len;
		plen = len - sizeof(*h);
		if (len > left || plen < 0) {
			wpa_printf(MSG_DEBUG,"Malformed netlink message: "
					"len=%d left=%d plen=%d\n",
					len, left, plen);
			break;
		}

		switch (h->nlmsg_type) {
		case RTM_NEWLINK:
			bsd_wireless_event_rtm_newlink(drv, h, plen);
			break;
		}

		len = NLMSG_ALIGN(len);
		left -= len;
		h = (struct nlmsghdr *) ((char *) h + len);
	}

	if (left > 0) {
			wpa_printf(MSG_DEBUG,"%d extra bytes in the end of netlink message\n", left);
	}

	free(buf);

#if 0
	struct bsd_driver_data *drv = sock_ctx;
	char buf[2048];
	struct if_announcemsghdr *ifan;
	struct if_msghdr *ifm;
	struct rt_msghdr *rtm;
	union wpa_event_data event;
	struct ieee80211_michael_event *mic;
	struct ieee80211_leave_event *leave;
	struct ieee80211_join_event *join;
	int n;

	n = read(sock, buf, sizeof(buf));
	if (n < 0) {
		if (errno != EINTR && errno != EAGAIN)
			perror("read(PF_ROUTE)");
		return;
	}

	rtm = (struct rt_msghdr *) buf;
	if (rtm->rtm_version != RTM_VERSION) {
		wpa_printf(MSG_DEBUG, "Routing message version %d not "
			"understood\n", rtm->rtm_version);
		return;
	}
	os_memset(&event, 0, sizeof(event));
	switch (rtm->rtm_type) {
	case RTM_IFANNOUNCE:
		ifan = (struct if_announcemsghdr *) rtm;
		if (ifan->ifan_index != drv->ifindex)
			break;
		os_strlcpy(event.interface_status.ifname, drv->ifname,
			   sizeof(event.interface_status.ifname));
		switch (ifan->ifan_what) {
		case IFAN_DEPARTURE:
			event.interface_status.ievent = EVENT_INTERFACE_REMOVED;
		default:
			return;
		}
		wpa_printf(MSG_DEBUG, "RTM_IFANNOUNCE: Interface '%s' %s",
			   event.interface_status.ifname,
			   ifan->ifan_what == IFAN_DEPARTURE ?
				"removed" : "added");
		wpa_supplicant_event(ctx, EVENT_INTERFACE_STATUS, &event);
		break;
	case RTM_IEEE80211:
		ifan = (struct if_announcemsghdr *) rtm;
		if (ifan->ifan_index != drv->ifindex)
			break;
		switch (ifan->ifan_what) {
		case RTM_IEEE80211_ASSOC:
		case RTM_IEEE80211_REASSOC:
			if (drv->is_ap)
				break;
			wpa_supplicant_event(ctx, EVENT_ASSOC, NULL);
			break;
		case RTM_IEEE80211_DISASSOC:
			if (drv->is_ap)
				break;
			wpa_supplicant_event(ctx, EVENT_DISASSOC, NULL);
			break;
		case RTM_IEEE80211_SCAN:
			if (drv->is_ap)
				break;
			wpa_supplicant_event(ctx, EVENT_SCAN_RESULTS, NULL);
			break;
		case RTM_IEEE80211_LEAVE:
			leave = (struct ieee80211_leave_event *) &ifan[1];
			drv_event_disassoc(ctx, leave->iev_addr);
			break;
		case RTM_IEEE80211_JOIN:
#ifdef RTM_IEEE80211_REJOIN
		case RTM_IEEE80211_REJOIN:
#endif
			join = (struct ieee80211_join_event *) &ifan[1];
			bsd_new_sta(drv, ctx, join->iev_addr);
			break;
		case RTM_IEEE80211_REPLAY:
			/* ignore */
			break;
		case RTM_IEEE80211_MICHAEL:
			mic = (struct ieee80211_michael_event *) &ifan[1];
			wpa_printf(MSG_DEBUG,
				"Michael MIC failure wireless event: "
				"keyix=%u src_addr=" MACSTR, mic->iev_keyix,
				MAC2STR(mic->iev_src));

			os_memset(&event, 0, sizeof(event));
			event.michael_mic_failure.unicast =
				!IEEE80211_IS_MULTICAST(mic->iev_dst);
			wpa_supplicant_event(ctx, EVENT_MICHAEL_MIC_FAILURE,
				&event);
			break;
		}
		break;
	case RTM_IFINFO:
		ifm = (struct if_msghdr *) rtm;
		if (ifm->ifm_index != drv->ifindex)
			break;
		if ((rtm->rtm_flags & RTF_UP) == 0) {
			os_strlcpy(event.interface_status.ifname, drv->ifname,
				   sizeof(event.interface_status.ifname));
			event.interface_status.ievent = EVENT_INTERFACE_REMOVED;
			wpa_printf(MSG_DEBUG, "RTM_IFINFO: Interface '%s' DOWN",
				   event.interface_status.ifname);
			wpa_supplicant_event(ctx, EVENT_INTERFACE_STATUS, &event);
		}
		break;
	}
#endif
}

static void
wpa_driver_bsd_add_scan_entry(struct wpa_scan_results *res,
			      struct ieee80211req_scan_result *sr)
{
	struct wpa_scan_res *result, **tmp;
	size_t extra_len;
	u8 *pos;

	extra_len = 2 + sr->isr_ssid_len;
	extra_len += 2 + sr->isr_nrates;
	extra_len += 3; /* ERP IE */
	extra_len += sr->isr_ie_len;

	result = os_zalloc(sizeof(*result) + extra_len);
	if (result == NULL)
		return;
	os_memcpy(result->bssid, sr->isr_bssid, ETH_ALEN);
	result->freq = sr->isr_freq;
	result->beacon_int = sr->isr_intval;
	result->caps = sr->isr_capinfo;
	result->qual = sr->isr_rssi;
	result->noise = sr->isr_noise;
#ifdef RSI_CCX
	result->tsf = sr->tsf;
	result->ccx_capable = sr->ccx_capable;
#endif
	/*
	 * the rssi value reported by the kernel is in 0.5dB steps relative to
	 * the reported noise floor. see ieee80211_node.h for details.
	 */
	//result->level = sr->isr_rssi / 2 + sr->isr_noise;
	result->level = sr->isr_rssi;


	pos = (u8 *)(result + 1);

	*pos++ = WLAN_EID_SSID;
	*pos++ = sr->isr_ssid_len;
	os_memcpy(pos, sr + 1, sr->isr_ssid_len);
	pos += sr->isr_ssid_len;

	/*
	 * Deal all rates as supported rate.
	 * Because net80211 doesn't report extended supported rate or not.
	 */
	*pos++ = WLAN_EID_SUPP_RATES;
	*pos++ = sr->isr_nrates;
	os_memcpy(pos, sr->isr_rates, sr->isr_nrates);
	pos += sr->isr_nrates;

	*pos++ = WLAN_EID_ERP_INFO;
	*pos++ = 1;
	*pos++ = sr->isr_erp;

	os_memcpy(pos, (u8 *)(sr + 1) + sr->isr_ssid_len, sr->isr_ie_len);
	pos += sr->isr_ie_len;

	result->ie_len = pos - (u8 *)(result + 1);

	tmp = os_realloc_array(res->res, res->num + 1,
			       sizeof(struct wpa_scan_res *));
	if (tmp == NULL) {
		os_free(result);
		return;
	}
	tmp[res->num++] = result;
	res->res = tmp;
}

struct wpa_scan_results *
wpa_driver_bsd_get_scan_results2(void *priv)
{
	struct ieee80211req_scan_result *sr;
	struct wpa_scan_results *res;
	int len, rest;
	//uint8_t buf[24*1024], *pos;
	uint8_t *buf, *pos;
	
	buf = os_zalloc(32*1024);

	len = get80211var(priv, IEEE80211_IOC_SCAN_RESULTS, (buf), 32*1024);

	if (buf == NULL)
		return NULL;
	if (len < 0) {
		os_free(buf);
		return NULL;
	}

	res = os_zalloc(sizeof(*res));
	if (res == NULL) {
		os_free(buf);
		return NULL;
	}

	pos = (buf);
	rest = len;
	while (rest >= sizeof(struct ieee80211req_scan_result)) {
		sr = (struct ieee80211req_scan_result *)pos;
		wpa_driver_bsd_add_scan_entry(res, sr);
		pos += sr->isr_len;
		rest -= sr->isr_len;
	}

	wpa_printf(MSG_DEBUG, "Received %d bytes of scan results (%lu BSSes)",
		   len, (unsigned long)res->num);

	os_free(buf);
	return res;
}

static int wpa_driver_bsd_capa(struct bsd_driver_data *drv)
{
#ifdef IEEE80211_IOC_DEVCAPS
/* kernel definitions copied from net80211/ieee80211_var.h */
#define IEEE80211_CIPHER_WEP            0
#define IEEE80211_CIPHER_TKIP           1
#define IEEE80211_CIPHER_AES_CCM        3
#define IEEE80211_CRYPTO_WEP            (1<<IEEE80211_CIPHER_WEP)
#define IEEE80211_CRYPTO_TKIP           (1<<IEEE80211_CIPHER_TKIP)
#define IEEE80211_CRYPTO_AES_CCM        (1<<IEEE80211_CIPHER_AES_CCM)
#define IEEE80211_C_HOSTAP      0x00000400      /* CAPABILITY: HOSTAP avail */
#define IEEE80211_C_WPA1        0x00800000      /* CAPABILITY: WPA1 avail */
#define IEEE80211_C_WPA2        0x01000000      /* CAPABILITY: WPA2 avail */
#define IEEE80211_C_P2P         0x10000000      /* CAPABILITY: P2P  avail */
	struct ieee80211_devcaps_req *devcaps;
	devcaps = os_zalloc(sizeof(struct ieee80211_devcaps_req));

	//if (get80211var(drv, IEEE80211_IOC_DEVCAPS, &devcaps,
	if (get80211var(drv, IEEE80211_IOC_DEVCAPS, devcaps,
			sizeof(struct ieee80211_devcaps_req)) < 0) {
		wpa_printf(MSG_ERROR, "failed to IEEE80211_IOC_DEVCAPS: %s",
			   strerror(errno));
    os_free(devcaps);
		return -1;
	}

	wpa_printf(MSG_DEBUG, "%s: drivercaps=0x%08x,cryptocaps=0x%08x",
		   __func__, devcaps->dc_drivercaps, devcaps->dc_cryptocaps);

	if (devcaps->dc_drivercaps & IEEE80211_C_WPA1)
		drv->capa.key_mgmt = WPA_DRIVER_CAPA_KEY_MGMT_WPA |
			WPA_DRIVER_CAPA_KEY_MGMT_WPA_PSK;
	if (devcaps->dc_drivercaps & IEEE80211_C_WPA2)
		drv->capa.key_mgmt = WPA_DRIVER_CAPA_KEY_MGMT_WPA2 |
			WPA_DRIVER_CAPA_KEY_MGMT_WPA2_PSK;

	if (devcaps->dc_cryptocaps & IEEE80211_CRYPTO_WEP)
		drv->capa.enc |= WPA_DRIVER_CAPA_ENC_WEP40 |
			WPA_DRIVER_CAPA_ENC_WEP104;
	if (devcaps->dc_cryptocaps & IEEE80211_CRYPTO_TKIP)
		drv->capa.enc |= WPA_DRIVER_CAPA_ENC_TKIP;
	if (devcaps->dc_cryptocaps & IEEE80211_CRYPTO_AES_CCM)
		drv->capa.enc |= WPA_DRIVER_CAPA_ENC_CCMP;

	if (devcaps->dc_drivercaps & IEEE80211_C_HOSTAP)
		drv->capa.flags |= WPA_DRIVER_FLAGS_AP;

	if (devcaps->dc_drivercaps & IEEE80211_C_P2P)
		drv->capa.flags |= WPA_DRIVER_FLAGS_P2P_CAPABLE;
#undef IEEE80211_CIPHER_WEP
#undef IEEE80211_CIPHER_TKIP
#undef IEEE80211_CIPHER_AES_CCM
#undef IEEE80211_CRYPTO_WEP
#undef IEEE80211_CRYPTO_TKIP
#undef IEEE80211_CRYPTO_AES_CCM
#undef IEEE80211_C_HOSTAP
#undef IEEE80211_C_WPA1
#undef IEEE80211_C_WPA2
#else /* IEEE80211_IOC_DEVCAPS */
	/* For now, assume TKIP, CCMP, WPA, WPA2 are supported */
	drv->capa.key_mgmt = WPA_DRIVER_CAPA_KEY_MGMT_WPA |
		WPA_DRIVER_CAPA_KEY_MGMT_WPA_PSK |
		WPA_DRIVER_CAPA_KEY_MGMT_WPA2 |
		WPA_DRIVER_CAPA_KEY_MGMT_WPA2_PSK;
	drv->capa.enc = WPA_DRIVER_CAPA_ENC_WEP40 |
		WPA_DRIVER_CAPA_ENC_WEP104 |
		WPA_DRIVER_CAPA_ENC_TKIP |
		WPA_DRIVER_CAPA_ENC_CCMP;
	drv->capa.flags |= WPA_DRIVER_FLAGS_AP;
#endif /* IEEE80211_IOC_DEVCAPS */
#ifdef IEEE80211_IOC_SCAN_MAX_SSID
	drv->capa.max_scan_ssids = IEEE80211_IOC_SCAN_MAX_SSID;
#else /* IEEE80211_IOC_SCAN_MAX_SSID */
	drv->capa.max_scan_ssids = 1;
#endif /* IEEE80211_IOC_SCAN_MAX_SSID */
	drv->capa.auth = WPA_DRIVER_AUTH_OPEN |
		WPA_DRIVER_AUTH_SHARED |
		WPA_DRIVER_AUTH_LEAP;
  os_free(devcaps);
	return 0;
}

static enum ieee80211_opmode
get80211opmode(struct bsd_driver_data *drv)
{
	struct ifmediareq ifmr;

	(void) memset(&ifmr, 0, sizeof(ifmr));
	(void) os_strlcpy(ifmr.ifm_name, drv->ifname, sizeof(ifmr.ifm_name));

	if (ioctl(drv->sock, SIOCGIFMEDIA, (caddr_t)&ifmr) >= 0) {
		if (ifmr.ifm_current & IFM_IEEE80211_ADHOC) {
			if (ifmr.ifm_current & IFM_FLAG0)
				return IEEE80211_M_AHDEMO;
			else
				return IEEE80211_M_IBSS;
		}
		if (ifmr.ifm_current & IFM_IEEE80211_HOSTAP)
			return IEEE80211_M_HOSTAP;
		if (ifmr.ifm_current & IFM_IEEE80211_MONITOR)
			return IEEE80211_M_MONITOR;
#ifdef IEEE80211_M_MBSS
		if (ifmr.ifm_current & IFM_IEEE80211_MBSS)
			return IEEE80211_M_MBSS;
#endif /* IEEE80211_M_MBSS */
	}
	return IEEE80211_M_STA;
}

static void *
wpa_driver_bsd_init(void *ctx, const char *ifname)
{
#define	GETPARAM(drv, param, v) \
	(((v) = get80211param(drv, param)) != -1)
	struct bsd_driver_data *drv;
	struct sockaddr_nl local;

	drv = os_zalloc(sizeof(*drv));
	if (drv == NULL)
		return NULL;

	drv->event_buf_len = rtbuf_len();

	drv->event_buf = os_malloc(drv->event_buf_len);
	if (drv->event_buf == NULL) {
		wpa_printf(MSG_ERROR, "%s: os_malloc() failed", __func__);
		goto fail1;
	}

	/*
	 * NB: We require the interface name be mappable to an index.
	 *     This implies we do not support having wpa_supplicant
	 *     wait for an interface to appear.  This seems ok; that
	 *     doesn't belong here; it's really the job of devd.
	 */
	drv->ifindex = if_nametoindex(ifname);
	if (drv->ifindex == 0) {
		wpa_printf(MSG_DEBUG, "%s: interface %s does not exist",
			   __func__, ifname);
		goto fail1;
	}
	drv->sock = socket(PF_INET, SOCK_DGRAM, 0);
	if (drv->sock < 0)
		goto fail1;
	drv->route = socket(PF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
	if (drv->route < 0) {
		perror("socket(PF_NETLINK,SOCK_RAW,NETLINK_ROUTE)");
		goto fail;
	}

	memset(&local, 0, sizeof(local));
	local.nl_family = AF_NETLINK;
	local.nl_groups = RTMGRP_LINK;
	if (bind(drv->route, (struct sockaddr *) &local, sizeof(local)) < 0) {
		perror("bind(netlink)");
		close(drv->route);
		goto fail2;
	}

	drv->ctx = ctx;
	eloop_register_read_sock(drv->route,
		wpa_driver_bsd_event_receive, drv, NULL);

	os_strlcpy(drv->ifname, ifname, sizeof(drv->ifname));

	/* Down interface during setup. */
	if (bsd_ctrl_iface(drv, 0) < 0)
		goto fail;

	if (!GETPARAM(drv, IEEE80211_IOC_ROAMING, drv->prev_roaming)) {
		wpa_printf(MSG_DEBUG, "%s: failed to get roaming state: %s",
			__func__, strerror(errno));
		goto fail;
	}
	if (!GETPARAM(drv, IEEE80211_IOC_PRIVACY, drv->prev_privacy)) {
		wpa_printf(MSG_DEBUG, "%s: failed to get privacy state: %s",
			__func__, strerror(errno));
		goto fail;
	}
	if (!GETPARAM(drv, IEEE80211_IOC_WPA, drv->prev_wpa)) {
		wpa_printf(MSG_DEBUG, "%s: failed to get wpa state: %s",
			__func__, strerror(errno));
		goto fail;
	}

	if (wpa_driver_bsd_capa(drv))
		goto fail;

	drv->opmode = get80211opmode(drv);

	return drv;
fail2:
	close(drv->route);
fail:
	close(drv->sock);
fail1:
	os_free(drv);
	return NULL;
#undef GETPARAM
}

static void
wpa_driver_bsd_deinit(void *priv)
{
	struct bsd_driver_data *drv = priv;

	wpa_driver_bsd_set_wpa(drv, 0);
	eloop_unregister_read_sock(drv->route);
	close(drv->route);

	/* NB: mark interface down */
	bsd_ctrl_iface(drv, 0);

	wpa_driver_bsd_set_wpa_internal(drv, drv->prev_wpa, drv->prev_privacy);
	if (set80211param(drv, IEEE80211_IOC_ROAMING, drv->prev_roaming) < 0)
		wpa_printf(MSG_DEBUG, "%s: failed to restore roaming state",
			__func__);

	if (drv->sock_xmit != NULL)
		l2_packet_deinit(drv->sock_xmit);
	(void) close(drv->route);		/* ioctl socket */
	(void) close(drv->sock);		/* event socket */
	os_free(drv->event_buf);
	os_free(drv);
}

static int
wpa_driver_bsd_get_capa(void *priv, struct wpa_driver_capa *capa)
{
	struct bsd_driver_data *drv = priv;

	os_memcpy(capa, &drv->capa, sizeof(*capa));
	return 0;
}
#endif /* HOSTAPD */

static int
hostapd_bsd_set_freq(void *priv, struct hostapd_freq_params *freq)
{
//	return bsd_set_freq(priv, freq->channel);
	if(bsd_set_freq(priv, freq->channel) < 0)
	{
		return -1;
	}

#ifdef RSI_CHANGES
	if(freq->acs)
	{
		struct wpa_driver_scan_params params;
		os_memset(&params, 0, sizeof(params));
		if(wpa_driver_bsd_scan(priv, &params) < 0)
		{
			return -1;
		}
	}
#endif
	return 0;
}

#ifdef CONFIG_P2P

/* Ideally this event should be coming from driver, depending upon the tx status */ //FIXME
void send_action_tx_status (void *priv, const u16 fc, const u8 *data, size_t len, const u8 *dst)
{
	union wpa_event_data wpa_event;
	struct bsd_driver_data *drv = priv;

	os_memset(&wpa_event, 0, sizeof(wpa_event));
	wpa_event.tx_status.type = WLAN_FC_GET_TYPE(fc);
	wpa_event.tx_status.stype = WLAN_FC_GET_STYPE(fc);
	wpa_event.tx_status.dst = dst;
	wpa_event.tx_status.data = data;
	wpa_event.tx_status.data_len = len;
#ifndef WIFI_ALLIANCE
	wpa_event.tx_status.ack = 0;
#else
	wpa_event.tx_status.ack = 1;
#endif
	wpa_supplicant_event(drv->ctx, EVENT_TX_STATUS, &wpa_event);
}

static int bsd_probe_req_report(void *priv, int report)
{
	return set80211param(priv, IEEE80211_IOC_PROBE_REQ_REPORT, report);
}

static int bsd_send_action(void *priv, unsigned int freq, unsigned int wait,
					  const u8 *dst, const u8 *src,
					  const u8 *bssid,
					  const u8 *data, size_t data_len, int no_cck)
{
	int status;
	u8 *buf;

	buf = os_zalloc(data_len + 3 * ETH_ALEN);
	if(buf == NULL)
		return -1;

	os_memcpy(buf, dst, ETH_ALEN);
	os_memcpy(buf + ETH_ALEN, src, ETH_ALEN);
	os_memcpy(buf + 2 * ETH_ALEN, bssid, ETH_ALEN);
	os_memcpy(buf + 3 * ETH_ALEN, data, data_len);
	status = bsd_set80211(priv, IEEE80211_IOC_SEND_ACTION, freq,
			    buf, (data_len + 3 * ETH_ALEN));
	os_free(buf);

	send_action_tx_status(priv, 0x00D0, data, data_len, dst);
	return status;
}

static int bsd_remain_on_channel(void *priv, unsigned int freq,
						unsigned int duration)
{
	int status;
	u8 *buf;
	struct bsd_driver_data *drv = priv;

	/* Cancelling the scan timer if registered any */
	eloop_cancel_timeout(wpa_driver_bsd_scan_timeout, drv, drv->ctx);

	buf = os_zalloc(sizeof(unsigned int));
	if(buf == NULL)
		return -1;

	os_memcpy(buf, &duration, sizeof(unsigned int));
	status = bsd_set80211(priv, IEEE80211_IOC_REMAIN_ON_CHANNEL, freq,
			    buf, sizeof(unsigned int));
	os_free(buf);
	return status;
}

static int bsd_cancel_remain_on_channel(void *priv)
{
	return set80211param(priv, IEEE80211_IOC_CANCEL_REMAIN_ON_CHANNEL, 0);
}
#endif

static int
bsd_commit(void *priv)
{
		wpa_printf(MSG_DEBUG,"before calling bsd_ctrl_iface in %s and %d\n",__func__,__LINE__);
	return bsd_ctrl_iface(priv, 1 );
}
#ifdef RSI_CCX
int wpa_driver_rsi_reassoc(void *priv, u8 *mic, u8 *rn)
{
//	struct wpa_driver_rsi_data *drv = priv;
//	struct iwreq iwr;
	int ret = 0;
	u8 buffer[12];


	os_memset(buffer, 0, 12);
	os_memcpy(buffer, rn, 4);
	os_memcpy(&buffer[4], mic, 8);

	ret = bsd_set80211(priv, IEEE80211_IOC_CCKM_ASSOC_IE, 12,
			    buffer, sizeof(buffer));

	return ret;
}
#endif

void bsd_set_rekey_info(void *priv, const u8 *kek, const u8 *kck, 
                        const u8 *replay_ctr)
{
  int ret = 0;
  u8 buffer[IEEE80211_KEK_LEN + IEEE80211_KCK_LEN + IEEE80211_REPLAY_CTR_LEN];
  os_memset(buffer, 0 , sizeof(buffer));
  os_memcpy(buffer, kek, IEEE80211_KEK_LEN);
  os_memcpy(&buffer[IEEE80211_KEK_LEN], kck, IEEE80211_KCK_LEN);
  os_memcpy(&buffer[IEEE80211_KEK_LEN + IEEE80211_KCK_LEN], replay_ctr, 
            IEEE80211_REPLAY_CTR_LEN);
  ret = bsd_set80211(priv, IEEE80211_IOC_GTK_REKEY_INFO, IEEE80211_KEK_LEN + 
                     IEEE80211_KCK_LEN + IEEE80211_REPLAY_CTR_LEN, buffer, 
                     sizeof(buffer));
}

const struct wpa_driver_ops wpa_driver_bsd_ops = {
	.name			= "bsd",
	.desc			= "BSD 802.11 support",
#ifdef HOSTAPD
	.hapd_init		= bsd_init,
	.hapd_deinit		= bsd_deinit,
	.set_privacy		= bsd_set_privacy,
	.get_seqnum		= bsd_get_seqnum,
	.flush			= bsd_flush,
	.read_sta_data		= bsd_read_sta_driver_data,
	.sta_disassoc		= bsd_sta_disassoc,
#else /* HOSTAPD */
	.sta_set_flags		= bsd_set_sta_authorized,
	.init			= wpa_driver_bsd_init,
	.deinit			= wpa_driver_bsd_deinit,
	.get_bssid		= wpa_driver_bsd_get_bssid,
	.get_ssid		= wpa_driver_bsd_get_ssid,
	.set_countermeasures	= wpa_driver_bsd_set_countermeasures,
	.scan2			= wpa_driver_bsd_scan,
	.get_scan_results2	= wpa_driver_bsd_get_scan_results2,
	.deauthenticate		= wpa_driver_bsd_deauthenticate,
	//.disassociate		= wpa_driver_bsd_disassociate,
	.associate		= wpa_driver_bsd_associate,
	.get_capa		= wpa_driver_bsd_get_capa,
	.set_privacy		= bsd_set_privacy,
#endif /* HOSTAPD */
	.set_freq		= hostapd_bsd_set_freq,
	.sta_deauth		= bsd_sta_deauth,
	.set_ap_wps_ie		= bsd_set_ap_wps_ie,
	.set_key		= bsd_set_key,
	.set_ieee8021x		= bsd_set_ieee8021x,
	.hapd_set_ssid		= bsd_set_ssid,
	.hapd_get_ssid		= bsd_get_ssid,
	.hapd_send_eapol	= bsd_send_eapol,
	.set_generic_elem	= bsd_set_opt_ie,
	.commit 		= bsd_commit,

#ifdef CONFIG_P2P
	//.get_mac_addr		= bsd_get_mac_addr,
	.probe_req_report   = bsd_probe_req_report,
	.send_action		= bsd_send_action,
	.remain_on_channel  = bsd_remain_on_channel,
	.cancel_remain_on_channel = bsd_cancel_remain_on_channel,
#endif
#ifdef RSI_CCX
	.set_reassoc = wpa_driver_rsi_reassoc,
#endif
  	.set_rekey_info = bsd_set_rekey_info,
};

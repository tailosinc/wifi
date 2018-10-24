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
#include "onebox_wlan_core.h"

#define RSI_IAP_INIT 0
#define RSI_MFI_WRITE_CHALLENGE 1
#define RSI_MFI_READ_CERTIFICATE 2

#define NWP_BASE_ADDR                0x41300000
#define M4_CLK_ADDR                  (NWP_BASE_ADDR + 0x110)      
#define PMU_SPI_BASE_ADDR            0x24050000
#define IPMU_DIRECT_ACCESS(_x)       (PMU_SPI_BASE_ADDR + 0xA000 + ((_x) << 2))
#define IPMU_SPI_ACCESS(_x)          (PMU_SPI_BASE_ADDR + 0x8000 + ((_x) << 2))
#define GPIO_BASE_ADDR            0x40200000
#define GPIO_REG(ID)              (GPIO_BASE_ADDR + ((ID) * 2))
#define EGPIO_BASE_ADDR           0x2404C000

ONEBOX_STATUS onebox_iap_config(WLAN_ADAPTER adapter, uint8 type, struct mfi_challenge *mfi)
{
	onebox_mac_frame_t *mgmt_frame;
	netbuf_ctrl_block_t *netbuf_cb = NULL;
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;
	uint16 *frame_body;
	struct driver_assets *d_assets = adapter->d_assets;
	uint16 buf_len = 0;
	uint8 *challenge_data;
	
	challenge_data = kmalloc(20, GFP_KERNEL);
	if( !challenge_data)
			return -ENOMEM;

	if (type == RSI_IAP_INIT || type == RSI_MFI_READ_CERTIFICATE) {
		netbuf_cb = adapter->os_intf_ops->onebox_alloc_skb(FRAME_DESC_SZ);
		buf_len = FRAME_DESC_SZ;
	} else if (type == RSI_MFI_WRITE_CHALLENGE) {
		netbuf_cb = adapter->os_intf_ops->onebox_alloc_skb(FRAME_DESC_SZ + sizeof(struct mfi_challenge));
		buf_len = FRAME_DESC_SZ + sizeof(struct mfi_challenge);
 	} else {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Invalid type\n"), __func__));
		kfree(challenge_data);
		return ONEBOX_STATUS_FAILURE;
	}
		
	if (netbuf_cb == NULL) { 
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		kfree(challenge_data);
		return ONEBOX_STATUS_FAILURE;
	}
   
	adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, buf_len);
	adapter->os_intf_ops->onebox_memset(netbuf_cb->data, 0, (FRAME_DESC_SZ));

	mgmt_frame = (onebox_mac_frame_t *)&netbuf_cb->data[0];
	adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, buf_len);

	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(((buf_len - FRAME_DESC_SZ)) 
	                                              | (COEX_TX_Q << 12));
	netbuf_cb->tx_pkt_type = COEX_Q;
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(IAP_CONFIG);
	//mgmt_frame->desc_word[2] = ONEBOX_CPU_TO_LE16(extnd_size);
	mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(type);

	if (type == RSI_MFI_WRITE_CHALLENGE) {
		if(copy_from_user(challenge_data, mfi->challenge_data, 20))
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
			kfree(challenge_data);
			return -EINVAL;
		}
		if (mfi == NULL) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: MFI challenge is NULL!!!\n"), __func__));
			kfree_skb(netbuf_cb->pkt_addr);
			kfree(challenge_data);
			return ONEBOX_STATUS_FAILURE;
		}
		frame_body = (uint16 *)&netbuf_cb->data[16];
		adapter->os_intf_ops->onebox_memcpy(frame_body, challenge_data, sizeof(struct mfi_challenge));
		adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR,	(uint8 *)netbuf_cb->data, netbuf_cb->len) ;
	}

	adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG,
					(uint8 *)mgmt_frame,
					 buf_len) ;
	status = adapter->onebox_send_pkt_to_coex(d_assets, netbuf_cb, COEX_Q);
	kfree(challenge_data);
	return status;
}

/**
 *  This function will return index of a given ioctl command.
 *  And the output parameter private indicates whether the given
 *  ioctl is a standard ioctl command or a private ioctl.
 *   0: Standard
 *   1: Private ioctl
 *  -1: Illiegal ioctl
 *
 * @param  value of the ioctl command, input to this function
 * @param  Indicates whether the ioctl is private or standart, output pointer
 * @return returns index of the ioctl
 */

static int get_ioctl_index(int cmd, int *private)
{
	int index = 0;
	*private = 0;

	if ( (cmd >= SIOCIWFIRSTPRIV) && (cmd <= SIOCIWLASTPRIV)) 
	{
		/* Private IOCTL */
		index = cmd - SIOCIWFIRSTPRIV;
		*private = 1;
	} 
	else if ((cmd >= 0x8B00) && (cmd <= 0x8B2D)) 
	{
		/* Standard IOCTL */
		index = cmd - 0x8B00;
		*private = 0;
	} 
	else 
	{
		*private = -1;
	}
	return index;
}

/**
 * This function handles the ioctl for deleting a VAP.
 * @param  Pointer to the ieee80211com structure
 * @param  Pointer to the ifreq structure
 * @param  Pointer to the netdevice structure
 * @return Success or failure  
 */
int
ieee80211_ioctl_delete_vap(struct ieee80211com *ic, struct ifreq *ifr, struct net_device *mdev)
{
	struct ieee80211vap *vap = NULL;
	struct ieee80211_clone_params cp;
	char name[IFNAMSIZ];
	uint8_t wait_for_lock = 0;
	uint8_t vap_del_flag = 0;
	//WLAN_ADAPTER w_adapter = (WLAN_ADAPTER)netdev_priv(mdev);

	if (!capable(CAP_NET_ADMIN))
	{
		return -EPERM;
	}

	if (copy_from_user(&cp, ifr->ifr_data, sizeof(cp)))
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Copy from user failed\n")));
		return -EFAULT;
	}

	strncpy(name, cp.icp_parent, sizeof(name));

	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("User given vap name =%s and list of vap names=%s\n"), name, vap->iv_ifp->name));
		if (!strcmp(vap->iv_ifp->name, name)) 
		{
			//w_adapter->net80211_ops->onebox_ifdetach(ic);
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
			   (TEXT("deleting vap: %s\n"), 
			   vap->iv_ifp->name));
			ic->ic_vap_delete(vap, wait_for_lock);
			vap_del_flag =1;
			break;
		}
	}
	if(vap_del_flag)
	{
			return 0;
	}
	else
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
		   (TEXT("unknown vap: %s\n"), name));
		return -1;
	}
}

int ieee80211_useonly_rates(struct ieee80211_useonly_rates *ur, 
		struct iwreq *wrq)
{
	uint8 nrates = wrq->u.data.length; 
	uint8 orates[IEEE80211_RATE_MAXSIZE] = {0};
	uint8 rs_rates[IEEE80211_RATE_MAXSIZE] = {0x02, 0x04, 0x0b,
		0x16, 0x0c, 0x12,
		0x18, 0x24, 0xFF,
		0xFF, 0xFF, 0xFF};
	int i, j, k = 0;
	enum ieee80211_phymode mode = 0;

	if (copy_from_user(&orates, wrq->u.data.pointer, 
				wrq->u.data.length)) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
				(TEXT("%s: copy_from_user failed %d\n"), 
				 __func__, __LINE__));
		return -EINVAL;
	}

	for (i = 0; i < nrates; i++) {
		for (j = 0; j < IEEE80211_RATE_MAXSIZE; j++) {
			if (orates[i] == rs_rates[j]) { 
				ur->ur_rates[k++] = orates[i];
				if (j <= 3)
					mode = IEEE80211_MODE_11B;
				else if (j > 3 && j <= 8) 
					mode = IEEE80211_MODE_11G;
			}
		}
	}
	ur->ur_nrates = k;
	ur->ur_mode = mode;

	return k;
}

/**
 * This function creates a virtual ap.This is public as it must be
 * implemented outside our control (e.g. in the driver).
 * @param  Pointer to the ieee80211com structure
 * @param  Pointer to the ifreq structure
 * @param  Pointer to the netdevice structure
 * @return Success or failure  
 */
int ieee80211_ioctl_create_vap(struct ieee80211com *ic, struct ifreq *ifr,
                                struct net_device *mdev)
{
	struct ieee80211_clone_params cp;
	struct ieee80211vap *vap;
	char name[IFNAMSIZ];
	WLAN_ADAPTER w_adapter = (WLAN_ADAPTER)netdev_priv(mdev);
	//uint8 vap_id;

	if (!capable(CAP_NET_ADMIN))
	{
		return -EPERM;
	}
	if (copy_from_user(&cp, ifr->ifr_data, sizeof(cp)))
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Copy from user failed\n")));
		return -EFAULT;
	}

	strncpy(name, cp.icp_parent, sizeof(name));
	if((cp.icp_opmode == 1) && (w_adapter->beacon_recv_disable == 1))
	{
		/* If Vap is created in station mode and if beacon_receive is disabled, we need to enable it. */
		w_adapter->devdep_ops->conf_beacon_recv(w_adapter, 0);
	}
#if 0
	vap_id = w_adapter->os_intf_ops->onebox_extract_vap_id(name);
	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
	{
		if(vap && (vap->hal_priv_vap->vap_id == vap_id)) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Virtual Interface with similar name is already created\n")));
			return -EFAULT;
		}
	}
#endif
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Name for vap creation is: %s rtnl %d\n"), name, rtnl_is_locked()));
	/*  */ //Check 3,6,7 param whether it should be 0 or not
	vap = w_adapter->core_ops->onebox_create_vap(ic, name, 0, cp.icp_opmode, cp.icp_flags, NULL, w_adapter->mac_addr);
	if (vap == NULL)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("VAP = NULL\n")));
		return -EIO;
	}

	/*
 	 * If AP is the first vap and the user has not set a reg domain,
 	 * setting the reg domain to default FCC / United States.
 	 */
	if (!w_adapter->sc_nvaps) {
#ifdef ONEBOX_CONFIG_CFG80211
		TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
			if (vap->iv_opmode == IEEE80211_M_HOSTAP && 
			    ic->ic_regdomain.country == CTRY_DEFAULT) {
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Setting regdom to US\n")));
				ic->ic_regdomain.country = CTRY_UNITED_STATES;
				ic->ic_regdomain.location = ' ';
				ic->ic_regdomain.isocc[0] = 'U';
				ic->ic_regdomain.isocc[1] = 'S';
				ic->ic_regdomain.pad[0] = 0;			
				w_adapter->net80211_ops->onebox_media_init(ic);
			}
			break;
		}
#endif
	}

	/* return final device name */
	strncpy(ifr->ifr_name, vap->iv_ifp->name, IFNAMSIZ);
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		(TEXT("created vap: %s\n"), name));
	return 0;
}

bool check_valid_bgchannel(uint16 *data_ptr, uint8_t supported_band)
{
	uint8_t ii, jj;
	uint8_t num_chan = *((uint8 *)(data_ptr) + 6) ;
	uint16_t chan_5g[] = {36, 40, 44, 48, 149, 153, 157, 161, 165};
	uint16_t chan_check[num_chan];

	memcpy(chan_check, (uint16 *)(data_ptr + 6), 2*num_chan);

	if (!supported_band) {
		for (ii = 0; ii < num_chan; ii++) {
			for (jj = 0; jj < num_chan; jj++) {
				if (chan_check[ii] == chan_5g[jj]) {
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERROR: Trying to program 5GHz channel on a card supporting only 2.4GHz\n")));
					return false;
				}
			}
		}
	}

	return true;
}

static void send_sleep_req_in_per_mode(WLAN_ADAPTER w_adapter, uint8 *data)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status = 0;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];
	struct pwr_save_params ps_params_ioctl;//Parameters to store IOCTL parameters from USER
	struct driver_assets *d_assets = w_adapter->d_assets;
	uint8 request =1;

	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	//memcpy(&ps_params_ioctl, data, sizeof(struct pwr_save_params));
	if(copy_from_user(&ps_params_ioctl, data, sizeof(struct pwr_save_params)))
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
		return;
	}

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, MAX_MGMT_PKT_SIZE);
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(sizeof(mgmt_frame->u.ps_req_params) | (ONEBOX_WIFI_MGMT_Q << 12));
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(WAKEUP_SLEEP_REQUEST);

	mgmt_frame->u.ps_req_params.ps_req.sleep_type = ps_params_ioctl.sleep_type; //LP OR ULP
	mgmt_frame->u.ps_req_params.listen_interval = ps_params_ioctl.listen_interval;
	mgmt_frame->u.ps_req_params.ps_req.sleep_duration = ps_params_ioctl.deep_sleep_wakeup_period;
	mgmt_frame->u.ps_req_params.ps_req.ps_en = ps_params_ioctl.ps_en;
	mgmt_frame->u.ps_req_params.ps_req.connected_sleep = DEEP_SLEEP;

	if(!ps_params_ioctl.ps_en) {
		mgmt_frame->desc_word[0] |= 1 << 15; //IMMEDIATE WAKE UP
	}

	ONEBOX_DEBUG(ONEBOX_ZONE_PWR_SAVE, (" <==== Sending Power save request =====> In %s Line %d  \n", __func__, __LINE__));
	status = w_adapter->devdep_ops->onebox_send_internal_mgmt_frame(w_adapter,
			(uint16 *)mgmt_frame,
			FRAME_DESC_SZ + sizeof(mgmt_frame->u.ps_req_params));
	if (d_assets->host_intf_type == HOST_INTF_SDIO) {
		msleep (2);
		ONEBOX_DEBUG (ONEBOX_ZONE_ERROR,
				(TEXT ("Writing disable to wakeup register\n")));
		status =
			d_assets->onebox_common_write_register (d_assets, 0, SDIO_WAKEUP_REG,
					&request);
	}
	return ;
}

ONEBOX_STATUS send_wowlan_params(struct ieee80211vap *vap, WLAN_ADAPTER w_adapter)
{
	onebox_mac_frame_t *mgmt_frame;
	ONEBOX_STATUS status = 0;
	uint8  pkt_buffer[MAX_MGMT_PKT_SIZE];

	mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, MAX_MGMT_PKT_SIZE);
	mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(sizeof(mgmt_frame->u.wowlan_params) | (ONEBOX_WIFI_MGMT_Q << 12));
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(WOWLAN_CONFIG_PARAMS);
	memcpy(&mgmt_frame->u.wowlan_params, &vap->hal_priv_vap->wowlan_params, sizeof(struct wowlan_config));

	ONEBOX_DEBUG(ONEBOX_ZONE_PWR_SAVE, (" <==== Sending WOWLAN PARAMS =====> In %s Line %d  \n", __func__, __LINE__));
	status = w_adapter->devdep_ops->onebox_send_internal_mgmt_frame(w_adapter,
			(uint16 *)mgmt_frame,
			FRAME_DESC_SZ + sizeof(mgmt_frame->u.wowlan_params));
	return status;

}

static ONEBOX_STATUS wlan_deregister_fw(WLAN_ADAPTER w_adapter)
{
	onebox_mac_frame_t *mgmt_frame;
	netbuf_ctrl_block_t *netbuf_cb = NULL;
	ONEBOX_STATUS status = ONEBOX_STATUS_SUCCESS;
	struct driver_assets *d_assets = w_adapter->d_assets;
	d_assets->dreg_frame = 1;

	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
			(TEXT("===> Deregister WLAN FW <===\n")));

	netbuf_cb = w_adapter->os_intf_ops->onebox_alloc_skb(FRAME_DESC_SZ);
	if(netbuf_cb == NULL)
	{	
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
		status = ONEBOX_STATUS_FAILURE;
		return status;
	}
	w_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, FRAME_DESC_SZ);
    /* Aligning the netbuf->data pointer to avoid issues on embedded platforms */

	mgmt_frame = (onebox_mac_frame_t *)netbuf_cb->data;

	w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);

	/* FrameType*/
	mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(WLAN_DE_REGISTER);
#define IMMEDIATE_WAKEUP 1
	mgmt_frame->desc_word[0] = ((ONEBOX_WIFI_MGMT_Q << 12)| (IMMEDIATE_WAKEUP << 15));
	netbuf_cb->tx_pkt_type = WLAN_TX_M_Q;

	ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("<==== DEREGISTER FRAME ====>\n")));
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)mgmt_frame, FRAME_DESC_SZ);
	status = w_adapter->onebox_send_pkt_to_coex(w_adapter->d_assets, netbuf_cb, WLAN_Q);
	if (status != ONEBOX_STATUS_SUCCESS) 
	{ 
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT 
			     ("%s: %d Failed To Write The Packet\n"),__func__, __LINE__));
	}

	if(d_assets->host_intf_type == HOST_INTF_SDIO)
		w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
	return status;
}

#ifdef ONEBOX_CONFIG_PUF
/**
 * This function prepares puf request frame and send it to LMAC
 *
 * @param Pointer to Adapter structure
 * @param Pointer to user space request structure
 * @return 0 if success else -1
 */
ONEBOX_STATUS send_puf_request(WLAN_ADAPTER w_adapter, struct iwreq *wrq)
{
  onebox_mac_frame_t *mgmt_frame;
  netbuf_ctrl_block_t *netbuf_cb = NULL;
  ONEBOX_STATUS status;
  uint16 frame_len, pkt_len, data_size;
  uint8 puf_sub_cmd;
  struct puf_init_params puf_enroll;
  struct puf_init_params puf_start;
  struct puf_set_key_params *puf_set_key;
  struct puf_get_key_params *puf_get_key;
  struct puf_aes_data_params puf_aes_data;

  FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);
  if(copy_from_user(&puf_sub_cmd, wrq->u.data.pointer, 1))
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
    return -EINVAL;
  }
  ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (" <==== Sending PUF Request =====>"));

  switch (puf_sub_cmd)
  {
    case PUF_ENROLL:    //! copying puf_enroll structure from userspace, and preparing mgmt packet and sending to firmware, copy activation code to user space after receving puf event after confirm packet
      if(copy_from_user(&puf_enroll, wrq->u.data.pointer, wrq->u.data.length))
      {
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
        return -EINVAL;
      }
      frame_len = PUF_SUB_CMD_BYTE + PUF_AC_SOURCE_BYTE;
      pkt_len = FRAME_DESC_SZ + frame_len;
      netbuf_cb = w_adapter->os_intf_ops->onebox_alloc_skb(pkt_len);
      if(netbuf_cb == NULL) {
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
        status = ONEBOX_STATUS_FAILURE;
        return status;
      }
      w_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, pkt_len);
      mgmt_frame = (onebox_mac_frame_t *)&netbuf_cb->data[0];
      w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, pkt_len);
      mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(frame_len | (COEX_TX_Q << 12));
      mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(PUF_REQUEST_FRAME);
      mgmt_frame->u.byte.buf[0] = puf_enroll.puf_sub_cmd;
      mgmt_frame->u.byte.buf[1] = puf_enroll.puf_ac_source;
      netbuf_cb->tx_pkt_type = COEX_Q;
      status = w_adapter->d_assets->common_send_pkt_to_coex(w_adapter->d_assets, netbuf_cb, COEX_Q);
      if (status < 0) {
        ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : Failed sending puf enroll request frame%d\n", __func__, __LINE__, status));
        return status;
      }
      status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->d_assets->puf_event), EVENT_WAIT_FOREVER);
      if (status < 0) {
        ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
        return status;
      }
      if (w_adapter->d_assets->puf_status == 0 && puf_enroll.puf_ac_source) {
        if(copy_to_user(puf_enroll.activation_code, w_adapter->d_assets->puf_activation_code, ACTIVATION_CODE_SIZE))
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
          w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->d_assets->puf_event));
          return ONEBOX_STATUS_FAILURE;
        }
      }
      break;
    case PUF_START:   //! copying puf start structure from userspace, and preparing mgmt packet and sending to firmware 
      if(copy_from_user(&puf_start, wrq->u.data.pointer, wrq->u.data.length))
      {
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
        return -EINVAL;
      }
      if (puf_start.puf_ac_source) {
        frame_len = PUF_SUB_CMD_BYTE + PUF_AC_SOURCE_BYTE + ACTIVATION_CODE_SIZE;
      } else {
        frame_len = PUF_SUB_CMD_BYTE + PUF_AC_SOURCE_BYTE;
      }
      pkt_len = FRAME_DESC_SZ + frame_len;
      netbuf_cb = w_adapter->os_intf_ops->onebox_alloc_skb(pkt_len);
      if(netbuf_cb == NULL) {
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
        status = ONEBOX_STATUS_FAILURE;
        return status;
      }
      w_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, pkt_len);
      mgmt_frame = (onebox_mac_frame_t *)&netbuf_cb->data[0];
      w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, pkt_len);
      mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(frame_len | (COEX_TX_Q << 12));
      mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(PUF_REQUEST_FRAME);
      mgmt_frame->u.byte.buf[0] = puf_start.puf_sub_cmd;
      mgmt_frame->u.byte.buf[1] = puf_start.puf_ac_source;
      if (puf_start.puf_ac_source) {
        if(copy_from_user(&netbuf_cb->data[FRAME_DESC_SZ + PUF_SUB_CMD_BYTE + PUF_AC_SOURCE_BYTE], puf_start.activation_code, ACTIVATION_CODE_SIZE))
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
          w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
          return ONEBOX_STATUS_FAILURE;
        }
      }
      netbuf_cb->tx_pkt_type = COEX_Q;
      status = w_adapter->d_assets->common_send_pkt_to_coex(w_adapter->d_assets, netbuf_cb, COEX_Q);
      if (status < 0) {
        ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : Failed sending puf start request frame%d\n", __func__, __LINE__, status));
        return status;
      }
      status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->d_assets->puf_event), EVENT_WAIT_FOREVER);
      if (status < 0) {
        ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
        return status;
      }
      break;
    case PUF_SET_KEY:    //! copying puf set key structure from userspace, and preparing mgmt packet and sending to firmware, copy key code to user space after receving puf event after confirm packet 
    case PUF_SET_INTRINSIC_KEY:
      frame_len = sizeof(mgmt_frame->u.puf_set_key_params);
      pkt_len = FRAME_DESC_SZ + frame_len;
      netbuf_cb = w_adapter->os_intf_ops->onebox_alloc_skb(pkt_len);
      if(netbuf_cb == NULL) {
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
        status = ONEBOX_STATUS_FAILURE;
        return status;
      }
      w_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, pkt_len);
      mgmt_frame = (onebox_mac_frame_t *)&netbuf_cb->data[0];
      w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, pkt_len);
      mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(frame_len | (COEX_TX_Q << 12));
      mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(PUF_REQUEST_FRAME);
      if(copy_from_user(&mgmt_frame->u.puf_set_key_params, wrq->u.data.pointer, frame_len))
      {
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
        w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
        return ONEBOX_STATUS_FAILURE;
      }
      netbuf_cb->tx_pkt_type = COEX_Q;
      status = w_adapter->d_assets->common_send_pkt_to_coex(w_adapter->d_assets, netbuf_cb, COEX_Q);
      if (status < 0) {
        ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : Failed sending puf set key request frame%d\n", __func__, __LINE__, status));
        return status;
      }
      status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->d_assets->puf_event), EVENT_WAIT_FOREVER);
      if (status < 0) {
        ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
        return status;
      }
      if (w_adapter->d_assets->puf_status == 0) {
        puf_set_key = (struct puf_set_key_params *)wrq->u.data.pointer;
        //w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, w_adapter->d_assets->puf_key_code, KEY_CODE_SIZE);
        if(copy_to_user(puf_set_key->key_code, w_adapter->d_assets->puf_key_code, KEY_CODE_SIZE))
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
          w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->d_assets->puf_event));
          return ONEBOX_STATUS_FAILURE;
        }
      }
      break;
    case PUF_GET_KEY:   //! copying puf get key structure from userspace, and preparing mgmt packet and sending to firmware, copy key to user space after receving puf event after confirm packet
    case PUF_LOAD_KEY:
      frame_len = sizeof(mgmt_frame->u.puf_get_key_params);
      pkt_len = FRAME_DESC_SZ + frame_len;
      netbuf_cb = w_adapter->os_intf_ops->onebox_alloc_skb(pkt_len);
      if(netbuf_cb == NULL) {
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
        status = ONEBOX_STATUS_FAILURE;
        return status;
      }
      w_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, pkt_len);
      mgmt_frame = (onebox_mac_frame_t *)&netbuf_cb->data[0];
      w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, pkt_len);
      mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(frame_len | (COEX_TX_Q << 12));
      mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(PUF_REQUEST_FRAME);
      if(copy_from_user(&mgmt_frame->u.puf_get_key_params, wrq->u.data.pointer, frame_len))
      {
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
        w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
        return ONEBOX_STATUS_FAILURE;
      }
      netbuf_cb->tx_pkt_type = COEX_Q;
      status = w_adapter->d_assets->common_send_pkt_to_coex(w_adapter->d_assets, netbuf_cb, COEX_Q);
      if (status < 0) {
        ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : Failed sending puf get key request frame%d\n", __func__, __LINE__, status));
        return status;
      }
      status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->d_assets->puf_event), EVENT_WAIT_FOREVER);
      if (status < 0) {
        ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
        return status;
      }
      if (w_adapter->d_assets->puf_status == 0 && puf_sub_cmd == PUF_GET_KEY) {
        puf_get_key = (struct puf_get_key_params *)wrq->u.data.pointer;
        if(copy_to_user(&puf_get_key->key_size, &w_adapter->d_assets->puf_recv_len, 2))
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
          w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->d_assets->puf_event));
          return ONEBOX_STATUS_FAILURE;
        }
        //w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, w_adapter->d_assets->puf_key, w_adapter->d_assets->puf_recv_len);
        if(copy_to_user(puf_get_key->key, w_adapter->d_assets->puf_key, w_adapter->d_assets->puf_recv_len))
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
          w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->d_assets->puf_event));
          return ONEBOX_STATUS_FAILURE;
        }
      }
      break;
    case PUF_AES_ENCRYPT:  //! copying puf aes data structure from userspace, and preparing mgmt packet and sending to firmware, copy encoded/decoded/mac data to user space after receving puf event after confirm packet
    case PUF_AES_DECRYPT:
    case PUF_AES_MAC:
      if(copy_from_user(&puf_aes_data, wrq->u.data.pointer, wrq->u.data.length))
      {
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
        return -EINVAL;
      }
      frame_len = PUF_SUB_CMD_BYTE + MODE_BYTE + (MAX_KEY_SIZE + MAX_IV_SIZE + KEY_SIZE + IV_SIZE) + puf_aes_data.data_size;
      pkt_len = FRAME_DESC_SZ + frame_len;
      netbuf_cb = w_adapter->os_intf_ops->onebox_alloc_skb(pkt_len);
      if(netbuf_cb == NULL) {
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
        status = ONEBOX_STATUS_FAILURE;
        return status;
      }
      w_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, pkt_len);
      mgmt_frame = (onebox_mac_frame_t *)&netbuf_cb->data[0];
      w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, pkt_len);
      mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(frame_len | (COEX_TX_Q << 12));
      mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(PUF_REQUEST_FRAME);
      mgmt_frame->u.byte.buf[0] = puf_aes_data.puf_sub_cmd;
      mgmt_frame->u.byte.buf[1] = ((puf_aes_data.mode << MODE_BIT) | (puf_aes_data.key_size << KEY_SIZE_BIT) | (puf_aes_data.key_source << KEY_SOURCE_BIT) | (puf_aes_data.iv_size << IV_SIZE_BIT)); //! writting Four Info Mode,key_size,key_source & iv_size as BIT field. It is expected in Firmware in same format for LLD API's.
      memcpy(&netbuf_cb->data[FRAME_DESC_SZ + PUF_SUB_CMD_BYTE + MODE_INFO], &puf_aes_data.key, (MAX_KEY_SIZE + MAX_IV_SIZE )); 
      data_size = ONEBOX_CPU_TO_LE16(puf_aes_data.data_size);
      memcpy(&netbuf_cb->data[FRAME_DESC_SZ + PUF_SUB_CMD_BYTE + MODE_INFO + MAX_KEY_SIZE + MAX_IV_SIZE], &data_size, DATA_SIZE);
      if (copy_from_user(&netbuf_cb->data[FRAME_DESC_SZ + PUF_SUB_CMD_BYTE + MODE_INFO + MAX_KEY_SIZE + MAX_IV_SIZE + DATA_SIZE], puf_aes_data.data, puf_aes_data.data_size)) 
      {
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
        w_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
        return ONEBOX_STATUS_FAILURE;
      }
      netbuf_cb->tx_pkt_type = COEX_Q;
      status = w_adapter->d_assets->common_send_pkt_to_coex(w_adapter->d_assets, netbuf_cb, COEX_Q);
      if (status < 0) {
        ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : Failed sending puf get key request frame%d\n", __func__, __LINE__, status));
        return status;
      }
      status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->d_assets->puf_event), EVENT_WAIT_FOREVER);
      if (status < 0) {
        ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
        return status;
      }
      if (w_adapter->d_assets->puf_status == 0) {
        if(copy_to_user(puf_aes_data.enc_dec_mac_data, w_adapter->d_assets->enc_dec_mac_data, w_adapter->d_assets->puf_recv_len))
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
          return ONEBOX_STATUS_FAILURE;
        }
      }
      break;
    case PUF_BLOCK_ENROLL:      //! preparing mgmt packet for sending to firmware
    case PUF_BLOCK_SET_KEY:
    case PUF_BLOCK_GET_KEY:
      frame_len = PUF_SUB_CMD_BYTE;
      pkt_len = FRAME_DESC_SZ + frame_len;
      netbuf_cb = w_adapter->os_intf_ops->onebox_alloc_skb(pkt_len);
      if(netbuf_cb == NULL) {
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: Unable to allocate skb\n"), __func__));
        status = ONEBOX_STATUS_FAILURE;
        return status;
      }
      w_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, pkt_len);
      mgmt_frame = (onebox_mac_frame_t *)&netbuf_cb->data[0];
      w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, pkt_len);
      mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(frame_len | (COEX_TX_Q << 12));
      mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(PUF_REQUEST_FRAME);
      mgmt_frame->u.byte.buf[0] = puf_sub_cmd;
      netbuf_cb->tx_pkt_type = COEX_Q;
      status = w_adapter->d_assets->common_send_pkt_to_coex(w_adapter->d_assets, (netbuf_ctrl_block_t *)mgmt_frame, COEX_Q);
      if (status < 0) {
        ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : Failed sending puf start request frame%d\n", __func__, __LINE__, status));
        return status;
      }
      status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->d_assets->puf_event), EVENT_WAIT_FOREVER);
      if (status < 0) {
        ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
        return status;
      }
      break;
    default:
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid PUF command \n")));
      return status;
  }
  w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->d_assets->puf_event));
  if (w_adapter->d_assets->puf_status != 0) {
    status = ONEBOX_STATUS_FAILURE;
  }
  return status;
}
#endif

/**
 *  Calls the corresponding (Private) IOCTL functions
 *
 * @param  pointer to the net_device
 * @param  pointer to the ifreq
 * @param  value of the ioctl command, input to this function
 * @return returns 0 on success otherwise returns the corresponding 
 * error code for failure
 */
int onebox_ioctl(struct net_device *dev,struct ifreq *ifr, int cmd)
{
	WLAN_ADAPTER w_adapter = NULL;
	struct ieee80211com *ic = NULL;
	struct ieee80211_useonly_rates *ur;
	struct iwreq *wrq = (struct iwreq *)ifr;
	int index, priv, ret_val=0;
	struct ieee80211_node *ni = NULL;
	struct ieee80211vap *vap = NULL;
	struct ieee80211vap *vap_ap = NULL;
	uint8_t macaddr[IEEE80211_ADDR_LEN] = {0};
	uint8_t no_of_vaps = 0; 
	unsigned int value = 0;
	unsigned int channel = 1;
	unsigned int status;
	unsigned char sta_state;
	onebox_mac_frame_t *mgmt_frame;
	struct test_mode test;
	struct get_info getinfo;
	struct get_info *get_ptr;
	int found = 0;
	struct wowlan_config *wowlan_params = NULL;
	struct driver_assets *d_assets = NULL;
	//uint32_t enable = 0;
	uint32_t protocol_status = 0;
	driver_params_t driver_params;
	unsigned short country_code;
	uint16_t rx_filter_word;
    uint32_t buf = 0;
  coex_cmd_t coex_cmd;

  programming_stats_t programming_stats;
  prog_structure_t *prog_structure;
	unsigned char buffer = 0;
  int no_of_samples = 0;
	char rc;
    ipmu_params_t *ipmu_params;
    gpio_reg_t *gpio_registers;

#ifdef RADAR_AUTO
	uint64 radar_intr_state;
	struct radar_app *radar_pkt_to_app,*temp;
#endif
#ifdef ONEBOX_CONFIG_GTK_OFFLOAD
  struct ieee80211_gtk_rekey_data gtk_rekey_data;
#endif

	w_adapter = (WLAN_ADAPTER)netdev_priv(dev);
	if (w_adapter == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
			(TEXT("%s: Invalid w_adapter\n"), __func__));
		return -1;
	}

	ic = &w_adapter->vap_com;
	d_assets =  w_adapter->d_assets;
	if (ic == NULL || d_assets == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
		   (TEXT("%s: Invalid pointer ic %p d_assets %p cmd %x\n"), 
		   __func__, ic, d_assets, cmd));
		return -1;
	}
	
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO, ("In onebox_ioctl function\n"));
	/* Check device is present or not */
	if (!netif_device_present(dev))
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Device not present\n")));
		return -ENODEV;
	}

	if ((w_adapter->fsm_state != FSM_MAC_INIT_DONE) &&
             (cmd != SIOCGPROTOCOL) && (cmd != SIOCGIWNAME) && (cmd != ONEBOX_SET_BB_RF)) {
        return -EBUSY;
    }

	/* Get the IOCTL index */
	index = get_ioctl_index(cmd, &priv);

	/*vap creation command*/
	switch(cmd)
	{
		case RSI_WATCH_IOCTL:
		{
			wrq->u.data.length = 4;
			if (w_adapter->buffer_full)
			{
				w_adapter->watch_bufferfull_count++;

				if (w_adapter->watch_bufferfull_count > 10) /*  : not 10, should dependent on time */
				{
					/* Incase of continous buffer full, give the last beacon counter */
					ret_val = copy_to_user(wrq->u.data.pointer, &w_adapter->total_beacon_count, 4);
					return ret_val;
				}
			}

			ret_val = copy_to_user(wrq->u.data.pointer, &w_adapter->total_beacon_count, 4);
			return ret_val;
		} 
		break;
		case ONEBOX_VAP_CREATE:
		{
			if ((w_adapter->Driver_Mode != WIFI_MODE_ON) && (w_adapter->Driver_Mode != SNIFFER_MODE))
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Driver Mode is not in WIFI_MODE vap creation is not Allowed\n")));	
				return ONEBOX_STATUS_FAILURE;
			}
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (" VAP Creation \n"));
			ret_val = ieee80211_ioctl_create_vap(ic, ifr, dev);

			if(ret_val == 0)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Created VAP with dev name:%s\n"),ifr->ifr_name));
			}
			return ret_val;
		}
		break;
		case ONEBOX_VAP_DELETE:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (" VAP delete \n"));
			ret_val = ieee80211_ioctl_delete_vap(ic, ifr, dev);
			if(ret_val == 0)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Deleted VAP with dev name:%s\n"),ifr->ifr_name));
			}
			return ret_val;
		}
#define IS_RUNNING(ifp) \
((ifp->if_flags & IFF_UP) && (ifp->if_drv_flags & IFF_DRV_RUNNING))
		case SIOCSIFFLAGS:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In SIOCSIFFLAGS case dev->flags =%x\n"), dev->if_flags));
			/* Not doing anything here */
			if (IS_RUNNING(dev)) 
			{
				/* Nothing to be done here */
			} 
			else if (dev->if_flags & IFF_UP) 
			{
				dev->if_drv_flags |= IFF_DRV_RUNNING;
				ieee80211_start_all(ic);
			} 
			else 
			{
				dev->if_drv_flags &= ~IFF_DRV_RUNNING;
			} 
			return ret_val;
		}
		break;
		case SIOCGPROTOCOL:
        {
            protocol_status = wrq->u.data.length;
            switch((uint32_t)(wrq->u.data.flags)) 
            {
                case PROTOCOL_ENABLE:
                    {
                        if (protocol_status & WLAN_PROTOCOL) {
                            if(!(d_assets->protocol_enabled & WLAN_PROTOCOL) && 
                                    (d_assets->techs[WLAN_ID].fw_state == FW_ACTIVE)) {
                                w_adapter->fsm_state = FSM_LOAD_BOOTUP_PARAMS ;
                                d_assets->protocol_enabled |= WLAN_PROTOCOL;
                                d_assets->dreg_frame = 0 ;
                                if (onebox_load_bootup_params(w_adapter) != ONEBOX_STATUS_SUCCESS) {
                                    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT ("%s: Failed to load bootup parameters\n"), 
                                                __func__));
                                    return -1;
                                }

                                ONEBOX_DEBUG(ONEBOX_ZONE_FSM,
                                        (TEXT("%s: BOOTUP Parameters loaded successfully\n"),__func__));
                            } else {
                                ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
                                        (TEXT("%s: %d Error: WLAN firmware is not active\n"),__func__, __LINE__));
                                return -1;
                            }
                        }

#ifdef BT_ENABLE
                        if (protocol_status & BT_PROTOCOL) { 
                            if(!(d_assets->protocol_enabled & BT_PROTOCOL) &&
                                    (d_assets->techs[BT_ID].fw_state == FW_ACTIVE) && 
                                    (d_assets->techs[BT_ID].drv_state != MODULE_ACTIVE)
                              ) {
                                d_assets->techs[BT_ID].inaugurate(d_assets);
                                d_assets->protocol_enabled |= BT_PROTOCOL;
                            } else {
                                ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
                                        (TEXT("%s: %d Error: BT firmware is not active\n"),__func__, __LINE__));
                                return -1;
                            }
                        }
#endif
#ifdef ZIGB_ENABLE
                        if (protocol_status & ZIGB_PROTOCOL) { 
                            if (!(d_assets->protocol_enabled & ZIGB_PROTOCOL) &&
                                    (d_assets->techs[ZB_ID].fw_state == FW_ACTIVE) && 
                                    (d_assets->techs[ZB_ID].drv_state != MODULE_ACTIVE)) {
                                d_assets->techs[ZB_ID].inaugurate(d_assets);
                                d_assets->protocol_enabled |= ZIGB_PROTOCOL;
                            } else {
                                ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
                                        (TEXT("%s: %d Error: ZIGB firmware is not active\n"),__func__, __LINE__));
                                return -1;
                            }
                        }
#endif
                    } 
                    break;
                case PROTOCOL_DISABLE: 
                    {
                        if ((protocol_status & WLAN_PROTOCOL) &&
                                (d_assets->protocol_enabled & WLAN_PROTOCOL)) {
                            TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
                                ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Deleting vap\n")));
                                ic->ic_vap_delete(vap, 0);
                            }
                            core_net80211_detach(w_adapter);
                            d_assets->techs[WLAN_ID].tx_intention = 1;
                            d_assets->update_tx_status(d_assets, WLAN_ID);

                            if(!d_assets->techs[WLAN_ID].tx_access) {
                                ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Waiting for tx_acces from common hal cmntx %d\n"), d_assets->common_hal_tx_access));
                                d_assets->techs[WLAN_ID].deregister_flags = 1;
                                ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Waiting event %s Line %d\n"), __func__, __LINE__));
                                if (wait_event_timeout((d_assets->techs[WLAN_ID].deregister_event), (d_assets->techs[WLAN_ID].deregister_flags == 0), msecs_to_jiffies(6000) )) {
                                    if(!d_assets->techs[WLAN_ID].tx_access) {
                                        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d unable to get access \n"), __func__, __LINE__));
                                        return -1;
                                    }
                                } else {
                                    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERR: In %s Line %d Initialization of WLAN Failed as Wlan TX access is not granted from Common Hal \n"), __func__, __LINE__));
                                    return -1;
                                }
                            }
                            if(d_assets->card_state != GS_CARD_DETACH)	
                                wlan_deregister_fw(w_adapter);

                            d_assets->techs[WLAN_ID].tx_intention = 0;
                            d_assets->techs[WLAN_ID].tx_access = 0;
                            d_assets->update_tx_status(d_assets, WLAN_ID);
                            d_assets->techs[WLAN_ID].fw_state = FW_INACTIVE;
                            d_assets->protocol_enabled &= ~WLAN_PROTOCOL;
                        }

#ifdef BT_ENABLE
                        if ((protocol_status & BT_PROTOCOL) &&
                                (d_assets->protocol_enabled & BT_PROTOCOL) &&
                                (d_assets->techs[BT_ID].drv_state == MODULE_ACTIVE)) {
                            d_assets->techs[BT_ID].disconnect(d_assets);
                            d_assets->protocol_enabled &= ~BT_PROTOCOL;
                        }
#endif
#ifdef ZIGB_ENABLE
                        if ((protocol_status & ZIGB_PROTOCOL) &&
                                (d_assets->protocol_enabled & ZIGB_PROTOCOL) &&
                                (d_assets->techs[ZB_ID].drv_state == MODULE_ACTIVE)) {
                            d_assets->techs[ZB_ID].disconnect(d_assets);
                            d_assets->protocol_enabled &= ~ZIGB_PROTOCOL;
                        }
#endif

                    } 
                    break;
                case WLAN_9116_FEATURE:
                    {
                      if (w_adapter->device_model == RSI_DEV_9116) {
                        if(copy_from_user(&d_assets->w_9116_features, wrq->u.data.pointer, wrq->u.data.length))
                        {
                          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
                          return -EINVAL;
                        }
                        onebox_send_w_9116_features(w_adapter);
                      } else {
                          return -EINVAL;
                      }
                    }
                    break;
                case DISABLE_PROGRAMMING :
                    {
                        buffer =0 ;
                        if(copy_from_user(&buffer, wrq->u.data.pointer, wrq->u.data.length))
                        {
                            ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
                            return -EINVAL;
                        }
                        onebox_send_disable_programming(w_adapter,buffer);
                        if((buffer & 0x7 ) == 0x7) {
                            d_assets->disable_programming =1 ;
                        } else {
                            d_assets->disable_programming = 0;
                        }
                    }
                    break;
                case LOG_STRUCT_PRGMG:
                    {
                        w_adapter->os_intf_ops->onebox_memset(&programming_stats, 0, sizeof(programming_stats_t));
                        if(copy_from_user(&programming_stats, wrq->u.data.pointer, wrq->u.data.length))
                        {
                            ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
                            return -EINVAL;
                        }
                        onebox_send_structure_prog_stats_request(w_adapter,&programming_stats);
                    }
                    break; 

                case MASTER_READ:
                    {
                        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("performing master read\n")));
                        if(w_adapter->devdep_ops->onebox_do_master_ops(w_adapter, wrq->u.data.pointer, ONEBOX_MASTER_READ) != ONEBOX_STATUS_SUCCESS)
                        {
                            ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT(" Data Read Failed\n")));
                        }
                    }
                    break;
                case MASTER_WRITE:
                    {
                        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("performing master write \n")));
                        if(w_adapter->devdep_ops->onebox_do_master_ops(w_adapter, wrq->u.data.pointer, ONEBOX_MASTER_WRITE) != ONEBOX_STATUS_SUCCESS)
                        {
                            ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT(" Data Write Failed\n")));
                        }
                    }
                    break;
				if(w_adapter->device_model == RSI_DEV_9116) {
                case IPMU_REG:
                    {
                        ipmu_params = kzalloc(sizeof(ipmu_params_t), GFP_KERNEL);
                        if(ipmu_params == NULL){
                            ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:No Memory\n"), __func__, __LINE__));
                            return -ENOMEM;
                        }
                        if(copy_from_user(ipmu_params, wrq->u.data.pointer, (sizeof(ipmu_params_t))))
                        {
                            ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
                            return -EINVAL;
                        }
                        buf = UNGATE_CLOCK; 
                        if( d_assets->onebox_common_master_reg_write(d_assets, M4_CLK_ADDR,buf, 4) < 0 ) {
                            ret_val = ONEBOX_STATUS_FAILURE;
                            goto IPMU_READ_END;
                        }
                        if(ipmu_params->mode < 2) {
                            ipmu_params->address = IPMU_DIRECT_ACCESS(ipmu_params->address);
                        } else { 
                            ipmu_params->address = IPMU_SPI_ACCESS(ipmu_params->address);
                        }
                        if(ipmu_params->mode == 1 || ipmu_params->mode == 3 )  {     
                            if( d_assets->onebox_common_master_reg_write(d_assets,ipmu_params->address,ipmu_params->value, 4) < 0 ) {
                                ret_val = ONEBOX_STATUS_FAILURE;
                                goto IPMU_READ_END;
                            }
                        } else {
                            if( d_assets->onebox_common_master_reg_read(d_assets,ipmu_params->address,&(ipmu_params->value), 4) < 0 ) {
                                ret_val = ONEBOX_STATUS_FAILURE;
                                goto IPMU_READ_END;
                            }
                            if (copy_to_user(wrq->u.data.pointer, ipmu_params, sizeof(ipmu_params_t)))
                                ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERROR copying ipmu data to user\n")));
                        }
                        buf = GATE_CLOCK; 
                        if( d_assets->onebox_common_master_reg_write(d_assets, M4_CLK_ADDR,buf, 4) < 0 ) {
                            ret_val = ONEBOX_STATUS_FAILURE;
                            goto IPMU_READ_END;
                        }
IPMU_READ_END:
                        kfree(ipmu_params);
                        return ret_val;
                    }
                    break;
                case GPIO_R_W:
                    {
                        gpio_registers = kzalloc(sizeof(gpio_reg_t), GFP_KERNEL);
                        if(gpio_registers == NULL) {
                            ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:No Memory\n"), __func__, __LINE__));
                            return -ENOMEM;
                        }
                        if(copy_from_user(gpio_registers, wrq->u.data.pointer,(sizeof(gpio_reg_t))))
                        {
                            ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
                            return -EINVAL;
                        }
                        if(gpio_registers->id >= 64 ) {
                            if (d_assets->onebox_map_ulp_gpio_to_ta_gpio(d_assets, gpio_registers->id) < ONEBOX_STATUS_SUCCESS) {
                                ret_val = ONEBOX_STATUS_FAILURE;
                                goto GPIO_R_W_END;
                            }
                        }
                        gpio_registers->address = GPIO_REG(gpio_registers->id);
                        if(gpio_registers->read_write) {
                            gpio_registers->value = (gpio_registers->mode|gpio_registers->value << 4 |gpio_registers->direction << 5);
                            if( d_assets->onebox_common_master_reg_write(d_assets,gpio_registers->address,gpio_registers->value, 2) < 0 ) 
                            {
                                ret_val = ONEBOX_STATUS_FAILURE;
                                goto GPIO_R_W_END;
                            }
                        } else {
                            if(d_assets->onebox_common_master_reg_read(d_assets,gpio_registers->address,(uint32*)&(gpio_registers->value), 2) < 0 ) 
                            {
                                ret_val = ONEBOX_STATUS_FAILURE;
                                goto GPIO_R_W_END;
                            }
                            if (copy_to_user(wrq->u.data.pointer, gpio_registers, sizeof(gpio_reg_t)))
                                ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERROR copying ipmu data to user\n")));
                        }
GPIO_R_W_END:
                        kfree(gpio_registers);
                        return ret_val;
                    }
                    break;
                case COEX_SLOT_PARAMS:
                    {
                        if(copy_from_user(&coex_cmd, wrq->u.data.pointer, wrq->u.data.length))
                        {
                            ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
                            return -EINVAL;
                        }
#define BT_LE 0
#define BT_CLASSIC 1
#define WIFI 2
#define ZIGBEE 3
#define OP_WLAN_STA_MODE	BIT(0)
#define OP_WLAN_AP_MODE		BIT(1)
#define OP_BT_CLASSIC_MODE	BIT(2)
#define OP_BT_LE_MODE		BIT(3)
#define OP_ZIGBEE_MODE		(BIT(4) | BIT(5))
                        if(coex_cmd.start_stop) {
                            switch (coex_cmd.protocol_id) {
                                case BT_LE : {
                                                 if(!(d_assets->oper_mode & OP_BT_LE_MODE)){
                                                     goto BREAK;       
                                                 }
                                             }
                                             break;
                                case BT_CLASSIC: {
                                                     if(!(d_assets->oper_mode & OP_BT_CLASSIC_MODE)){
                                                         goto BREAK;       
                                                     }
                                                 }
                                                 break;
                                case WIFI : {
                                                if(!(d_assets->oper_mode & (OP_WLAN_AP_MODE|OP_WLAN_STA_MODE) )){
                                                    goto BREAK;       
                                                }
                                            }
                                            break;
                                case ZIGBEE : {
                                                  if(!(d_assets->oper_mode & OP_ZIGBEE_MODE)){
                                                      goto BREAK;       
                                                  }
                                              }
                                              break;
                                default: {
BREAK:
                                             ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid protocol_id  = %d [0-4] Allowed,Please check the COEX = %d mode in common_insert.sh \n"),coex_cmd.protocol_id,d_assets->oper_mode));
                                             return -EINVAL;
                                         }
                            }
                        }
                        d_assets->onebox_send_coex_configuration(d_assets,&coex_cmd);
                    } 
                    break;
                case PROG_STRUCTURE: {
                                         if((d_assets->protocol_enabled & WLAN_PROTOCOL) && (w_adapter->fsm_state == FSM_MAC_INIT_DONE))
                                         {
                                             buffer =0 ;
                                             prog_structure = kzalloc(wrq->u.data.length, GFP_KERNEL);
                                             if(prog_structure == NULL){
                                                 ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
                                                 return -ENOMEM;
                                             }
                                             if(copy_from_user(prog_structure, wrq->u.data.pointer, wrq->u.data.length))
                                             {
                                                 ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
                                                 return -EINVAL;
                                             }
                                             w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR,(PUCHAR)prog_structure,wrq->u.data.length );
                                             onebox_send_programming_structs(w_adapter, prog_structure);
                                             kfree (prog_structure);
                                         }
                                     }
                                     break;
									 }
                default:
                                     {
                                         ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Unknown IOCTL %d  \n"), __func__, __LINE__));
                                     }
            }
        }
            break;

        case  WLAN_I_Q_STATS:
            {
                if (w_adapter->device_model == RSI_DEV_9116) {
                    switch((unsigned char)wrq->u.data.flags)
                        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("WLAN IQ \n"));
                    if(copy_from_user(&w_adapter->wlan_iqs_stats, wrq->u.data.pointer,wrq->u.data.length))  //sizeof(wlan_iq_struct_t) in ioctl and driver are differ by 2 bytes
                    {
                        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
                        return -EINVAL;
                    }
                    w_adapter->endpoint_params.per_ch_bw = (w_adapter->wlan_iqs_stats.rate_flags & 0x07);
                    w_adapter->endpoint_params.enable_11j =((w_adapter->wlan_iqs_stats.rate_flags >> 3 )& (0x1));
                    w_adapter->recv_channel = w_adapter->wlan_iqs_stats.freq;
                    w_adapter->endpoint_params.channel = w_adapter->recv_channel;
                    no_of_samples = w_adapter->wlan_iqs_stats.no_of_samples * 4 ;
                    w_adapter->devdep_ops->onebox_band_check(w_adapter);
                    w_adapter->fsm_state = FSM_SCAN_CFM;		
                    w_adapter->devdep_ops->onebox_set_channel(w_adapter,w_adapter->recv_channel);
                    status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->bb_rf_event), EVENT_WAIT_FOREVER);
                    if (status < 0) {
                        ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
                        return status;
                    }
                    w_adapter->fsm_state = FSM_MAC_INIT_DONE;
                    w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->bb_rf_event));
                    onebox_send_wlan_iq_capture_request (w_adapter);
                    w_adapter->wlan_iqs_stats.no_of_samples = no_of_samples;
                    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("Waiting for wlan IQ stats \n"));
                    status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->wlan_iqs_event), EVENT_WAIT_FOREVER);
                    if (status < 0) {
                        ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
                        return status;
                    }
                    w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->wlan_iqs_event));
                    if(w_adapter->wlan_iqs_stats.pkt != NULL){
                        ret_val = copy_to_user(w_adapter->wlan_iqs_stats.iq_stats,(void*)w_adapter->wlan_iqs_stats.pkt,no_of_samples);
                        kfree(w_adapter->wlan_iqs_stats.pkt);
                        w_adapter->wlan_iqs_stats.pkt = NULL;
                    }
                    return ret_val;
                } else {
                    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("Wlan IQ stats not supported for 9113 \n"));
                    return -EINVAL;
                }
            }
            break;
		case ONEBOX_HOST_IOCTL:
		{
			if(w_adapter->Driver_Mode == WIFI_MODE_ON)
			{
				value = wrq->u.data.length;
				switch((unsigned char)wrq->u.data.flags)
				{
					case PER_RECEIVE_STOP:
						w_adapter->recv_stop = 1;
						w_adapter->rx_running = 0;
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("PER_RECEIVE_STOP\n"));
					case PER_RECEIVE:
						if (!w_adapter->rx_running)
						{
							if(!(w_adapter->core_ops->onebox_stats_frame(w_adapter)))
							{
								w_adapter->rx_running = 1;
								if (w_adapter->recv_stop)
								{
									w_adapter->recv_stop = 0;
									w_adapter->rx_running = 0;
									return ONEBOX_STATUS_SUCCESS;
								}  
								status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->stats_event), EVENT_WAIT_FOREVER);
								if (status < 0) {
										ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
										return status;
								}
								w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->stats_event));
								ret_val = copy_to_user(wrq->u.data.pointer, &w_adapter->sta_info, sizeof(per_stats));
								return ret_val;
							}
						}
						else
						{
							status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->stats_event), EVENT_WAIT_FOREVER);
							if (status < 0) {
									ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
									return status;
							}
							w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->stats_event));
							ret_val = copy_to_user(wrq->u.data.pointer, &w_adapter->sta_info, sizeof(per_stats));
							return ret_val;
						}
						break;
					case SET_BEACON_INVL:
						if (IEEE80211_BINTVAL_MIN_AP <= value &&
								value <= IEEE80211_BINTVAL_MAX) 
						{
							ic->ic_bintval = ((value + 3) & ~(0x3));
						} 
						else
						{
							ret_val = EINVAL;
						}
						w_adapter->beacon_interval = ONEBOX_CPU_TO_LE16(((value + 3) & ~(0x3)));
						break;
					case SET_ENDPOINT:
						value = ((unsigned short)wrq->u.data.flags >> 8); //endpoint value 
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("ENDPOINT type is : %d \n"),value));
						if (!w_adapter->band_supported) {
							if (value == 2 || value == 3) {
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERROR: 5GHz endpoint not supported\n")));
								return -EINVAL;
							}
						}
						w_adapter->endpoint = value;
						w_adapter->devdep_ops->onebox_program_bb_rf(w_adapter);
						break;
					case ANT_SEL:
						if (w_adapter->antenna_diversity) {
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
								     (TEXT("ERROR: Trying to select\
									   antenna when antenna diversity\
									   feature is enabled\n")));
							return -EINVAL;
						}
						value = ((unsigned short)wrq->u.data.flags >> 8); //endpoint value 
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("ANT_SEL value is : %d \n"),value));
						w_adapter->devdep_ops->onebox_program_ant_sel(w_adapter, value, CONFIG_ANT_SEL);
						break;
					case ANT_TYPE:
						w_adapter->ant_path = ((unsigned short)wrq->u.data.flags >> 8); //endpoint value 
						if(copy_from_user(&w_adapter->ant_type, wrq->u.data.pointer, 1))
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
							return -EINVAL;
						}
						if( d_assets->onboard_antenna && (w_adapter->ant_path == PATH_ONBOARD) && (w_adapter->ant_type != 1))
						{
								printk("Invalid antenna path configuration, as Onboard antenna already present\n" );
								return ONEBOX_STATUS_FAILURE;
						}
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("ANT_PATH is %d type is : %d \n"),w_adapter->ant_path, w_adapter->ant_type));
						w_adapter->devdep_ops->onebox_program_ant_sel(w_adapter, 0, CONFIG_ANT_TYPE);
						break;
					case CHECK_STA_STATE:
						if( w_adapter->sc_nstavaps == 0 ) {
								sta_state = 0xFF;
								ret_val = copy_to_user(wrq->u.data.pointer, &sta_state, 1);
								return ret_val;
						} 
						TAILQ_FOREACH(vap_ap, &ic->ic_vaps, iv_next) 
						{
							if(vap_ap->iv_opmode == IEEE80211_M_HOSTAP) {
								TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
									if(vap->iv_opmode == IEEE80211_M_STA) {
										if( vap_ap->iv_state == IEEE80211_S_RUN ) {
											w_adapter->devdep_ops->onebox_check_scan(w_adapter);
											status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->scan_check_event), 2000); 
											if (status <= 0) {
												ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
												return ONEBOX_STATUS_FAILURE;
											}
											w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->scan_check_event));
											if( w_adapter->scan_state )
												sta_state = IEEE80211_S_SCAN;
											else if( vap->iv_state == IEEE80211_S_SCAN)
												sta_state = IEEE80211_S_INIT;
											else if( vap->iv_state == IEEE80211_S_RUN && vap->hal_priv_vap->conn_in_prog)
											{
												sta_state = IEEE80211_S_ASSOC;
											}
											else 
												sta_state = vap->iv_state;
											ret_val = copy_to_user(wrq->u.data.pointer, &sta_state, 1);
											return ret_val;
										} else if( vap->iv_state == IEEE80211_S_SCAN ) {
											sta_state = IEEE80211_S_SCAN;
											ret_val = copy_to_user(wrq->u.data.pointer, &sta_state, 1);
											return ret_val;
										} else if( vap->iv_state == IEEE80211_S_RUN && vap->hal_priv_vap->conn_in_prog) {
											sta_state = IEEE80211_S_ASSOC;
										} else {
											sta_state = vap->iv_state;
											ret_val = copy_to_user(wrq->u.data.pointer, &sta_state, 1);
											return ret_val;
										}
									}
								}
							} 
						}
						TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
						{
								if((vap->iv_opmode == IEEE80211_M_STA)) {
										if( vap->iv_state == IEEE80211_S_RUN && vap->hal_priv_vap->conn_in_prog) {
												sta_state = IEEE80211_S_ASSOC;
										} else {
												sta_state = vap->iv_state;
										}
										ret_val = copy_to_user(wrq->u.data.pointer, &sta_state, 1);
										return ret_val;
								}
								break;
						}
						sta_state = 0xFF;
						ret_val = copy_to_user(wrq->u.data.pointer, &sta_state, 1);
						return ret_val;
						break;
					case SET_HOST_SCAN:
						no_of_vaps = 0;
						TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
						{
								if( vap->iv_opmode == IEEE80211_M_HOSTAP)
										break;
								else
										no_of_vaps++;
						}
						if( no_of_vaps >= w_adapter->sc_nvaps )
						{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("AP Vap is a MUST for Host scan command \n")));
								return -EINVAL;
						}
						TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
						{ 
								ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" mac addr=%02x:%02x:%02x:%02x:%02x:%02x\n"), macaddr[0], 
														macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]));
								if(vap->iv_opmode == IEEE80211_M_STA)
								{
                 memset(&vap->hal_priv_vap->bgscan_params_ioctl, 0, sizeof(mgmt_frame->u.bgscan_params));

								 if(copy_from_user(&vap->hal_priv_vap->bgscan_params_ioctl, wrq->u.data.pointer, sizeof(mgmt_frame->u.bgscan_params)))
								 {
									 ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
									 return -EINVAL;
								 }

								 if (!check_valid_bgchannel((uint16 *)&vap->hal_priv_vap->bgscan_params_ioctl, 
                        w_adapter->band_supported)) {
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid channel\n")));
									return -EINVAL;
								}

								vap->hal_priv_vap->bgscan_params_ioctl.bg_ioctl = 0;
								vap->iv_host_scan = 1;

								ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("scan: In %s and %d \n"), __func__, __LINE__));
								if(onebox_send_bgscan_params(vap, (uint16 *)&vap->hal_priv_vap->bgscan_params_ioctl , 0))
								{
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("scan: In %s and %d \n"), __func__, __LINE__));
									return ONEBOX_STATUS_FAILURE;
								}

								ni = w_adapter->net80211_ops->onebox_find_node(&vap->iv_ic->ic_sta, vap->iv_myaddr);
								if (ni == NULL)
								{    
									ONEBOX_DEBUG(ONEBOX_ZONE_OID, (TEXT("Ni is null vap node not found\n")));
									return ENOENT;
								}

								send_bgscan_probe_req(w_adapter, ni, BIT(0));
								return 0;
							}
						}
						break;
#ifdef IEEE80211K
					case SET_MSRMNT_PARAMS:  
							TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
							{ 
								if((vap->iv_opmode == IEEE80211_M_STA)  || (vap->iv_opmode == IEEE80211_M_HOSTAP) )
								{
									memset(&vap->hal_priv_vap->msrmnt_req, 0, sizeof( vap->hal_priv_vap->msrmnt_req));
									if(copy_from_user(&vap->hal_priv_vap->msrmnt_req, wrq->u.data.pointer, sizeof(vap->hal_priv_vap->msrmnt_req)))
									{
										ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("in %s line %d:copying failed\n"), __func__, __LINE__));
										return -EINVAL;
									}
									vap->hal_priv_vap->params_11k_set = 1;	

								}
							}
						break;
#endif
#ifdef ONEBOX_CONFIG_GTK_OFFLOAD
          case GTK_OFFLOAD:
            TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next)
            {
              if(vap->iv_opmode == IEEE80211_M_STA)
              {
                if(copy_from_user(&w_adapter->gtk_en, wrq->u.data.pointer, 1))
                { 
		  					  ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
  		  					return -EINVAL;
	  		  			}
                if(!w_adapter->gtk_en) {
                  vap->send_gtk_rekey_data(vap, &gtk_rekey_data);
                }
              }
            }
            break;
#endif
  				case SET_BGSCAN_PARAMS:
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("<<< BGSCAN >>>\n")));
//						onebox_send_bgscan_params(w_adapter, wrq->u.data.pointer , 0);
						TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
						{ 
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" mac addr=%02x:%02x:%02x:%02x:%02x:%02x\n"), macaddr[0], 
										macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]));
							if(vap->iv_opmode == IEEE80211_M_STA)
							{
								memset(&vap->hal_priv_vap->bgscan_params_ioctl, 0, sizeof(mgmt_frame->u.bgscan_params));

								if(copy_from_user(&vap->hal_priv_vap->bgscan_params_ioctl, wrq->u.data.pointer, sizeof(mgmt_frame->u.bgscan_params)))
								{
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
									return -EINVAL;
								}

								if (!check_valid_bgchannel((uint16 *)&vap->hal_priv_vap->bgscan_params_ioctl, w_adapter->band_supported)) {
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid channel in bg param set; 5GHz not supported by card\n")));
									return -EINVAL;
								}
								
								vap->hal_priv_vap->bgscan_params_ioctl.bg_ioctl = 1;
								if(vap->iv_state == IEEE80211_S_RUN && (!w_adapter->sta_mode.delay_sta_support_decision_flag))
								{
									ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Bgscan: In %s and %d \n"), __func__, __LINE__));
									if(onebox_send_bgscan_params(vap, (uint16 *)&vap->hal_priv_vap->bgscan_params_ioctl , 0))
                 							 {
										ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Bgscan: In %s and %d \n"), __func__, __LINE__));
			            				                    return ONEBOX_STATUS_FAILURE;
                  							}
								}
								else
								{
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Bgscan: In %s and %d \n"), __func__, __LINE__));
									return 0;
								}
								ni = w_adapter->net80211_ops->onebox_find_node(&vap->iv_ic->ic_sta, vap->iv_myaddr);
								if (ni == NULL)
								{    
									ONEBOX_DEBUG(ONEBOX_ZONE_OID, (TEXT("Ni is null vap node not found\n")));
									return ENOENT;
								}

								send_bgscan_probe_req(w_adapter, ni, 0);
								return 0;
							}
						}
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Issue IOCTL after vap creation in %s Line %d\n"), __func__, __LINE__));
						return -EINVAL;
						break;
					case DO_BGSCAN:
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("<<< DO BGSCAN IOCTL Called >>>\n")));
							TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
							{ 
								if(vap && (vap->iv_opmode == IEEE80211_M_STA))
								{
									ni = w_adapter->net80211_ops->onebox_find_node(&vap->iv_ic->ic_sta, vap->iv_myaddr);
									if (ni == NULL)
									{    
										ONEBOX_DEBUG(ONEBOX_ZONE_OID, (TEXT("Ni is null vap node not found\n")));
										return ENOENT;
									}
									if(vap && (vap->hal_priv_vap->bgscan_params_ioctl.bg_ioctl))
									{
										if(copy_from_user(&vap->hal_priv_vap->bgscan_params_ioctl.bg_cmd_flags, wrq->u.data.pointer, sizeof(uint16_t)))
										{
											ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
											return -EINVAL;
										}

										if((vap->iv_state == IEEE80211_S_RUN))
										{
											send_bgscan_probe_req(w_adapter, ni, vap->hal_priv_vap->bgscan_params_ioctl.bg_cmd_flags);
										}
										return 0;
									}
									else
									{
										ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Issue this IOCTL only after issuing bgscan_params ioctl\n")));
										return -EINVAL;
									}
								}
							}
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Issue IOCTL after vap creation in %s Line %d\n"), __func__, __LINE__));
							return -EINVAL;

						break;
					case BGSCAN_SSID:
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("<<< BGSCAN SSID >>>\n")));
						TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
						{ 
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" mac addr=%02x:%02x:%02x:%02x:%02x:%02x\n"), macaddr[0], 
							macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]));
							if(vap->iv_opmode == IEEE80211_M_STA)
							{
								if(copy_from_user(&vap->hal_priv_vap->bg_ssid, wrq->u.data.pointer, sizeof(vap->hal_priv_vap->bg_ssid)))
								{
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
									return -EINVAL;
								}

								ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("SSID len is %d ssid is %s\n"), vap->hal_priv_vap->bg_ssid.ssid_len, vap->hal_priv_vap->bg_ssid.ssid));
							}
						}

						break;
#ifdef PWR_SAVE_SUPPORT
						case PS_REQUEST:
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_PWR_SAVE, (TEXT("Name for vap creation is rtnl %d\n"),  rtnl_is_locked()));
							ONEBOX_DEBUG(ONEBOX_ZONE_PWR_SAVE, (TEXT("In %s Line %d issued PS_REQ ioctl \n"),__func__,__LINE__));
							TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
							{ 
								if(vap->iv_opmode == IEEE80211_M_STA){
									break;
								}
							}
							if(vap)
							{
								if(copy_from_user(&vap->hal_priv_vap->ps_params_ioctl, wrq->u.data.pointer, sizeof(vap->hal_priv_vap->ps_params)))
								{
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
									return -EINVAL;
								}

								ONEBOX_DEBUG(ONEBOX_ZONE_PWR_SAVE, (TEXT("monitor interval %d\n"), vap->hal_priv_vap->ps_params_ioctl.monitor_interval));
								driver_ps.update_ta = 1;
								update_pwr_save_status(vap, PS_ENABLE, IOCTL_PATH);
							}
							else
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERROR: Give IOCTL after station vap Creation\n")));
								return -EINVAL;
							}
							break;
						}
						case UAPSD_REQ:
						{
							TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
							{ 
								if(vap->iv_opmode == IEEE80211_M_STA){
									break;
								}
							}
							if(vap && (vap->iv_state == IEEE80211_S_RUN))
							{
								if(copy_from_user(&vap->hal_priv_vap->uapsd_params_ioctl, wrq->u.data.pointer, sizeof(vap->hal_priv_vap->uapsd_params_ioctl)))
								{
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
									return -EINVAL;
								}

							}
							else if(vap)
							{
								if(copy_from_user(&vap->hal_priv_vap->uapsd_params_ioctl, wrq->u.data.pointer, sizeof(vap->hal_priv_vap->uapsd_params_ioctl)))
								{
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
									return -EINVAL;
								}

								ONEBOX_DEBUG(ONEBOX_ZONE_PWR_SAVE, (TEXT("acs %02x wakeup %02x \n"), vap->hal_priv_vap->uapsd_params_ioctl.uapsd_acs, vap->hal_priv_vap->uapsd_params_ioctl.uapsd_wakeup_period));
								memcpy(&vap->hal_priv_vap->uapsd_params_updated, &vap->hal_priv_vap->uapsd_params_ioctl, sizeof(vap->hal_priv_vap->uapsd_params_ioctl));
							}
							else
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Give IOCTL after vap Creation\n")));
								return -EINVAL;
							}
							break;
						}
#endif
						case RESET_ADAPTER:
						{
							if (w_adapter->sc_nvaps > 1) 
								return -1;
							TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
							{ 
								if(vap->iv_opmode == IEEE80211_M_STA){
									break;
								}
							}
							if(vap)
							{
#if 0
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Copying ioctl values to updated \n")));
								if((&vap->hal_priv_vap->uapsd_params_ioctl) && (&vap->hal_priv_vap->uapsd_params_updated))
								{
									memcpy(&vap->hal_priv_vap->uapsd_params_updated, &vap->hal_priv_vap->uapsd_params_ioctl, sizeof(vap->hal_priv_vap->uapsd_params_ioctl));
								}
#endif
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Resetting the Adapter settings \n")));
								ni = vap->iv_bss; 
								if(ni && (vap->iv_state == IEEE80211_S_RUN))
								{
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Issuing sta leave cmd\n")));
									w_adapter->net80211_ops->onebox_ieee80211_sta_leave(ni);
								}
							}
							break;
						}
						case RX_FILTER:
						{
							if(copy_from_user(&w_adapter->rx_filter_word, wrq->u.data.pointer, sizeof(w_adapter->rx_filter_word)))
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
								return -EINVAL;
							}

							rx_filter_word = w_adapter->rx_filter_word;
							
									TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
									{
										if ( vap->iv_opmode == IEEE80211_M_STA ) {
											if ((vap->iv_state == IEEE80211_S_RUN) && (vap->iv_flags_ext & IEEE80211_FEXT_SWBMISS) ) {	
												rx_filter_word &= ~DISALLOW_BEACONS;
											}
										}
									}
									ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Setting RX_FILTER %04x\n"), w_adapter->rx_filter_word));
									status = onebox_send_rx_filter_frame(w_adapter, rx_filter_word);
									if(status < 0)
									{
													ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Sending of RX filter frame failed\n")));
									}
									break;
						}
						case RF_PWR_MODE:
						{
							if(copy_from_user(&w_adapter->rf_pwr_mode, wrq->u.data.pointer, sizeof(w_adapter->rf_pwr_mode)))
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
								return -EINVAL;
							}

							ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Setting RF PWR MODE %04x\n"), w_adapter->rf_pwr_mode));
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Setting RF PWR MODE %d\n"), w_adapter->rf_pwr_mode));
							break;
						}
						case RESET_PER_Q_STATS:
						{
							int q_num;
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("Resetting WMM stats\n"));
							if (copy_from_user(&q_num, ifr->ifr_data, sizeof(q_num)))
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Copy from user failed\n")));
								return -EFAULT;
							}
							if(q_num < MAX_HW_QUEUES) {
								w_adapter->total_tx_data_dropped[q_num] = 0;
								w_adapter->total_tx_data_sent[q_num] = 0;
							}else if(q_num == 15) {
								memset(w_adapter->total_tx_data_dropped, 0, sizeof(w_adapter->total_tx_data_dropped));
								memset(w_adapter->total_tx_data_sent, 0, sizeof(w_adapter->total_tx_data_sent));
							
							} else {
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("INVALID Q_NUM\n")));
								return -EFAULT;
					
							}
							/* : Reset all the queue stats for now. Individual queue stats for AP/STA needs to be
							 * modified.
							 */
							/* Reset Station queue stats */
#if 0
							w_adapter->total_sta_data_vo_pkt_send = 0;
							w_adapter->total_sta_vo_pkt_freed = 0;
							w_adapter->total_sta_data_vi_pkt_send = 0;
							w_adapter->total_sta_vi_pkt_freed = 0;
							w_adapter->total_sta_data_be_pkt_send = 0;
							w_adapter->total_sta_be_pkt_freed = 0;
							w_adapter->total_sta_data_bk_pkt_send = 0;
							w_adapter->total_sta_bk_pkt_freed = 0;

							/* Reset AP queue stats */
							w_adapter->total_ap_data_vo_pkt_send = 0;
							w_adapter->total_ap_vo_pkt_freed = 0;
							w_adapter->total_ap_data_vi_pkt_send = 0;
							w_adapter->total_ap_vi_pkt_freed = 0;
							w_adapter->total_ap_data_be_pkt_send = 0;
							w_adapter->total_ap_be_pkt_freed = 0;
							w_adapter->total_ap_data_bk_pkt_send = 0;
							w_adapter->total_ap_bk_pkt_freed = 0;

							w_adapter->tx_vo_dropped = 0;
							w_adapter->tx_vi_dropped = 0;
							w_adapter->tx_be_dropped = 0;
							w_adapter->tx_bk_dropped = 0;
#endif


							w_adapter->buf_semi_full_counter = 0;
							w_adapter->buf_full_counter = 0;
							w_adapter->no_buffer_fulls = 0;

#if 0
							switch(qnum)
							{
								case VO_Q:
									w_adapter->total_data_vo_pkt_send = 0;
									w_adapter->total_vo_pkt_freed = 0;
									break;
								case VI_Q:
									w_adapter->total_data_vi_pkt_send = 0;
									w_adapter->total_vi_pkt_freed = 0;
									break;
								case BE_Q:
									w_adapter->total_data_be_pkt_send = 0;
									w_adapter->total_be_pkt_freed = 0;
									break;
								case BK_Q:
									w_adapter->total_data_bk_pkt_send = 0;
									w_adapter->total_bk_pkt_freed = 0;
									break;
								default:
									w_adapter->total_data_vo_pkt_send = 0;
									w_adapter->total_vo_pkt_freed = 0;
									w_adapter->total_data_vi_pkt_send = 0;
									w_adapter->total_vi_pkt_freed = 0;
									w_adapter->total_data_be_pkt_send = 0;
									w_adapter->total_be_pkt_freed = 0;
									w_adapter->total_data_bk_pkt_send = 0;
									w_adapter->total_bk_pkt_freed = 0;
									break;
							}
#endif
						}
					break;
					case AGGR_LIMIT:
						if(copy_from_user(&w_adapter->aggr_limit, wrq->u.data.pointer, wrq->u.data.length))
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
							return -EINVAL;
						}
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s: Aggr params set are tx=%d rx=%d\n"), __func__, w_adapter->aggr_limit.tx_limit, w_adapter->aggr_limit.rx_limit));
						TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
						{ 
							if(vap->iv_opmode == IEEE80211_M_STA){
								break;
							}
						}
						if(vap)
						{
							vap->hal_priv_vap->aggr_rx_limit = w_adapter->aggr_limit.rx_limit;
						}
	 				break;
					case TEST_MODE:
					{
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("starting in test mode")));
						if(copy_from_user(&test, wrq->u.data.pointer, sizeof(test)))
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
							return -EINVAL;
						}

						if(w_adapter->devdep_ops->onebox_send_debug_frame(w_adapter, &test) != ONEBOX_STATUS_SUCCESS)
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT(" Sending Debug frame Failed\n")));
						}
					}
	 				break;
					case SET_COUNTRY:

					if(!w_adapter->sc_nvaps && (d_assets->protocol_enabled & WLAN_PROTOCOL)) {
						status = set_region (ic, value);
						if (status < 0) {
							return -EINVAL;
						}
						w_adapter->net80211_ops->onebox_media_init(ic);
					}	else {
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Vaps Are installed so please ISSUE IOCTL before creating VAPS\n")));
						return -EINVAL;

					}
					break;
					case GET_INFO:
						if(copy_from_user(&getinfo, wrq->u.data.pointer, sizeof(struct get_info)))
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
							return -EINVAL;
						}
						getinfo.data = kmalloc(2, GFP_KERNEL);
						if( !getinfo.data ) {
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:NOMEM\n"), __func__, __LINE__));
							return -ENOMEM;
						}
						if (!strcmp(getinfo.param_name, "country")) {
							memcpy(getinfo.data, ic->ic_regdomain.isocc, 2);
						} else if(!strcmp(getinfo.param_name, "country_code")) {
							*(uint16_t *)(getinfo.data) = ic->ic_regdomain.country;
						} else {
								return -EINVAL;
						}
						get_ptr = wrq->u.data.pointer;
						if(copy_to_user(get_ptr->data, getinfo.data, 2))
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
							return -EINVAL;
						}
						kfree(getinfo.data);
					break;
					case SET_SCAN_TYPE:
					if(!w_adapter->sc_nvaps && (d_assets->protocol_enabled & WLAN_PROTOCOL) && (w_adapter->fsm_state == FSM_MAC_INIT_DONE)) {
							ic->band_to_scan = value;
							w_adapter->net80211_ops->onebox_media_init(ic);
					}
					else {
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Vaps Are installed so please ISSUE IOCTL before creating VAPS\n")));
					}
					break;
					case SET_WOWLAN_CONFIG:
					TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
					{ 
						if(vap->iv_opmode == IEEE80211_M_STA)
						{
							wowlan_params = &vap->hal_priv_vap->wowlan_params;
							if(copy_from_user(wowlan_params, wrq->u.data.pointer, sizeof(struct wowlan_config)))
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
								return -EINVAL;
							}

							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT(" mac addr=%02x:%02x:%02x:%02x:%02x:%02x\n"), wowlan_params->macaddr[0], 
										wowlan_params->macaddr[1], wowlan_params->macaddr[2], wowlan_params->macaddr[3], wowlan_params->macaddr[4], wowlan_params->macaddr[5]));
							send_wowlan_params(vap, w_adapter);
							found = 1;
						}
					}

					if(!found)
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Vaps Are not installed so please ISSUE IOCTL after creating Station VAP\n")));
					break;

					case SET_EXT_ANT_GAIN:
					{
						if(copy_from_user(w_adapter->ant_gain, wrq->u.data.pointer, 2))
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
							return -EINVAL;
						}
						break;
					}
					case DRV_PARAMS:
					w_adapter->os_intf_ops->onebox_memset(&driver_params, 0, sizeof(driver_params_t));
					w_adapter->os_intf_ops->onebox_memcpy(driver_params.mac_addr, w_adapter->mac_addr, IEEE80211_ADDR_LEN );
					w_adapter->os_intf_ops->onebox_memcpy(driver_params.fw_ver, w_adapter->lmac_ver.ver.info.fw_ver, 8);
					driver_params.module_type = w_adapter->band_supported;
					ret_val = copy_to_user(wrq->u.data.pointer, &driver_params, sizeof(driver_params_t));
					return ret_val;

					case ADD_APPLE_IE:
					{
						TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
							if (vap->iv_opmode == IEEE80211_M_HOSTAP)
								found = 1;
						}
						if (!found) {
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
									(TEXT("No AP VAP found, cannot set \
									      APPLE IE:%s\n"),ifr->ifr_name));
							return ONEBOX_STATUS_FAILURE;
						}
						ic->ic_apple_ie.add_apple_ie = true;
						ic->ic_apple_ie.ie_len = wrq->u.data.length;
						ic->ic_apple_ie.ie_data = kmalloc(wrq->u.data.length, GFP_ATOMIC);
						if (ic->ic_apple_ie.ie_data == NULL) {
							ic->ic_apple_ie.add_apple_ie = false;
							return -EINVAL;
						}
						if(copy_from_user(ic->ic_apple_ie.ie_data, wrq->u.data.pointer, ic->ic_apple_ie.ie_len))
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
							return -EINVAL;
						}

					}
					break; 
					case IAP_INIT:
					{
						status = onebox_iap_config(w_adapter, RSI_IAP_INIT, wrq->u.data.pointer);
						return status;
					}
					break;
					case IAP_MFI_CHALLENGE:
					{
						status = onebox_iap_config(w_adapter, RSI_MFI_WRITE_CHALLENGE, wrq->u.data.pointer);
						if (status != ONEBOX_STATUS_SUCCESS) {
							return -EINVAL;
						}

						if (status != ONEBOX_STATUS_SUCCESS) {
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Write challenge failed!!!!")));
							return -EINVAL;
						}
						if(w_adapter->os_intf_ops->onebox_wait_event(&(d_assets->iap_event), EVENT_WAIT_FOREVER) < 0 ){
							ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed\n", __func__, __LINE__));
							return ONEBOX_STATUS_FAILURE;
						}
						w_adapter->os_intf_ops->onebox_reset_event(&d_assets->iap_event);
						//memset(wrq->u.data.pointer, 0, 128);
						if (copy_to_user(wrq->u.data.pointer, d_assets->mfi_signature, 128))
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERROR copying buffer to user\n")));
						return status;
					}
					break;
					case IAP_READ_CERTIFICATE:
					{
						status = onebox_iap_config(w_adapter, RSI_MFI_READ_CERTIFICATE, wrq->u.data.pointer);
						if( w_adapter->os_intf_ops->onebox_wait_event(&(d_assets->iap_event), EVENT_WAIT_FOREVER) < 0 ) {
							ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed\n", __func__, __LINE__));
							return ONEBOX_STATUS_FAILURE;
						}
						w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)d_assets->mfi_certificate, 908);
						if (copy_to_user(wrq->u.data.pointer, d_assets->mfi_certificate, 908))
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ERROR copying buffer to user\n")));

						w_adapter->os_intf_ops->onebox_reset_event(&d_assets->iap_event);
						return status;
					}
					break;
					case ENABLE_MAX_POWER:
					if(copy_from_user(&value, wrq->u.data.pointer, 1))
					{
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
						return -EINVAL;
					}
					w_adapter->max_pwr_enable = value;
					break;
					case CONF_BEACON_RECV:
					{
						TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
						{
							if( vap->iv_opmode == IEEE80211_M_STA )
							{
								status = ONEBOX_STATUS_FAILURE;
								return status;
							}
						}
						if(copy_from_user(&value, wrq->u.data.pointer, 1))
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
							return -EINVAL;
						}
						status = w_adapter->devdep_ops->conf_beacon_recv(w_adapter,(uint8)value);
						return status;
					}
					break;
					case SPECTRAL_MASK:
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("ONEBOX_IOCTL: SPECTRAL_MASK  \n"));
							w_adapter->spec_mask_set = 0;
							
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO, ("[%s][%d] : w_adapter->spec_mask_set is %d\n", __func__, __LINE__, w_adapter->spec_mask_set));
							if(copy_from_user(&w_adapter->spec_mask_set, wrq->u.data.pointer, 
										(sizeof(w_adapter->spec_mask_set))))
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
								return -EINVAL;
							}

							ONEBOX_DEBUG(ONEBOX_ZONE_INFO, ("[%s][%d] : w_adapter->spec_mask_set is %d\n", __func__, __LINE__, w_adapter->spec_mask_set));
						}
						break;
					case GET_TXPOWER:
					{
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("ONEBOX_IOCTL: GET TX_POWER  \n"));
						status = w_adapter->devdep_ops->onebox_get_txpower(w_adapter);
						if (status == ONEBOX_STATUS_SUCCESS)
						{
							status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->bb_rf_event), 10000); 
							if (status < 0) {
								ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
								return status;
							}
							w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->bb_rf_event));
							return ret_val;
						}
						else
						{	
							ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d :Error returning tx power \n", __func__, __LINE__));
							return ONEBOX_STATUS_FAILURE;
						}
					}
					break;
					case RSI_USEONLY_RATES:
 						ur = &ic->ic_ur;
						rc = ieee80211_useonly_rates(ur, wrq);
						if (rc <= 0) {
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
							    (TEXT("%s: Invalid supported rates\n"), 
							    __func__));
							return -EINVAL;
						}
						ur->ur_enable = 1;
					break;
#ifdef RADAR_AUTO
					case RADAR_PKT:
					w_adapter->os_intf_ops->onebox_acquire_spinlock(&w_adapter->radar_lock,&radar_intr_state);
					if (w_adapter->radar_front == NULL)
					{	
						w_adapter->os_intf_ops->onebox_release_spinlock(&w_adapter->radar_lock,radar_intr_state);
						return -1;
					} 
					else 
					{
						radar_pkt_to_app = w_adapter->radar_front; 
						ret_val = copy_to_user(wrq->u.data.pointer, radar_pkt_to_app->ptr, 258);
						w_adapter->radar_front = w_adapter->radar_front->next;
						if (w_adapter->radar_front == NULL)
						{
							w_adapter->radar_rear = NULL;
						}
						temp = radar_pkt_to_app;
						kfree(temp->ptr);
						kfree(temp);
					}
					w_adapter->os_intf_ops->onebox_release_spinlock(&w_adapter->radar_lock,radar_intr_state);
					break;
#endif
#ifdef ONEBOX_CONFIG_PUF
          case PUF_REQUEST:
            if(send_puf_request(w_adapter, wrq) != ONEBOX_STATUS_SUCCESS)
            {
              ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT(" puf operation failed\n")));
              return ONEBOX_STATUS_FAILURE;
            }
            return ONEBOX_STATUS_SUCCESS;
#endif 
					default:
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("No match yet\n"));
					return -EINVAL;
					break;
				}
			}
			else if(w_adapter->Driver_Mode == SNIFFER_MODE)
			{
				switch((unsigned char)wrq->u.data.flags)
				{
					case PER_RECEIVE:
						if(copy_from_user(&value, wrq->u.data.pointer, 1))
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
							return -EINVAL;
						}
						w_adapter->endpoint_params.per_ch_bw = (*(uint8 *)&value & 0x07);
						w_adapter->endpoint_params.enable_11j =(*(uint8 *)&value >> 3 )&0x1;
						w_adapter->recv_channel = (uint8)(wrq->u.data.flags >> 8);
						w_adapter->endpoint_params.channel = w_adapter->recv_channel;
						if(((w_adapter->recv_channel >= 36) && (w_adapter->recv_channel <= 165)) && (!(w_adapter->band_supported)))
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("5Ghz Band Not supported\n"));
							w_adapter->ch_util_start_flag = 1;
							return -1;
						}
						w_adapter->ch_util_start_flag = 0;
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("In %s and %d ch_width %d recv_channel %d 11J enable is %d\n", __func__, __LINE__, w_adapter->endpoint_params.per_ch_bw, w_adapter->recv_channel, w_adapter->endpoint_params.enable_11j));
						w_adapter->devdep_ops->onebox_band_check(w_adapter);
							w_adapter->devdep_ops->onebox_set_channel(w_adapter,w_adapter->recv_channel);
							return ret_val;
							break;
#ifdef CHANNEL_UTILIZATION
					case CH_UTIL_START:
							if (w_adapter->ch_util_start_flag == 0) {
								if(copy_from_user(&w_adapter->ch_util_start_flag, wrq->u.data.pointer, 2))
								{
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
									return -EINVAL;
								}
								if(copy_from_user(&w_adapter->stats_interval, wrq->u.data.pointer + 2, 2))
								{
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
									return -EINVAL;
								}
								if(copy_from_user(&w_adapter->false_cca_rssi_threshold, wrq->u.data.pointer + 4, 1))
								{
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
									return -EINVAL;
								}

								w_adapter->recv_stop = 0;
								w_adapter->rx_running = 1;
								/*send frame to start receiving stats*/
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("********* %d %d %d\n"), w_adapter->ch_util_start_flag, w_adapter->stats_interval, w_adapter->false_cca_rssi_threshold));
								if((w_adapter->core_ops->onebox_stats_frame(w_adapter)))
								{
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("Failed: to send start stats frame\n"));
									return ONEBOX_STATUS_FAILURE;
								}
								ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("**** Channel Utilization test Started ****\n"));
							}
							else if (w_adapter->ch_util_start_flag == 1)
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("Failed: Card Does not support Band\n"));
								return ONEBOX_STATUS_FAILURE;
							}
							else
							{
								/*wait for event to receive stats from firmware*/
								//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("waiting for event \n")));
								status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->stats_event), EVENT_WAIT_FOREVER);
								if (status < 0) {
									ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
									return status;
								}
								w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->stats_event));
								if(w_adapter->recv_stop) {
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("Recvd stop Command\n"));
									break;
								}
								//ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("********* %d %d %d %d %d\n"), w_adapter->sta_info.utilization, w_adapter->sta_info.rssi_utilization, w_adapter->sta_info.tot_bytes, w_adapter->sta_info.rssi_bytes, w_adapter->sta_info.interval_duration));
								ret_val = copy_to_user(wrq->u.data.pointer, &w_adapter->sta_info, sizeof(per_stats));
								return ret_val;
							}
							break;
					case CH_UTIL_STOP:
							w_adapter->ch_util_start_flag = 0; // reset to start channel utilization again
							w_adapter->recv_stop = 1;
							w_adapter->rx_running = 0;
							/* send frame to stop receiving stats*/
							if((w_adapter->core_ops->onebox_stats_frame(w_adapter)))
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("Failed: to send stop stats frame\n"));
							}
							w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->stats_event));
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("**** Channel Utilization test Stopped ****\n"));
							break;
					case ANT_SEL:
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("antenna sel %d in sniffer mode \n"),value));
						value = ((unsigned short)wrq->u.data.flags >> 8); //endpoint value 
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("ANT_SEL value is : %d \n"),value));
						w_adapter->devdep_ops->onebox_program_ant_sel(w_adapter, value, CONFIG_ANT_SEL);
						break;
#endif
					default:
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Failed: in %d case\n"),(unsigned char)wrq->u.data.flags));
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("No match yet\n"));
							return -EINVAL;
							break;
				}
			}
			else if(w_adapter->Driver_Mode == RF_EVAL_MODE_ON )
			{
				switch((unsigned char)wrq->u.data.flags)
				{
          case MASTER_READ:
            {
              ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("performing master read\n")));
              if(w_adapter->devdep_ops->onebox_do_master_ops(w_adapter, wrq->u.data.pointer, ONEBOX_MASTER_READ) != ONEBOX_STATUS_SUCCESS)
              {
                ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT(" Data Read Failed\n")));
              }
            }
            break;
          case MASTER_WRITE:
            {
              ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("performing master write \n")));
              if(w_adapter->devdep_ops->onebox_do_master_ops(w_adapter, wrq->u.data.pointer, ONEBOX_MASTER_WRITE) != ONEBOX_STATUS_SUCCESS)
              {
                ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT(" Data Write Failed\n")));
              }
            }
            break;

					case PER_TRANSMIT:
						if((d_assets->protocol_enabled & WLAN_PROTOCOL) && (w_adapter->fsm_state == FSM_MAC_INIT_DONE))
						{

							if(copy_from_user(&w_adapter->endpoint_params, wrq->u.data.pointer, wrq->u.data.length))
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
								return -EINVAL;
							}

							if (w_adapter->endpoint_params.enable)
							{
								if ( w_adapter->endpoint_params.ctry_region == 127 ){
									/* Code for World Domain is 3 */
									country_code = 3;
								} else {
									country_code = w_adapter->endpoint_params.ctry_region;
								}
								if ( ic->ic_regdomain.pad[0] != country_code ) {
									ic->ic_regdomain.pad[0] = country_code;
									w_adapter->net80211_ops->onebox_media_init(ic);
								}
								if(!(w_adapter->band_supported) && 
										((w_adapter->endpoint_params.channel >= 36) || (w_adapter->endpoint_params.enable_11j))) {
									ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Invalid Channel Number for Module type %02X\n",ic->module_model_type));
									return -1;
								}
							}
							if(w_adapter->core_ops->onebox_start_per_tx(w_adapter))
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid arguments issued by user\n")));
								return -EINVAL;
							}
						}
						else
						{
							ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Issue enable protocol command before issuing transmit command\n"));
							ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Usage: ./onebox_util rpine0 enable_protocol 1\n"));
							return -EINVAL;
						}
						return 0;
						break;
					case PER_RECEIVE_STOP:
						w_adapter->recv_stop = 1;
						w_adapter->rx_running = 0;
						w_adapter->sta_info.stop_per = 1;
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("PER_RECEIVE_STOP\n"));
					case PER_RECEIVE:
						if((d_assets->protocol_enabled & WLAN_PROTOCOL) && (w_adapter->fsm_state == FSM_MAC_INIT_DONE))
						{
										ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("per_ch_bw :%d \n",w_adapter->endpoint_params.per_ch_bw));
										if(copy_from_user(&value, wrq->u.data.pointer, 1))
										{
											ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
											return -EINVAL;
										}
										w_adapter->endpoint_params.per_ch_bw = (*(uint8 *)&value & 0x07);
										w_adapter->endpoint_params.enable_11j =(*(uint8 *)&value >> 3 )&0x1;
										w_adapter->recv_channel = (uint8)(wrq->u.data.flags >> 8);
										w_adapter->endpoint_params.channel = w_adapter->recv_channel;
										ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("In %s and %d ch_width %d recv_channel %d 11J enable is %d\n", __func__, __LINE__, w_adapter->endpoint_params.per_ch_bw, w_adapter->recv_channel, w_adapter->endpoint_params.enable_11j));
										if (w_adapter->endpoint_params.channel == 0xFF)
										{
														if(w_adapter->devdep_ops->onebox_mgmt_send_bb_reset_req(w_adapter) != ONEBOX_STATUS_SUCCESS) {
																		return ONEBOX_STATUS_FAILURE;
														}
										}
										else if (w_adapter->endpoint_params.channel)
										{
                    if (w_adapter->device_model == RSI_DEV_9116) {
                      if(w_adapter->d_assets->disable_programming){
                        goto  SEND_STATS_FRAME;
                      } else {
                        w_adapter->devdep_ops->onebox_band_check(w_adapter);
                        set_per_configurations (w_adapter); 
                      }
                    } else {
														w_adapter->devdep_ops->onebox_band_check(w_adapter);
                    }
														w_adapter->fsm_state = FSM_SCAN_CFM;		
														w_adapter->devdep_ops->onebox_set_channel(w_adapter,w_adapter->recv_channel);
														status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->bb_rf_event), EVENT_WAIT_FOREVER);
														if (status < 0) {
																		ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
																		return status;
														}
														w_adapter->fsm_state = FSM_MAC_INIT_DONE;
														w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->bb_rf_event));
										}
SEND_STATS_FRAME:
										if (!w_adapter->rx_running)
										{
														if(!(w_adapter->core_ops->onebox_stats_frame(w_adapter)))
														{
																		w_adapter->rx_running = 1;
																		if (w_adapter->recv_stop)
																		{
																						w_adapter->recv_stop = 0;
																						w_adapter->rx_running = 0;
																						return ONEBOX_STATUS_SUCCESS;
																		}  
														}
										}

										status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->stats_event), EVENT_WAIT_FOREVER);
										if (status < 0) {
														ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
														return ONEBOX_STATUS_FAILURE;
										}
										w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->stats_event));
										w_adapter->sta_info.stop_per = 0;
										ret_val = copy_to_user(wrq->u.data.pointer, &w_adapter->sta_info, sizeof(per_stats));
										return ret_val;
						}
						else
						{
										ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Issue enable protocol command before issuing receive command\n"));
										ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("Usage: ./onebox_util rpine0 enable_protocol 1\n"));
										return -EINVAL;

						}
						break;
					case PER_PACKET:
						if(copy_from_user(&w_adapter->per_packet, wrq->u.data.pointer, wrq->u.data.length))
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Copying PER Packet Failed in %s\n"), __func__));
							return -EINVAL;
						}
						ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG, (TEXT("Copying PER Packet in %s\n"), __func__));
						w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_DEBUG, (PUCHAR)&w_adapter->per_packet.packet, w_adapter->per_packet.length);
						return 0;
						break;
					case ENABLE_MAX_POWER:
					if(copy_from_user(&value, wrq->u.data.pointer, 1))
					{
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
						return -EINVAL;
					}
					w_adapter->max_pwr_enable = value;
					break;
					case ANT_SEL:
						value = ((unsigned short)wrq->u.data.flags >> 8); //endpoint value 
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("ANT_SEL value is : %d \n"),value));
						w_adapter->devdep_ops->onebox_program_ant_sel(w_adapter, value, CONFIG_ANT_SEL);
						break;
					case ANT_TYPE:
						if(copy_from_user(&value, wrq->u.data.pointer, 1))
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
							return -EINVAL;
						}
						w_adapter->ant_path = ((unsigned short)wrq->u.data.flags >> 8); //endpoint value 
						w_adapter->ant_type = (unsigned char )value;
						if( d_assets->onboard_antenna && (w_adapter->ant_path == PATH_ONBOARD) && (w_adapter->ant_type != 1))
						{
								printk("Invalid antenna path configuration, as Onboard antenna already present\n" );
								return ONEBOX_STATUS_FAILURE;
						}
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("ANT_PATH is %d type is : %d \n"),w_adapter->ant_path, w_adapter->ant_type));
						w_adapter->devdep_ops->onebox_program_ant_sel(w_adapter, 0, CONFIG_ANT_TYPE);
					case SET_EXT_ANT_GAIN:
						//return copy_from_user(w_adapter->ant_gain, wrq->u.data.pointer, wrq->u.data.length);
						if(copy_from_user(w_adapter->ant_gain, wrq->u.data.pointer, 2))
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
							return -EINVAL;
						}
						break;
					case SET_ENDPOINT:
						value = ((unsigned short)wrq->u.data.flags >> 8); //endpoint value 
						ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("ENDPOINT type is : %d \n"),value));
						w_adapter->endpoint = value;
#ifdef PROGRAMMING_BBP_TA	
						w_adapter->devdep_ops->onebox_program_bb_rf(w_adapter);
#else
						if (value == 0)
						{
							w_adapter->endpoint_params.per_ch_bw = 0;
							w_adapter->endpoint_params.channel = 1;
						}
						else if (value == 1)
						{
							w_adapter->endpoint_params.per_ch_bw = 6;
							w_adapter->endpoint_params.channel = 1;
						}
						else if (value == 2)
						{
							w_adapter->endpoint_params.per_ch_bw = 0;
							w_adapter->endpoint_params.channel = 36;
						}
						else if (value == 3)
						{
							w_adapter->endpoint_params.per_ch_bw = 6;
							w_adapter->endpoint_params.channel = 36;
						}
						w_adapter->devdep_ops->onebox_band_check(w_adapter);
#endif
						break;
					
					case SPECTRAL_MASK:
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("ONEBOX_IOCTL: SPECTRAL_MASK  \n"));
							w_adapter->spec_mask_set = 0;
							
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO, ("[%s][%d] : w_adapter->spec_mask_set is %d\n", __func__, __LINE__, w_adapter->spec_mask_set));
							if(copy_from_user(&w_adapter->spec_mask_set, wrq->u.data.pointer, 
										(sizeof(w_adapter->spec_mask_set))))
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
								return -EINVAL;
							}

							ONEBOX_DEBUG(ONEBOX_ZONE_INFO, ("[%s][%d] : w_adapter->spec_mask_set is %d\n", __func__, __LINE__, w_adapter->spec_mask_set));
						}
						break;
					case EEPROM_READ_IOCTL:
						{
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO,("ONEBOX_IOCTL: EEPROM READ  \n"));
							w_adapter->os_intf_ops->onebox_memset(&w_adapter->eeprom, 0, sizeof(EEPROMRW));
							if(copy_from_user(&w_adapter->eeprom, wrq->u.data.pointer, 
										(sizeof(EEPROMRW) - sizeof(w_adapter->eeprom.data))))
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
								return -EINVAL;
							}

							status = w_adapter->devdep_ops->onebox_eeprom_rd(w_adapter);
							if (status == ONEBOX_STATUS_SUCCESS)
							{
								status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->bb_rf_event), 10000); 
								if (status < 0) {
									ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
									return status;
								}
								ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" eeprom length: %d, \n"), w_adapter->eeprom.length));
								ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT(" eeprom offset: %d, \n"), w_adapter->eeprom.offset));
								ret_val = copy_to_user(wrq->u.data.pointer, &w_adapter->eeprom, sizeof(EEPROMRW));
								w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->bb_rf_event));
								return ret_val;
							}
							else
							{	
								return ONEBOX_STATUS_FAILURE;
							}
						}
						break;
#if 0
        case EEPROM_WRITE_IOCTL:
          {
            ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("ONEBOX_IOCTL: EEPROM WRITE  \n"));
#if 1
            if(copy_from_user(&w_adapter->eeprom, wrq->u.data.pointer, wrq->u.data.length))
            {
              ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Copying Failed\n")));
              return -EINVAL;
            }
#endif
            w_adapter->eeprom.length = (wrq->u.data.length - 10);
            ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
                (TEXT("===> Frame to WRITE IN TO EEPROM <===\n")));

            mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;

            w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);

            /* FrameType*/
            mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(EEPROM_WRITE);
            mgmt_frame->desc_word[0] = (ONEBOX_WIFI_MGMT_Q << 12 | w_adapter->eeprom.length);
            if (!w_adapter->eeprom_erase)
            {
              mgmt_frame->desc_word[2] = ONEBOX_CPU_TO_LE16(BIT(10));
              w_adapter->eeprom_erase = 1;
            }  
            /* Number of bytes to read*/
            mgmt_frame->desc_word[3] = ONEBOX_CPU_TO_LE16(w_adapter->eeprom.length);
            /* Address to read*/
            mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(w_adapter->eeprom.offset);
            mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(w_adapter->eeprom.offset >> 16);
          	w_adapter->os_intf_ops->onebox_memcpy(mgmt_frame->u.byte.buf, w_adapter->eeprom.data, w_adapter->eeprom.length);
  
            w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_INFO, (PUCHAR)mgmt_frame, FRAME_DESC_SZ + w_adapter->eeprom.length);
            status = w_adapter->osi_host_intf_ops->onebox_host_intf_write_pkt(w_adapter,
                (uint8 *)mgmt_frame,
                FRAME_DESC_SZ + w_adapter->eeprom.length);
            if (status == ONEBOX_STATUS_SUCCESS)
            {
              w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->bb_rf_event), 10000); 
              w_adapter->os_intf_ops->onebox_memset(w_adapter->eeprom.data, 0, 480);
              return ONEBOX_STATUS_SUCCESS;
            }
            else
            {	
              return ONEBOX_STATUS_FAILURE;
            }
          }
          break;
#endif
						case PS_REQUEST:
						{
							if((w_adapter->Driver_Mode == RF_EVAL_MODE_ON)) {

								send_sleep_req_in_per_mode(w_adapter, wrq->u.data.pointer);

							}
							break;
						}
						case RF_PWR_MODE:
						{
							if(copy_from_user(&w_adapter->rf_pwr_mode, wrq->u.data.pointer, sizeof(w_adapter->rf_pwr_mode)))
							{
								ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
								return -EINVAL;
							}

							w_adapter->rf_power_mode_change = 1;
							ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Setting RF PWR MODE %04x\n"), w_adapter->rf_pwr_mode));
							break;
						}
#ifdef RADAR_AUTO
						case RADAR_ENABLE:
						value = (*(uint16 *)wrq->u.data.pointer >> 8); //enable_radar
						w_adapter->regdomain = *(uint8 *)wrq->u.data.pointer;
						w_adapter->devdep_ops->onebox_radar_req_frame(w_adapter, value, 1 );
						break;
						case RADAR_PKT:
						w_adapter->os_intf_ops->onebox_acquire_spinlock(&w_adapter->radar_lock,&radar_intr_state);
						if (w_adapter->radar_front == NULL)
						{	
							w_adapter->os_intf_ops->onebox_release_spinlock(&w_adapter->radar_lock,radar_intr_state);
							return -1;
						} 
						else 
						{
							radar_pkt_to_app = w_adapter->radar_front; 
							ret_val = copy_to_user(wrq->u.data.pointer, radar_pkt_to_app->ptr, 258);
							w_adapter->radar_front = w_adapter->radar_front->next;
							if (w_adapter->radar_front == NULL)
							{
								w_adapter->radar_rear = NULL;
							}
							temp = radar_pkt_to_app;
							kfree(temp->ptr);
							kfree(temp);
						}
						w_adapter->os_intf_ops->onebox_release_spinlock(&w_adapter->radar_lock,radar_intr_state);
						break;
#endif
					default:
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("No match yet\n"));
					//	return -EINVAL;
					//	break;
				}
            } 
        }
		break;
		case ONEBOX_SET_BB_RF:
        {
            if((w_adapter->device_model == RSI_DEV_9116) && 
			    (((unsigned char)wrq->u.data.flags == PROTOCOL_RF_WRITE ) || 
			     ((unsigned char)wrq->u.data.flags == PROTOCOL_RF_READ ))) {
                w_adapter->os_intf_ops->onebox_memset(&d_assets->common_bb_rf_params, 0, sizeof(bb_rf_params_t));
                if(copy_from_user(&d_assets->common_bb_rf_params.Data[0], wrq->u.data.pointer, (sizeof(bb_rf_params_t))))
                {
                    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
                    return -EINVAL;
                }
                w_adapter->bb_rf_params.protocol_id = 0x00;
                w_adapter->os_intf_ops->onebox_memcpy(&(w_adapter->bb_rf_params), &(w_adapter->d_assets->common_bb_rf_params), sizeof(bb_rf_params_t));

                if(w_adapter->devdep_ops->onebox_set_bb_rf_values(w_adapter, wrq))
                {
                    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid arguments issued by user\n")));
                    return -EINVAL;
                }

            } else {	
                if(copy_from_user(&w_adapter->bb_rf_params.Data[0], wrq->u.data.pointer, (sizeof(w_adapter->bb_rf_params))))
                {
                    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("In %s Line %d:Copying Failed\n"), __func__, __LINE__));
                    return -EINVAL;
                }
                if (w_adapter->device_model == RSI_DEV_9116) {
                    w_adapter->bb_rf_params.protocol_id = 0xff;
                }
                if(w_adapter->devdep_ops->onebox_set_bb_rf_values(w_adapter, wrq))
                {
                    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid arguments issued by user\n")));
                    return -EINVAL;
                }
                return ONEBOX_STATUS_SUCCESS;
            }
            return ONEBOX_STATUS_SUCCESS;
        }
		break;
		case ONEBOX_SET_CW_MODE:
		{
			if(w_adapter->Driver_Mode == RF_EVAL_MODE_ON)
			{
				channel = (unsigned int )wrq->u.data.flags; //cw_type & subtype info
				w_adapter->cw_type	=	(channel & 0x0f00) >> 8 ;
				w_adapter->cw_sub_type	=	(channel & 0xf000) >> 12 ;
				channel = (uint8) wrq->u.data.flags;
				if (!channel)
					channel = 1;
				if(w_adapter->cw_type == 2)
				{
					goto disable_cwmode;
				}	
				else
				{	
					if(channel <= 16 && (w_adapter->cw_type == 1))
					{
						w_adapter->endpoint_params.enable_11j = 1;
						w_adapter->cw_type = 0;	
					}
					else if(channel <= 14 && (w_adapter->cw_type == 0))
					{
						w_adapter->endpoint_params.enable_11j = 0;

					}
					else if((channel >= 36) && (channel <= 165) && (w_adapter->cw_type == 0))
					{
						w_adapter->endpoint_params.enable_11j = 0;
					}
					else if(((channel >= 184) && (channel <= 196)) && (w_adapter->cw_type == 1))
					{
						w_adapter->endpoint_params.enable_11j = 1;
						w_adapter->cw_type = 0;	
					}
					else
					{
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid channel setting\n")));
						return -1;
					}
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ONEBOX_IOCTL:  channel is %d \n"),channel));
					w_adapter->endpoint_params.channel = channel;
					w_adapter->endpoint_params.per_ch_bw = BW_20Mhz; 
					w_adapter->devdep_ops->onebox_band_check(w_adapter);
					w_adapter->fsm_state = FSM_SCAN_CFM;		
					w_adapter->devdep_ops->onebox_set_channel(w_adapter,channel);
					status = w_adapter->os_intf_ops->onebox_wait_event(&(w_adapter->bb_rf_event), EVENT_WAIT_FOREVER);

					if (status < 0) {
						ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("%s line no %d : wait event failed %d\n", __func__, __LINE__, status));
						return status;
					}
					w_adapter->fsm_state = FSM_MAC_INIT_DONE;
					w_adapter->os_intf_ops->onebox_reset_event(&(w_adapter->bb_rf_event));
				}

				//	channel = (unsigned int)wrq->u.data.flags; //cw_type & subtype info
				//	w_adapter->cw_type	=	(channel & 0x0f00) >> 8 ;
				//	w_adapter->cw_sub_type	=	(channel & 0xf000) >> 12 ;
disable_cwmode:
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("ONEBOX_IOCTL:  SET_CW_MODE , cw_mode:%d cw_type:%d channel is %d\n"),w_adapter->cw_type,w_adapter->cw_sub_type, channel));
				if(w_adapter->devdep_ops->onebox_cw_mode(w_adapter, w_adapter->cw_sub_type))
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Invalid arguments issued by user\n")));
					return -EINVAL;
				}
				break;
				return ONEBOX_STATUS_SUCCESS;
			}
			else 
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("\n")));
				return -EINVAL;

			}	
		}
		break;
		default:
		{
			if (priv == 0) /* req is a standard ioctl */ 
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, 
					("%s: unable to handle ioctl 0x%X index %d\n",
					 __func__ ,cmd , index));
			} 
			else /* Ignore it, Bad ioctl */ 
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_INFO, 
					("%s: ignoring unrecognised ioctl: 0x%X\n", __func__, cmd));
			}
			return -EFAULT;
        }
    }
	return ret_val;
}



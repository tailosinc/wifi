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
#include "onebox_linux.h"
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/file.h>

/**
 * This function gives the driver and firmware version number.
 *
 * @param  Pointe to the page.
 * @param  start.  
 * @param  off.  
 * @param  Count.  
 * @param  EOF.  
 * @param  data.  
 * @return Returns the number of bytes read. 
 */
#if 0
int32 onebox_proc_version_read (int8  *page,int8  **start, off_t off,
                                int32 count,int32 *eof, PVOID data )
{
	int32        len;
	WLAN_ADAPTER w_adapter    = data;
	w_adapter->driver_ver.major = 0;
	w_adapter->driver_ver.minor = 0;

	w_adapter->driver_ver.release_num = 11;
	w_adapter->driver_ver.patch_num = 0;
	len  = sprintf(page,"UMAC : %x.%d.%d.%d\nLMAC : %d.%d.%d.%d.%d\n",
	               w_adapter->driver_ver.major,
	               w_adapter->driver_ver.minor, 
	               w_adapter->driver_ver.release_num, 
	               w_adapter->driver_ver.patch_num,
	               w_adapter->lmac_ver.major,
	               w_adapter->lmac_ver.minor, 
	               w_adapter->lmac_ver.release_num, 
	               w_adapter->lmac_ver.patch_num,
	               w_adapter->lmac_ver.ver.info.fw_ver[0]);
	*eof = 1;
	return len;
}



/**
 * * This function display the currently enabled debug zones.
 * *
 * * @param page   Page Is The Buffer Where Data Will
 * *               Be Written
 * * @param start  From Where Data Start(NULL)
 * * @param off    Offset Location
 * * @param count  Same As Offset For Reading
 * * @param eof    To Signal No More Data Present
 * * @param data   Pointer To Driver Priv Area
 * */
int onebox_proc_debug_zone_read(char *page, char **start, off_t off,
                                int count, int32 *eof, void *data)
{
	int32 len;
	len  = ONEBOX_SPRINTF(page,"The zones available are %#x\n",*(int32 *)data);
	*eof = 1;
	return len;
}
#endif

static int onebox_proc_version_show(struct seq_file *seq, void *data)
{
	WLAN_ADAPTER w_adapter = seq->private;

	w_adapter->driver_ver.major =1;
	w_adapter->driver_ver.minor =4;
	w_adapter->driver_ver.release_num = 2;
	w_adapter->driver_ver.patch_num = 0;

	seq_printf(seq, 
		   "UMAC : %x.%d.%d.%d\nLMAC : %d.%d.%d.%d.%d\n",
		   w_adapter->driver_ver.major,
		   w_adapter->driver_ver.minor, 
		   w_adapter->driver_ver.release_num, 
		   w_adapter->driver_ver.patch_num,
		   w_adapter->lmac_ver.major,
		   w_adapter->lmac_ver.minor, 
		   w_adapter->lmac_ver.release_num, 
		   w_adapter->lmac_ver.patch_num,
		   w_adapter->lmac_ver.ver.info.fw_ver[0]);
	return 0;
	
}

static int onebox_proc_stats_show(struct seq_file *seq, void *data)
{
	WLAN_ADAPTER w_adapter = seq->private;
	struct ieee80211com *ic = &w_adapter->vap_com;
	struct ieee80211vap *vap;
	char fsm_state_lp[][32] = { 
		"FSM_CARD_NOT_READY",
		"FSM_FW_LOADED",
		"FSM_LOAD_BOOTUP_PARAMS",
		"FSM_EEPROM_CHECK",
		"FSM_EEPROM_READ_RF_TYPE",
		"FSM_EEPROM_READ_MAC_ADDR",
		"FSM_EEPROM_READ_2P4_PWR_VALS",
		"FSM_EEPROM_READ_5P1_PWR_VALS",
		"FSM_RESET_MAC_CFM",
		"FSM_BB_RF_START",
		"FSM_WAKEUP_SLEEP_VALS",
		"FSM_OPEN",
		"FSM_DEEP_SLEEP_ENABLE",
		"FSM_MAC_INIT_DONE"
	};

	TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) 
	{ 
		if (vap->iv_opmode == IEEE80211_M_STA)
			break;
	}

	seq_printf(seq, "==> ONEBOX DRIVER STATUS <==\n");
	seq_printf(seq, "DRIVER_FSM_STATE: ");
#if 1
	if (w_adapter->fsm_state <= FSM_MAC_INIT_DONE)
		seq_printf(seq,
			   "%s",
			   fsm_state_lp[w_adapter->fsm_state]);
#endif
	seq_printf(seq,
		   "(%d)\n\n", 
	           w_adapter->fsm_state);

	if (w_adapter->Driver_Mode == RF_EVAL_MODE_ON)
		seq_printf(seq,
			   "total PER packets sent : %d\n",
			   w_adapter->total_per_pkt_sent);

	seq_printf(seq,
		   "total beacon send : %d\n",
		   w_adapter->total_beacon_count);
	seq_printf(seq,
		   "total beacon Interrupts Received : %d\n",
		   w_adapter->beacon_interrupt);
	seq_printf(seq,
		   "beacon debugcnt : %d\n",
		   w_adapter->beacon_dbg_count);
	/* Mgmt TX Path Stats */
	seq_printf(seq,
		   "total_mgmt_pkt_queued : %d\t",
		   get_skb_queue_len(&w_adapter->host_tx_queue[4]));
	seq_printf(seq,
		   "total_mgmt_pkt_send : %d\n",
		   w_adapter->total_tx_data_sent[MGMT_SOFT_Q]);
	/* RX Path Stats */
	seq_printf(seq,
		   "total_mgmt_rx       : %d\n",
		   w_adapter->total_mgmt_rx_done_intr);
	seq_printf(seq,
		   "total_data_rx       : %d\n",
		   w_adapter->total_data_rx_done_intr);
	seq_printf(seq,
		   "BUFFER FULL STATUS  : %d\n",
		   w_adapter->buffer_full);
	seq_printf(seq,
		   "SEMI BUFFER FULL STATUS  : %d\n",
		   w_adapter->semi_buffer_full);
	seq_printf(seq,
		   "MGMT BUFFER FULL STATUS  : %d\n",
		   w_adapter->mgmt_buffer_full);
	seq_printf(seq,
		   "BLOCK_QUEUE STATUS  : %d\n",
		   w_adapter->sta_data_block);
	seq_printf(seq,
		   "BLOCK_AP_QUEUE STATUS  : %d\n",
		   w_adapter->block_ap_queues);
	if (vap)
		seq_printf(seq,
			   "PWR_SAVE STATUS  : %d\n",
			   vap->hal_priv_vap->drv_params.ps_state);

	seq_printf(seq,
		   "BUFFER FULL COUNTER  : %d\n",
		   w_adapter->buf_full_counter);
	seq_printf(seq,
		   "BUFFER SEMI FULL COUNTER  : %d\n",
		   w_adapter->buf_semi_full_counter);
	seq_printf(seq,
		   "MGMT BUFFER FULL COUNTER  : %d\n",
		   w_adapter->mgmt_buf_full_counter);
	/* AP mode TX data Path Statistics */
	seq_printf(seq,
		   "total_ap_vo_pkt_queued:  %8d\t",
		   get_skb_queue_len(&w_adapter->host_tx_queue[VO_Q_AP]));
	seq_printf(seq,
		   "total_ap_vo_pkt_send: %8d\t",
		   w_adapter->total_tx_data_sent[VO_Q_AP]);
	seq_printf(seq,
		   "total_ap_vo_packets dropped:  %d\n",
		   (uint32)w_adapter->total_tx_data_dropped[VO_Q_AP]);

	seq_printf(seq,
		   "total_ap_vi_pkt_queued:  %8d\t",
		   get_skb_queue_len(&w_adapter->host_tx_queue[VI_Q_AP]));
	seq_printf(seq,
		   "total_ap_vi_pkt_send: %8d\t",
		   w_adapter->total_tx_data_sent[VI_Q_AP]);
	seq_printf(seq,
		   "total_ap_vi_packets dropped:  %d\n",
		   (uint32)w_adapter->total_tx_data_dropped[VI_Q_AP]);


	seq_printf(seq,
		   "total_ap_be_pkt_queued:  %8d\t",
		   get_skb_queue_len(&w_adapter->host_tx_queue[BE_Q_AP]));
	seq_printf(seq,
		   "total_ap_be_pkt_send: %8d\t",
		   w_adapter->total_tx_data_sent[BE_Q_AP]);
	seq_printf(seq,
		   "total_ap_be_packets dropped:  %d\n",
		   (uint32)w_adapter->total_tx_data_dropped[BE_Q_AP]);

	seq_printf(seq,
		   "total_ap_bk_pkt_queued:  %8d\t",
		   get_skb_queue_len(&w_adapter->host_tx_queue[BK_Q_AP]));
	seq_printf(seq,
		   "total_ap_bk_pkt_send: %8d\t",
		   w_adapter->total_tx_data_sent[BK_Q_AP]);
	seq_printf(seq,
		   "total_ap_bk_packets dropped:  %d\n",
		   (uint32)w_adapter->total_tx_data_dropped[BK_Q_AP]);

	/*  Station mode TX data packet Statistics */
	seq_printf(seq, "total_sta_vo_pkt_queued: %8d\t",
	                      get_skb_queue_len(&w_adapter->host_tx_queue[VO_Q_STA]));
	seq_printf(seq, "total_sta_vo_pkt_send:%8d\t",
	                      w_adapter->total_tx_data_sent[VO_Q_STA]);
	seq_printf(seq,
		   "total_sta_vo_packets dropped: %d\n",
		   (uint32)w_adapter->total_tx_data_dropped[VO_Q_STA]);

	seq_printf(seq, "total_sta_vi_pkt_queued: %8d\t",
	                      get_skb_queue_len(&w_adapter->host_tx_queue[VI_Q_STA]));
	seq_printf(seq, "total_sta_vi_pkt_send:%8d\t",
	                      w_adapter->total_tx_data_sent[VI_Q_STA]);
	seq_printf(seq,
		   "total_sta_vi_packets dropped: %d\n",
		   (uint32)w_adapter->total_tx_data_dropped[VI_Q_STA]);

	seq_printf(seq, "total_sta_be_pkt_queued: %8d\t",
	                      get_skb_queue_len(&w_adapter->host_tx_queue[BE_Q_STA]));
	seq_printf(seq, "total_sta_be_pkt_send:%8d\t",
	                      w_adapter->total_tx_data_sent[BE_Q_STA]);
	seq_printf(seq,
		   "total_sta_be_packets dropped: %d\n",
		   (uint32)w_adapter->total_tx_data_dropped[BE_Q_STA]);

	seq_printf(seq, "total_sta_bk_pkt_queued: %8d\t",
	                      get_skb_queue_len(&w_adapter->host_tx_queue[BK_Q_STA]));
	seq_printf(seq, "total_sta_bk_pkt_send:%8d\t",
	                      w_adapter->total_tx_data_sent[BK_Q_STA]);
	seq_printf(seq,
		   "total_sta_bk_packets dropped: %d\n",
		   (uint32)w_adapter->total_tx_data_dropped[BK_Q_STA]);

	seq_printf(seq,
		   "total_invlaid_pkt_send: %8d\t",
		   w_adapter->total_data_invalid_pkt_send);

	seq_printf(seq, "total_invalid_pkt_freed: %8d\n",
	                      w_adapter->total_invalid_pkt_freed);

	seq_printf(seq,
		   "total_bcast_mcast_pkt_freed: %8d\n",
		   w_adapter->total_bcast_mcast_pkt_freed);
	seq_printf(seq,
		   "total_null_pkt_rcvd : %d\t",
		   w_adapter->total_null_pkt_rcvd);
	seq_printf(seq,
		   "total_unknown_interrupts: %d\n",
		   w_adapter->total_unknown_interrupts);
	seq_printf(seq,
		   "total RX packets Received: %d\n",
		   (uint32)w_adapter->stats.rx_packets);
	seq_printf(seq,
		   "total RX packets dropped: %d\n",
		   (uint32)w_adapter->stats.rx_dropped);
	seq_printf(seq,
		   "total RX Big Size packets dropped: %d\n",
		   (uint32)w_adapter->big_size_pkts);
	seq_printf(seq,
		   "total TX packets dropped: %d\n",
		   (uint32)w_adapter->stats.tx_dropped);
	seq_printf(seq,
		   " BufferFull's Not observed: %d\n",
		   (uint32)w_adapter->no_buffer_fulls);

	if (netif_queue_stopped(w_adapter->dev))
		seq_printf(seq, "Net queue stopped\n");
	else
		seq_printf(seq, "Net queue running\n");

	seq_printf(seq, "\n");
	return 0;
}


static int onebox_proc_debug_zone_show(struct seq_file *seq, void *data)
{
	
	seq_printf(seq,
		   "The zones available are %#x\n",
		   onebox_zone_enabled);
	return 0;
}

static ssize_t onebox_proc_debug_zone_write(struct file *filp,
					    const char __user *buff,
					    size_t len,
					    loff_t *data)
{
	char user_zone[20] = {0};

	if (!len)
		return 0;

	if (len > 20)
		return -EINVAL;

	if (copy_from_user(user_zone, (void __user *)buff, len)) 
		return -EFAULT;
	else {
		int32 dbg_zone = 0;
		if ((user_zone[0] == '0') &&
		    (user_zone[1] == 'x' || 
		     user_zone[1] == 'X'))
			dbg_zone = simple_strtol(&user_zone[2], NULL, 16);
		else
			dbg_zone = simple_strtol(user_zone, NULL, 10);

		onebox_zone_enabled = dbg_zone;
	}
	return len;
}



static int onebox_proc_version_open(struct inode *inode, struct file *file)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	return single_open(file,
			   onebox_proc_version_show,
			   PDE_DATA(inode));
#else
	return single_open(file,
			   onebox_proc_version_show,
			   PDE(inode)->data);
#endif
}

static int onebox_proc_stats_open(struct inode *inode, struct file *file)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	return single_open(file,
			   onebox_proc_stats_show,
			   PDE_DATA(inode));
#else
	return single_open(file,
			   onebox_proc_stats_show,
			   PDE(inode)->data);
#endif
}

static int onebox_proc_debug_zone_open(struct inode *inode, struct file *file)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	return single_open(file,
			   onebox_proc_debug_zone_show,
			   PDE_DATA(inode));
#else
	return single_open(file,
			   onebox_proc_debug_zone_show,
			   PDE(inode)->data);
#endif
}
#if 0
static int onebox_proc_data_rate_open(struct inode *inode, struct file *file)
{
	return single_open(file, 
			   onebox_proc_data_rate_show,
			   PDE(inode)->data);
}
#endif


static const struct file_operations proc_version_ops = {
	.open    = onebox_proc_version_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};	

#if 1
static const struct file_operations proc_stats_ops = {
	.open    = onebox_proc_stats_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};	

#endif	

static const struct file_operations proc_debug_zone_ops = {
	.open    = onebox_proc_debug_zone_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.write   = onebox_proc_debug_zone_write,
};

#if  0 /* coex */
static const struct file_operations proc_data_rate_ops = {
	.open    = onebox_proc_data_rate_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.write   = onebox_proc_data_rate_write,
};	
#endif


/**
 * This function initializes the proc file system entry.
 *
 * @param w_adapter   Pointer To Driver Priv Area
 *
 * @return
 * This function return the status success or failure.
 */
ONEBOX_STATUS setup_wlan_procfs(WLAN_ADAPTER w_adapter)
{
	struct proc_dir_entry *entry = NULL;

	w_adapter->wlan_proc_entry = proc_mkdir(w_adapter->dev->name, NULL);
	if (w_adapter->wlan_proc_entry == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, ("Unable to create onebox-mobile dir entry"));
		return ONEBOX_STATUS_FAILURE;
	} else {
		entry = proc_create_data("stats",
					 0,
					 w_adapter->wlan_proc_entry,
					 &proc_stats_ops,
					 w_adapter);
		if (entry == NULL) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("HAL : Unable to create proc entry [stats]\n")));
			return ONEBOX_STATUS_FAILURE;
		}


	}
	return ONEBOX_STATUS_SUCCESS;
}

/**
 * Removes the previously created proc file entries in the
 * reverse order of creation
 *
 * @param  void
 * @return void 
 */
void destroy_wlan_procfs(WLAN_ADAPTER w_adapter)
{
	remove_proc_entry("stats", w_adapter->wlan_proc_entry);
	remove_proc_entry(w_adapter->dev->name, NULL);
	return;
}

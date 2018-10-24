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

#include "onebox_common.h"
#include "onebox_linux.h"
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/file.h>
#include "onebox_zone.h"
#include "onebox_host_intf_ops.h"

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
static int onebox_proc_version_show(struct seq_file *seq, void *data)
{
	PONEBOX_ADAPTER adapter = seq->private;

	if (adapter->device_model == RSI_DEV_9116) {
		adapter->driver_ver.major = RS9116_DRIVER_MAJOR;
		adapter->driver_ver.minor = RS9116_DRIVER_MINOR;
		adapter->driver_ver.release_num = RS9116_DRIVER_RELEASE_NUM;
		adapter->driver_ver.patch_num = RS9116_DRIVER_PATCH_NUM;
	} else {
		adapter->driver_ver.major = RS9113_DRIVER_MAJOR;
		adapter->driver_ver.minor = RS9113_DRIVER_MINOR;
		adapter->driver_ver.release_num = RS9113_DRIVER_RELEASE_NUM;
		adapter->driver_ver.patch_num = RS9113_DRIVER_PATCH_NUM;
	}

	seq_printf(seq, 
		   "UMAC : %d.%d.%d.%d\nLMAC : %d.%d.%d.%d\n",
		   adapter->driver_ver.major,
		   adapter->driver_ver.minor, 
		   adapter->driver_ver.release_num, 
		   adapter->driver_ver.patch_num,
		   adapter->lmac_ver.major,
		   adapter->lmac_ver.minor, 
		   adapter->lmac_ver.release_num, 
		   adapter->lmac_ver.patch_num);
		   //adapter->lmac_ver.ver.info.fw_ver[0]);
	return 0;
}	

static void show_assetq_stats(struct seq_file *seq, uint32 asset_id, uint32 idx)
{
	PONEBOX_ADAPTER adapter = seq->private;
	uint8 asset_state[][10] = {
		"ACTIVE",
		"DORMANT",
		"EXTINCT",
	};
	struct driver_assets *d_assets = adapter->d_assets; 

	seq_printf(seq, "    Driver asset state...........: %s\n",
			asset_state[idx]);
	seq_printf(seq, "    Total packets Q'ed...........: %6d\n",
			adapter->tot_pkts_qed[asset_id]);
	seq_printf(seq, "    Remaining packets in Queue...: %6d\n",
			get_skb_queue_len(&adapter->coex_queues[asset_id]));
	seq_printf(seq, "    Total packets sent on air....: %6d\n",
			adapter->tot_pkts_sentout[asset_id]);
	seq_printf(seq, "    Total packets dropped by HAL.: %6d\n\n",
			adapter->tot_pkts_dropped[asset_id]);
	seq_printf(seq, "    TX_ACCESS....................: %6d\n\n",
			d_assets->techs[idx].tx_access);
	
	return;
}

static int onebox_proc_stats_show(struct seq_file *seq, void *data)
{
	uint32 idx = 0;
	PONEBOX_ADAPTER adapter = seq->private;
	struct driver_assets *d_assets = adapter->d_assets; 

#if 0
/* -------> */
	struct ieee80211com *ic = &adapter->vap_com;
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
	if (adapter->fsm_state <= FSM_MAC_INIT_DONE)
		seq_printf(seq,
			   "%s",
			   fsm_state_lp[adapter->fsm_state]);
	seq_printf(seq,
		   "(%d)\n\n", 
	           adapter->fsm_state);
#endif

		seq_printf(seq,"   COMMON HAL FSM_STATE.........: %6d\n",
			   d_assets->common_hal_fsm);

		seq_printf(seq,"   COMMON HAL TX_ACCESS.........: %6d\n",
			   d_assets->common_hal_tx_access);
	if (adapter->Driver_Mode == RF_EVAL_MODE_ON)
		seq_printf(seq,
			   "total PER packets sent : %d\n",
			   adapter->total_per_pkt_sent);

	seq_printf(seq, "WLAN QUEUE:\n");

	if (d_assets->techs[WLAN_ID].drv_state == MODULE_ACTIVE)
		idx = 0;
	else if (d_assets->techs[WLAN_ID].drv_state == MODULE_INSERTED)
		idx = 1;
	else
		idx = 2;

	show_assetq_stats(seq, WLAN_Q, idx);
	
	seq_printf(seq, "BLUETOOTH QUEUE:\n");

	if (d_assets->techs[BT_ID].drv_state == MODULE_ACTIVE)
		idx = 0;
	else if (d_assets->techs[BT_ID].drv_state == MODULE_INSERTED)
		idx = 1;
	else
		idx = 2;

	show_assetq_stats(seq, BT_Q, idx);

	seq_printf(seq, "ZIGBEE QUEUE:\n");

	if (d_assets->techs[ZB_ID].drv_state == MODULE_ACTIVE)
		idx = 0;
	else if (d_assets->techs[ZB_ID].drv_state == MODULE_INSERTED)
		idx = 1;
	else
		idx = 2;

	show_assetq_stats(seq, ZIGB_Q, idx);

	seq_printf(seq, "VERY IMPORTANT PACKET(VIP) QUEUE:\n"); // very important packet
	seq_printf(seq, "    Total Packets Q'ed...........: %6d\n",
			adapter->tot_pkts_qed[VIP_Q]);
	seq_printf(seq, "    Remaining packets in queue...: %6d\n",
			get_skb_queue_len(&adapter->coex_queues[VIP_Q]));
	seq_printf(seq, "    Total Packets sent on air....: %6d\n",
			adapter->tot_pkts_sentout[VIP_Q]);
	seq_printf(seq, "    Total Packets dropped by HAL.: %6d\n",
			adapter->tot_pkts_dropped[VIP_Q]);

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

int coex_cmd_usage (struct seq_file *seq, void *data)
{
  seq_printf(seq,"\n\nEg. echo 1 0 22 > /proc/onebox-hal/coex_cmd \n");
  seq_printf(seq,"\nUsage: echo <slotting stop or start> <proto id to stop> <protocol slot time> \n");
  seq_printf(seq,"\n<slotting stop or start> : WHETHER TO STOP/STOP SLOTTING. \n");
  seq_printf(seq,"1                     — START SLOTTING.\n");
  seq_printf(seq,"0                     — STOP SLOTTING. RADIO WILL BE WITH <PROTOCOL ID> PROTOCOL UNTIL USER ENABLES SLOTTING. \n");
  seq_printf(seq,"<proto id to stop> :  — PROTOCOL ID OF WHICH SLOTTING SHOULD STOP,");
  seq_printf(seq," [0- BT-LE; 1-BT; 2-WLAN; 3-ZIGBEE ]\n");
  seq_printf(seq,"<protocol n slot time> : IS IN TERMS OF MILLI SECONDS AND CAN BE ANY VALUE \n\n");
	return 0;
}
ssize_t onebox_proc_coex_cmd(struct file *filp,
					    const char __user *buff,
					    size_t len,
					    loff_t *data)
{
  char *cmd = NULL;
  coex_cmd_t *coex_cmd_p = NULL;
	gfp_t flag;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)PDE_DATA(filp->f_inode);
#else
	PONEBOX_ADAPTER adapter = (PONEBOX_ADAPTER)PDE(filp->f_mapping->host)->data;
#endif
	struct driver_assets *d_assets = adapter->d_assets;
  if (!len)
    return -EINVAL;
  if (len < 6)
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(KERN_ERR"Invalid arguments please refer Usage :\n cat /proc/onebox-hal/coex_cmd  len =%d \n",len));
    return -EINVAL;
  }
    flag = (in_atomic() || irqs_disabled())? GFP_ATOMIC : GFP_KERNEL;
    cmd = (char*)kzalloc(len,flag);
    coex_cmd_p = kzalloc(sizeof(coex_cmd_t),flag);
      if(cmd == NULL || coex_cmd_p == NULL)
        return -ENOMEM;
  if (copy_from_user(cmd, (void __user *)buff, len)) {
    kfree(cmd);
    kfree(coex_cmd_p);
    return -EFAULT;
  }
  cmd[0] = simple_strtol(&cmd[0], NULL, 10);
  if(!(cmd[0] < 0 || cmd[0] > 1)){
      coex_cmd_p->start_stop = cmd[0];
  } else {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("Invalid start_stop = %d [0-1] Allowed\n",cmd[0]));
    kfree(cmd);
    kfree(coex_cmd_p);
    return -EFAULT;
  }
  if(cmd[1] != 0x20) {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("INVALID FORMAT Please give space b/t elements\n"));
    kfree(cmd);
    kfree(coex_cmd_p);
    return -EINVAL;
  }
#define BT_LE 0
#define BT_CLASSIC 1
#define WIFI 2
#define ZIGBEE 3
  cmd[2] = simple_strtol(&cmd[2], NULL, 10);
  if(!(cmd[2] < 0 || cmd[2] > 4)){
    coex_cmd_p->protocol_id = cmd[2];
    if(coex_cmd_p->start_stop) {
      switch (coex_cmd_p->protocol_id) {
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
                   ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("Invalid protocol_id  = %d [0-4] Allowed,Please check the COEX = %d mode in common_insert.sh \n",coex_cmd_p->protocol_id,d_assets->oper_mode));
                   kfree(cmd);
                   kfree(coex_cmd_p);
                   return -EINVAL;
                 }
      }
    }
  } else {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("Invalid Protocol_ID  = %d [0-4] Allowed\n",cmd[2]));
    kfree(cmd);
    kfree(coex_cmd_p);
    return -EINVAL;
  }
  if(cmd[3] != 0x20) {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("Invalid Format Please give space b/t elements\n"));
    kfree(cmd);
    kfree(coex_cmd_p);
    return -EINVAL;
  }
  *(int32*)&cmd[4] = simple_strtol(&cmd[4], NULL, 10);
  if((*(int32*)&cmd[4] >= 0) && (*(int32*)&cmd[4] <= 0x7FFFFFFF)) {
    coex_cmd_p->slotting_time = *(int32*)&cmd[4];
  }
  else{
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,("Invalid slotting_time [0 - 0x7FFFFFFF] \n"));
    kfree(cmd);
    kfree(coex_cmd_p);
    return -EINVAL;
  }
  adapter->coex_osi_ops->send_coex_configuration(adapter,coex_cmd_p);
    kfree(cmd);
    kfree(coex_cmd_p);
  return len;
}
int onebox_proc_coex_cmd_usage (struct inode *inode, struct file *file)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	return single_open(file,
			   coex_cmd_usage,
			   PDE_DATA(inode));
#else
	return single_open(file,
			   coex_cmd_usage,
			   PDE(inode)->data);
#endif
}
static const struct file_operations proc_version_ops = {
	.open    = onebox_proc_version_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};	

static const struct file_operations proc_stats_ops = {
	.open    = onebox_proc_stats_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};	

static const struct file_operations proc_debug_zone_ops = {
	.open    = onebox_proc_debug_zone_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.write   = onebox_proc_debug_zone_write,
};	

#ifdef USE_SDIO_INTF
static const struct file_operations proc_sdio_ops = {
	.open    = onebox_proc_sdio_cmd_usage,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.write   = onebox_proc_sdio_cmd,
};
#endif 
static const struct file_operations proc_coex_ops = {
	.open    = onebox_proc_coex_cmd_usage,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.write   = onebox_proc_coex_cmd,
};	
static const struct file_operations proc_master_ops = {
	.open    = onebox_proc_master_reg,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.write   = onebox_proc_master_cmd,
};	
static const struct file_operations proc_gpio_ops = {
	.open    = onebox_proc_gpio_read,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.write   = onebox_proc_gpio_config,
};	
static const struct file_operations proc_modem_pll_ops = {
	.open    = onebox_proc_modem_pll_read,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.write   = onebox_proc_modem_pll_config,
};	
static const struct file_operations proc_soc_pll_ops = {
	.open    = onebox_proc_soc_pll_read,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.write   = onebox_proc_soc_pll_config,
};	



static int onebox_proc_sdio_stats_show(struct seq_file *seq, void *data)
{

	PONEBOX_ADAPTER adapter = seq->private;

	seq_printf(seq,
		   "total_sdio_interrupts: %d\n",
		   adapter->sdio_int_counter);
	seq_printf(seq,
		   "total_sdio_interrupts_zero_status: %d\n",
		   adapter->sdio_intr_status_zero);
	seq_printf(seq,
		   "sdio_msdu_pending_intr_count: %d\n", 
		   adapter->total_sdio_msdu_pending_intr);
	seq_printf(seq,
		   "sdio_buff_status_count : %d\n",
		   adapter->buf_status_interrupts);
	seq_printf(seq,
		   "sdio_unknown_intr_count: %d\n",
		   adapter->total_sdio_unknown_intr);
	
	return 0;
}
static int onebox_proc_sdio_stats_open(struct inode *inode, struct file *file)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0))
	return single_open(file,
			   onebox_proc_sdio_stats_show,
			   PDE_DATA(inode));
#else
	return single_open(file,
			   onebox_proc_sdio_stats_show,
			   PDE(inode)->data);
#endif
}

static const struct file_operations proc_sdio_stats_ops = {
	.open    = onebox_proc_sdio_stats_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};	

/**
 * This function initializes the proc file system entry.
 *
 * @param adapter   Pointer To Driver Priv Area
 *
 * @return
 * This function return the status success or failure.
 */
struct proc_dir_entry* init_proc_fs(void *adapter, uint8 *name)
{
	struct proc_dir_entry *entry = NULL;
	struct proc_dir_entry *onebox_entry;

	onebox_entry = proc_mkdir(name, NULL);
	if (onebox_entry == NULL) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             ("onebox_init_proc: Unable to create dir entry"));
			return NULL;
	} else {
		entry = proc_create_data("version",
					 0,
					 onebox_entry, 
					 &proc_version_ops,
					 adapter);
		if (entry == NULL) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("HAL : Unable to create proc entry [version]\n"));
			return NULL;
		}

		entry = proc_create_data("stats",
					 0,
					 onebox_entry,
					 &proc_stats_ops,
					 adapter);
		if (entry == NULL) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("HAL : Unable to create proc entry [stats]\n"));
			return NULL;
		}

		entry = proc_create("debug_zone",
				    0,
				    onebox_entry,
				    &proc_debug_zone_ops);
		if (entry == NULL) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("HAL : Unable to create proc entry [debug_zone]\n"));
			return NULL;
		}

		entry = proc_create_data("sdio_stats",
				0,
				onebox_entry,
				&proc_sdio_stats_ops, 
				adapter);
		if (entry == NULL) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("HAL : Unable to create proc entry [debug_zone]\n"));
			return NULL;
		}

		entry = proc_create_data("coex_cmds",
				0,
				onebox_entry,
        &proc_coex_ops,
				adapter);
		if (entry == NULL) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("HAL : Unable to create proc entry [sdio_cmds]\n"));
			return NULL;
		}
#ifdef USE_SDIO_INTF
		entry = proc_create_data("sdio_cmds",
				0,
				onebox_entry,
				&proc_sdio_ops, 
				adapter);
		if (entry == NULL) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("HAL : Unable to create proc entry [sdio_cmds]\n"));
			return NULL;
		}
#endif
		entry = proc_create_data("master_read_write",
				0,
				onebox_entry,
				&proc_master_ops, 
				adapter);
		if (entry == NULL) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("HAL : Unable to create proc entry [master_read_write]\n"));
			return NULL;
		}
		entry = proc_create_data("gpio",
				0,
				onebox_entry,
				&proc_gpio_ops, 
				adapter);
		if (entry == NULL) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("HAL : Unable to create proc entry [gpio]\n"));
			return NULL;
		}
		entry = proc_create_data("modem_pll",
				0,
				onebox_entry,
				&proc_modem_pll_ops, 
				adapter);
		if (entry == NULL) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("HAL : Unable to create proc entry modem_pll\n"));
			return NULL;
		}
		entry = proc_create_data("soc_pll",
				0,
				onebox_entry,
				&proc_soc_pll_ops, 
				adapter);
		if (entry == NULL) {
			ONEBOX_DEBUG( ONEBOX_ZONE_ERROR, ("HAL : Unable to create proc entry soc_pll\n"));
			return NULL;
		}
	}
	return onebox_entry;
}

/**
 * Removes the previously created proc file entries in the
 * reverse order of creation
 *
 * @param  void
 * @return void 
 */
void proc_entry_remove(struct proc_dir_entry *onebox_entry, uint8 *proc_name)
{
	ONEBOX_DEBUG( ONEBOX_ZONE_INFO, ("HAL :Removing HAL procfs\n"));
	remove_proc_entry("version", onebox_entry);
	remove_proc_entry("stats", onebox_entry);
	remove_proc_entry("debug_zone", onebox_entry);
	remove_proc_entry("sdio_stats", onebox_entry);
	remove_proc_entry("coex_cmds", onebox_entry);
#ifdef USE_SDIO_INTF
	remove_proc_entry("sdio_cmds", onebox_entry);
#endif
	remove_proc_entry("master_read_write", onebox_entry);
	remove_proc_entry("gpio", onebox_entry);
	remove_proc_entry("modem_pll", onebox_entry);
	remove_proc_entry("soc_pll", onebox_entry);
	remove_proc_entry(proc_name, NULL);

	return;
}

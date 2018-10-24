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

#ifndef __ONEBOX_COEX_H__
#define __ONEBOX_COEX_H__

#define GS_CARD_DETACH			0 /* card status */
#define GS_CARD_ABOARD			1 

/**
 * MODULE_REMOVED: 
 * modules are not installed or the modules are
 * uninstalled after an installation.
 *
 * MODULE_INSERTED:
 * modules are installled and the layer is not yet initialised.
 * or the layer is deinitialized.
 *
 * MODULE_ACTIVE:
 * modules are installed and the layer is initialised.
 *
 */
#define HOST_INTF_SDIO 0
#define HOST_INTF_USB 1

#define MODULE_REMOVED		0
#define MODULE_INSERTED		1
#define MODULE_ACTIVE		2

#define FW_INACTIVE		0
#define FW_ACTIVE		1

#define BT_CARD_READY_IND	0x89
#define WLAN_CARD_READY_IND	0x0
#define COMMON_HAL_CARD_READY_IND	0x0
#define ZIGB_CARD_READY_IND	0xff

#define COEX_Q      0 
#define BT_Q        1 
#define WLAN_Q      2 
#define VIP_Q       3 
#define ZIGB_Q      4

#define COEX_TX_Q      0 
#define ZIGB_TX_Q      1
#define BT_TX_Q        2 
#define WLAN_TX_M_Q    4 
#define WLAN_TX_D_Q    5 

#define COEX_PKT      0 
#define ZIGB_PKT      1
#define BT_PKT        2 
#define WLAN_PKT      3 

#define MAX_IDS  3
#define WLAN_ID  0
#define BT_ID    1
#define ZB_ID    2

#define ASSET_NAME(id) \
	  (id == WLAN_ID)  ? "wlan_asset" : \
	   ((id == BT_ID)  ? "bluetooth_asset" : \
	    ((id == ZB_ID) ? "zigbee_asset" : "null"))

#define COMMAN_HAL_WAIT_FOR_CARD_READY 1
#define COMMON_HAL_SEND_CONFIG_PARAMS 2
#define COMMON_HAL_TX_ACCESS 3
#define COMMON_HAL_WAIT_FOR_PROTO_CARD_READY 4

/***** PROTOCOL **************/
#define WLAN_PROTOCOL BIT(0)
#define BT_PROTOCOL   BIT(1)
#define ZIGB_PROTOCOL BIT(2)

#define PROTOCOL_ENABLE  1
#define PROTOCOL_DISABLE 0

#include "onebox_thread.h"

typedef struct coex_cmd_s {
  uint8 start_stop ;
  uint8 protocol_id;
  uint32 slotting_time;
}__attribute__ ((packed))coex_cmd_t;
typedef struct programming_stats_s {
  unsigned char start_stop ;
  uint32 interval;
} programming_stats_t;
typedef struct prog_structure_s {
  unsigned char prog_type ;
  unsigned char structure_present; 
  unsigned int TA_RAM_ADDRESS;
  unsigned int bb_rf_flags;
  unsigned char len;
} prog_structure_t;
typedef struct wlan_9116_features_s {
  uint8 pll_mode;
  uint8 rf_type;
  uint8 wireless_mode;
  uint8 afe_type;
  uint8 enable_ppe;
  uint8 dpd;
  uint8 SIFSTransmitenable;
  uint32 pwrsave_options;
}w_9116_features_t;
typedef struct gpio_registers_s
{
  uint32_t read_write;
  uint32_t id;
  uint32_t mode;
  uint32_t value;
  uint32_t direction;
  unsigned int address ;          
}gpio_reg_t;
#define UNGATE_CLOCK                  0x00830001
#define GATE_CLOCK                    0x00830000
#define NWP_BASE_ADDR                 0x41300000
#define M4_CLK_ADDR                   (NWP_BASE_ADDR + 0x110)      
#define EGPIO_BASE_ADDR          0x2404C000
typedef struct ipmu_params_s {
    unsigned int address;
    unsigned int value;
    unsigned char mode ;
} ipmu_params_t;

typedef struct common_bb_rf_params_s
{ 
	unsigned short Data[1024];
	unsigned short no_of_fields;
	unsigned short no_of_values;
	unsigned char value;
	unsigned char soft_reset;
  unsigned char protocol_id;

} common_bb_rf_params_t;

#ifndef __RSI_DEV_MODEL__
#define __RSI_DEV_MODEL__
typedef enum rsi_dev_m {
	RSI_DEV_9110=0,
	RSI_DEV_9113,
	RSI_DEV_9116
} rsi_dev_t;
#endif

struct driver_assets {
	uint32 card_state;	
	uint16 ta_aggr;
	uint8  asset_role;
	uint8  coex_mode;
	struct wireless_techs {
		uint32 drv_state;
		uint32 fw_state;
		uint32 buffer_status_reg_addr;
		int32  (*inaugurate)(struct driver_assets *d_assets);
		int32  (*disconnect)(struct driver_assets *d_assets);
		ONEBOX_STATUS (*onebox_get_pkt_from_coex)(struct driver_assets *d_assets, netbuf_ctrl_block_t *netbuf_cb);
		ONEBOX_STATUS (*onebox_get_buf_status)(uint8 buf_status, struct driver_assets *d_assets);
		ONEBOX_STATUS (*onebox_get_ulp_sleep_status)(uint8 sleep_status);
		void (*wlan_dump_mgmt_pending)(struct driver_assets *d_assets);//remove me after coex optimisations
		bool tx_intention;
		bool tx_access; /* Per protocol tx access */
		uint8 deregister_flags;
		wait_queue_head_t deregister_event;
		void *priv;
		uint32 default_ps_en;
		void (*schedule_pkt_tx)(struct driver_assets *d_assets);
		atomic_t pkt_write_pending;
	} techs[MAX_IDS];
	void *global_priv;
	void *pfunc;
	bool common_hal_tx_access;
	bool sleep_entry_recvd;
	bool ulp_sleep_ack_sent;
	struct semaphore tx_access_lock;
	struct semaphore wlan_init_lock;
	struct semaphore bt_init_lock;
	struct semaphore zigbee_init_lock;
	void (*update_tx_status)(struct driver_assets *, uint8 prot_id);
	ONEBOX_STATUS (*common_send_pkt_to_coex)(struct driver_assets *, netbuf_ctrl_block_t* netbuf_cb, uint8 hal_queue);
	uint32 common_hal_fsm;
	uint8 lp_ps_handshake_mode;
	uint8 ulp_ps_handshake_mode;
	uint8 rf_power_val;
	uint8 device_gpio_type;
	int32 (*update_usb_buf_status)(void *priv);
	spinlock_t usb_wlan_coex_write_pending;
	bool antenna_diversity;
	uint8 common_hal_fw_state;
	uint8  oper_mode ;
	uint8 wlan_rf_power_mode;
	uint8 bt_rf_power_mode;
	uint16 country_code; 
	uint16 peer_dist;
	uint8 region_code;
	uint8 zigb_rf_power_mode;
	uint8 obm_ant_sel_val;
	uint8 user_onboard_ant_val;
	uint8 host_intf_type;
	uint8 protocol_enabled;
	uint8 *mfi_signature;
	uint8 *mfi_certificate;
	ONEBOX_EVENT iap_event;
	uint16 retry_count;
  	uint16 bt_feature_bitmap;
	uint16 uart_debug;
    uint16 ext_opt;
    uint32 wlan_pwrsave_options;
    uint8 lmac_bcon_en_dis_threshold;
    uint8 host_intf_on_demand;

	ONEBOX_STATUS (*onebox_common_read_register)(struct driver_assets *d_assets, uint32 Addr, uint8 *data);
	ONEBOX_STATUS (*onebox_common_write_register)(struct driver_assets *d_assets, uint8 reg_dmn,
			uint32 Addr, uint8 *data) ;
	ONEBOX_STATUS (*onebox_common_read_multiple)(struct driver_assets *d_assets,
			uint32 Addr, uint32 Count, 
			uint8 *data );

	ONEBOX_STATUS (*onebox_common_write_multiple)(struct driver_assets *d_assets,
			uint32 Addr, 
			uint8 *data,
			uint32 Count,
			netbuf_ctrl_block_t *netbuf_cb);
	ONEBOX_STATUS (*onebox_common_ta_write_multiple)(struct driver_assets *,
			uint32 Addr,
			uint8 *data,
			uint32 Count);
	ONEBOX_STATUS (*onebox_common_ta_read_multiple)(struct driver_assets *,
			uint32 Addr, 
			uint8 *data,
			uint32 Count);
	ONEBOX_STATUS (*onebox_common_master_reg_read)(struct driver_assets *d_assets,
			uint32 addr,
			uint32 *data,
			uint16 size) ;
	ONEBOX_STATUS (*onebox_common_master_reg_write)(struct driver_assets *d_assets,
			unsigned long addr,
			unsigned long data,
			uint16 size) ;
void (*onebox_send_coex_configuration)(struct driver_assets *d_assets,coex_cmd_t *coex_cmd_p);
	uint8 wlan_mac_address[6];
	uint8 band_version; //Dual band or single band
	uint8 onboard_antenna;//This is valid for mfg_sw versions > 3.1 
	uint8 mfg_major_verion;//Major MFG S/wversion number 
	uint8 mfg_minor_verion; //Minor version number
	uint8 module_type; //HPM or NON-HPM or Wise MCU
	uint8 bt_deregister_sent;
	uint8 dreg_frame;
	rsi_dev_t device_model;
    w_9116_features_t w_9116_features;
    uint8 disable_programming; 
    ONEBOX_STATUS (*onebox_map_ulp_gpio_to_ta_gpio)(struct driver_assets *driver_assets, uint32 gpio_idx);
    common_bb_rf_params_t common_bb_rf_params;
    ONEBOX_STATUS (*onebox_wait_for_rf_prog_frame_rsp)(struct driver_assets *d_assets);
	uint16 txpkt_lifetime;
#ifdef ONEBOX_CONFIG_PUF
#define ACTIVATION_CODE_SIZE  1192
#define MAX_KEY_SIZE    32
#define KEY_CODE_SIZE   44
#define MAX_DATA_SIZE	512
	uint16 puf_status;
	uint8 puf_activation_code[ACTIVATION_CODE_SIZE];
	uint8 puf_key_code[KEY_CODE_SIZE];
	uint16 puf_recv_len;
	uint8 puf_key[MAX_KEY_SIZE];
	uint8 enc_dec_mac_data[MAX_DATA_SIZE];
	ONEBOX_EVENT  puf_event;
#endif
};

#define WLAN_TECH d_assets->techs[WLAN_ID]
#define BT_TECH d_assets->techs[BT_ID]
#define ZB_TECH d_assets->techs[ZB_ID]
/*
 * Generic Netlink Sockets
 */
struct genl_cb {
	uint8 gc_cmd, *gc_name;
	int32 gc_seq, gc_pid;
	int32 gc_assetid, gc_done;
	int32 gc_n_ops;
	void  *gc_drvpriv;
	struct nla_policy  *gc_policy;
	struct genl_family *gc_family;
	struct genl_ops    *gc_ops; 
	struct genl_info   *gc_info;
	struct sk_buff     *gc_skb;
};

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 6, 11)
# define get_portid(_info) (_info)->snd_pid
#else
# define get_portid(_info) (_info)->snd_portid
#endif

/*==========================================================/
 * attributes (variables): the index in this enum is 
 * used as a reference for the type, userspace application 
 * has to indicate the corresponding type the policy is 
 * used for security considerations
 *==========================================================*/

enum {
	RSI_USER_A_UNSPEC,
	RSI_USER_A_MSG,
	__RSI_USER_A_MAX,
};

/*=================================================================/ 
 * commands: enumeration of all commands (functions),
 * used by userspace application to identify command to be executed
 *=================================================================*/

enum {
	RSI_USER_C_UNSPEC,
	RSI_USER_C_CMD,
	__RSI_USER_C_MAX,
};
#define RSI_USER_A_MAX (__RSI_USER_A_MAX - 1)
#define RSI_VERSION_NR 1

#endif

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

#ifndef __ONEBOX_COMMON_H__
#define __ONEBOX_COMMON_H__

#include <linux/netdevice.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/init.h>
#include <linux/etherdevice.h>
#include <linux/if.h>
#include <linux/wireless.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>

/*Bluetooth Specific */
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <linux/workqueue.h>

#include "onebox_datatypes.h"
#include "onebox_common.h"
#include "onebox_zone.h"

#if((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18))&& \
    (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)))
#include <linux/sdio/ctsystem.h>
#include <linux/sdio/sdio_busdriver.h>
#include <linux/sdio/_sdio_defs.h>
#include <linux/sdio/sdio_lib.h>
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#endif

#include "onebox_datatypes.h"

/* Kernel version between and including a & b */
#define KERNEL_VERSION_BTWN_2_6_(a,b) \
  ((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,a)) && \
  (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,b)))

#define KERNEL_VERSION_EQUALS_2_6_(a) \
  (LINUX_VERSION_CODE == KERNEL_VERSION(2,6,a))

/* Kernel version greater than equals */
#define KERNEL_VERSION_GREATER_THAN_2_6_(a)  \
 (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,a)) 

/* Kernel version less than or equal to */
#define KERNEL_VERSION_LESS_THAN_3_6(a)  \
 (LINUX_VERSION_CODE <= KERNEL_VERSION(3,6,a)) 

#define KERNEL_VERSION_LESS_THAN_3_12_(a) \
 (LINUX_VERSION_CODE <= KERNEL_VERSION(3,12,a)) 



#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

#define ONEBOX_ZONE_ACL_DATA            ONEBOX_BIT(21) /* For BT_ACL DATA PACKETS     */
#define ONEBOX_ZONE_SCO_DATA            ONEBOX_BIT(22) /* For BT_SCO DATA PACKETS     */



#define INVALID_QUEUE              0xff
#define NO_STA_DATA_QUEUES          4

#define ONEBOX_HEADER_SIZE          18 /* Size of PS_POLL */
#define TX_VECTOR_SZ                12

#define ONEBOX_TOTAL_PWR_VALS_LEN   30
#define ONEBOX_BGN_PWR_VALS_LEN     30
#define ONEBOX_AN_PWR_VALS_LEN      18

#define ONEBOX_EXTERN   extern
#define ONEBOX_STATIC   static
#define HEX_FILE       1
#define BIN_FILE       0
#define ONEBOX_SDIO_FRM_TRF_SIZE   (256 - 16)

#define FRAME_DESC_SZ                    16
#define OFFSET_EXT_DESC_SIZE    		 4
#define ONEBOX_DESCRIPTOR_SZ             64
#define ONEBOX_EXTENDED_DESCRIPTOR       12
#define ONEBOX_RCV_BUFFER_LEN      2000

typedef int ONEBOX_STATUS;


#ifndef DANUBE_ADDRESSING
 #define ONEBOX_HOST_VIR_TO_PHY(virt_addr)  virt_to_phys((void *)virt_addr)
 #define ONEBOX_HOST_PHY_TO_VIR(phy_addr)  phys_to_virt((uint32)phy_addr)
#else
 #define ONEBOX_HOST_VIR_TO_PHY(virt_addr) ((void *)(((uint32)virt_addr) & 0x5FFFFFFF))
 #define ONEBOX_HOST_PHY_TO_VIR(phy_addr) (((uint32)phy_addr) | 0xA0000000)
#endif


#define ONEBOX_CPU_TO_LE16(x)   cpu_to_le16(x)
#define ONEBOX_CPU_TO_LE32(x)   cpu_to_le32(x)
#define ONEBOX_CPU_TO_BE16(x)   cpu_to_be16(x)

#define ETH_PROTOCOL_OFFSET             12
#define ETH_HDR_OFFSET                  0
#define ETH_HDR_LEN                     14


#define DMA_MEMORY                      1
#define NORMAL_MEMORY                   0


/*BT specific */
#define MAX_BER_PKT_SIZE    	1030
#define GANGES_BER_QUEUE_LEN    100
/* BT device states */
#define BT_STAND_BY     		0
#define BT_INQUIRY      		1
#define BT_PAGE         		2
#define BT_INQUIRY_SCAN     	3   
#define BT_PAGE_SCAN        	4
#define BT_INQUIRY_RESP     	5   
#define BT_SLAVE_PAGE_RESP  	6
#define BT_MASTER_PAGE_RESP 	6
#define BT_CONN         		7
#define BT_PARK         		8
#define BT_SLAVE_PAGE_RESP_2    6
#define BT_MASTER_PAGE_RESP_2   6
#define BT_CLASSIC_MODE     	0
#define BT_LE_MODE      		1

#define FREQ_HOP_ENABLE     0
#define FREQ_HOP_DISABLE    1

/* zigbee queues */
//#define BT_TA_MGMT_Q            0x0
//#define ZB_INTERNAL_MGMT_Q      0x0
#define BT_TA_MGMT_Q        	  0x6
#define ZB_DATA_Q                 0x7
#define ZB_INTERNAL_MGMT_Q        0x6
#define E2E_MODE_ON                      1

/* Bluetooth Queues */
#define BT_DATA_Q       	0x7
#define BT_TA_MGMT_Q        0x6
#define BT_INT_MGMT_Q       0x6
/* EPPROM_READ_ADDRESS */

#define WLAN_MAC_EEPROM_ADDR               40 
#define WLAN_MAC_MAGIC_WORD_LEN            01 
#define WLAN_HOST_MODE_LEN                 04 
#define MAGIC_WORD			   0x5A


/* sifs and slot time defines */
#define SIFS_TX_11N_VALUE 580
#define SIFS_TX_11B_VALUE 346
#define SHORT_SLOT_VALUE  360
#define LONG_SLOT_VALUE   640
#define OFDM_ACK_TOUT_VALUE 2720
#define CCK_ACK_TOUT_VALUE 9440
#define LONG_PREAMBLE    0x0000
#define SHORT_PREAMBLE    0x0001
#define REQUIRED_HEADROOM_FOR_BT_HAL	 16
#define ONEBOX_DEBUG_MESG_RECVD 1

/*Master access types*/
#define ONEBOX_MASTER_READ	11
#define ONEBOX_MASTER_WRITE	22
#define ONEBOX_MASTER_ACK	33

/*
 * Bluetooth-HCI
 */
#if ((LINUX_VERSION_CODE >= KERNEL_VERSION(3, 1, 0)) && \
     (LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)))
# define onebox_hci_dev_hold(_hdev) hci_dev_hold(_hdev)
# define onebox_hci_dev_put(_hdev)  hci_dev_put(_hdev)
#else
# define onebox_hci_dev_hold(_hdev)
# define onebox_hci_dev_put(_hdev)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
# define get_hci_drvdata(_hdev)        hci_get_drvdata(_hdev)
# define set_hci_drvdata(_hdev, _data) hci_set_drvdata(_hdev, _data)
#else
# define get_hci_drvdata(_hdev)        (_hdev)->driver_data
# define set_hci_drvdata(_hdev, _data) ((_hdev)->driver_data = _data)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
# define onebox_hci_recv_frame(_hd, _sk) hci_recv_frame(_sk)
#else
# define onebox_hci_recv_frame(_hd, _sk) hci_recv_frame(_hd, _sk)
#endif

typedef struct
{
	uint16  major;
	uint16  minor;
	uint8 release_num;
	uint8 patch_num;
	union 
	{
		struct
		{
			uint8 fw_ver[8];
		}info;
	}ver;
}__attribute__ ((packed))version_st;

typedef struct bb_rf_params_s
{ 
	unsigned char value;
	unsigned char no_of_fields;
	unsigned char no_of_values;
	unsigned char soft_reset;
	unsigned short Data[512];
} bb_rf_params_t;

typedef struct  __attribute__((__packed__)) bt_per_params_s
{
	unsigned char  enable;
	unsigned char bt_addr[6];
	unsigned char pkt_type;
	unsigned char link_type;
	unsigned char scrambler_seed;
	unsigned char edr_indication;
	unsigned char bt_rx_rf_chnl;
	unsigned char bt_tx_rf_chnl;
	unsigned char enable_hopping;
	unsigned int num_pkts;
	unsigned char mode;
	unsigned short esco_pkt_len;	
	unsigned char slot_mode;
	unsigned char pkt_length;
	unsigned char lt_addr;
	unsigned char tx_or_rx;
	unsigned char host_mode;
	unsigned char payload_data_type;
	unsigned char le_mode;
	unsigned char le_chnl;
	unsigned char tx_pwr_indx;
	unsigned char transmit_mode;
	unsigned char ant_select;
#if 0
	unsigned int crc_init;
	unsigned int access_addr;
#endif
}bt_per_params_t;

typedef struct bt_ber_param_s 
{
  struct {
    uint8 *data;
    uint16 len;
  } ber_pkts[1033];
  uint8 push_loc;
  uint8 pop_loc;
  uint16 num_pkts_avail;
} bt_ber_params_t;

typedef struct bt_ber_pkts_s{
        uint16 len;
        uint8 data[1032];
        uint16 num_pkts;
}get_ber_pkts_t;

#define BT_RF_POWER_MODE_REQ    0x55

#define TX_POWER_MODE_MASK	0x0F
#define RX_POWER_MODE_MASK	0xF0

#define BT_E2E_MODE_ON     1
#define RF_EVAL_MODE_ON    2


#define OP_WLAN_STA_MODE	BIT(0)
#define OP_WLAN_AP_MODE		BIT(1)
#define OP_BT_CLASSIC_MODE	BIT(2)
#define OP_BT_LE_MODE				BIT(3)
#define OP_ZIGBEE_MODE			BIT(4)
#define OP_BT_DUAL_MODE		  (BIT(2)|BIT(3))

typedef struct bt_priv *BT_ADAPTER;

#include "onebox_netbuf.h"
#include "onebox_coex.h"
#include "onebox_mgmt.h"
#include "onebox_bt_ops.h"
#include "onebox_os_intf_ops.h"
#include "onebox_sdio_intf.h"

/* Adapter structure */
struct bt_priv 
{

	/* Network Interface Specific Variables */
	struct hci_dev *hdev;
	onebox_netbuf_head_t		bt_tx_queue;
	uint32	fsm_state;
	uint32           core_init_done;
#ifdef RSI_CONFIG_ANDROID
	wait_queue_head_t rsi_btchr_read_wait;
#define QUEUE_SIZE 500
	struct sk_buff *rsi_skb_queue[QUEUE_SIZE];
	int rsi_skb_queue_front;
	int rsi_skb_queue_rear;
#endif
#define NAME_MAX_LENGTH         32
	uint8               name[NAME_MAX_LENGTH];
	struct net_device_stats  stats;
	struct onebox_os_intf_operations *os_intf_ops;
	struct onebox_osd_host_intf_operations *osd_host_intf_ops;
	struct onebox_osi_host_intf_operations *osi_host_intf_ops;
	struct onebox_osi_bt_ops         *osi_bt_ops;  // core ops + devdep ops
	struct onebox_bt_osd_operations  *osd_bt_ops;
	ONEBOX_STATUS (*onebox_send_pkt_to_coex)(struct driver_assets *, netbuf_ctrl_block_t* netbuf_cb, uint8 hal_queue);
	ONEBOX_EVENT    bt_per_event;

	struct semaphore bt_gpl_lock;
	/*version related variables*/
	version_st        driver_ver;
	version_st        ta_ver;
	version_st        lmac_ver;
	bt_per_params_t   bt_endpoint_params;//BT_MODE
	bb_rf_params_t	bb_rf_params;
	bt_ber_params_t ber_info;
	uint32 pkt_drop_cnt;
	uint8 *ber_packet;
	uint16 bt_ber_pkts;
	get_ber_pkts_t get_ber_pkts;
	uint8 cw_type;
	uint8 cw_sub_type;
	uint8 read_cmd;
	uint8 driver_mode;
	uint8 coex_mode;
	uint8 oper_mode;
	uint8 tx_is_in_progress;
	uint8 wait_cmd;
	uint8 bt_rf_power_mode;
	uint8 bt_proc_name[20];
	struct proc_dir_entry *bt_proc_entry;
	uint8 bt_rf_type;
    uint8 ble_tx_pwr_inx;
    uint8 ble_pwr_save_options;
	struct genl_cb *genl_cb;
	struct driver_assets *d_assets;
} __attribute__((__aligned__(4)));

extern uint32 onebox_bt_zone_enabled;

struct onebox_bt_osd_operations *onebox_get_bt_osd_operations_from_origin(void);
struct onebox_osi_bt_ops *onebox_get_osi_bt_ops(void);
ONEBOX_STATUS bt_read_pkt(BT_ADAPTER bt_adapter,
				netbuf_ctrl_block_t *netbuf_cb);
ONEBOX_STATUS setup_bt_procfs(BT_ADAPTER bt_adapter);
void destroy_bt_procfs(BT_ADAPTER bt_adapter);

#ifdef USE_BLUEZ_BT_STACK
int bluez_init(BT_ADAPTER);	
int bluez_deinit(BT_ADAPTER);	
ONEBOX_STATUS send_pkt_to_bluez(BT_ADAPTER bt_adapter, netbuf_ctrl_block_t *netbuf_cb);
#endif

#ifdef RSI_CONFIG_ANDROID
int rsi_bdroid_init(BT_ADAPTER);
void rsi_bdroid_deinit(BT_ADAPTER);
#endif

//#ifdef USE_GENL_BT_STACK
int32 btgenl_init(BT_ADAPTER bt_adapter);
int32 btgenl_deinit(BT_ADAPTER bt_adapter);
ONEBOX_STATUS send_pkt_to_btgenl(BT_ADAPTER bt_adapter, netbuf_ctrl_block_t *netbuf_cb);
//#endif

#define BB_READ                         0x0
#define BB_WRITE                        0x1
#define RF_READ                         0x2
#define RF_WRITE                        0x3
#define BT_PER_TRANSMIT                 0x4
#define BT_RECEIVE                      0x5
#define BUFFER_READ                     0x6
#define BUFFER_WRITE                    0x7
#define BT_PER_STATS 		                0x8
#define ANT_SEL                         0x9
#define BT_BER_PKT_CNT                  0xA
#define BT_BER_RECEIVE                  0xB
#define BT_BER_MODE                     0xC
#define BT_CW_MODE                      0xD 
#define TX_STATUS                       0xE
#define GET_DRV_COEX_MODE               0xF


#define BT_PER                          0x10
#define BT_BER                          0x11
#define BT_CW                           0x12

#define PER_BLE_TRANSMIT                0x13
#define PER_BLE_RECEIVE                 0x14
#define PER_BR_EDR_TRANSMIT             0x15
#define PER_BR_EDR_RECEIVE              0x16
#define BLE_AOA_AOD_TRANSMIT            0x17
#define BLE_AOA_AOD_RECEIVE             0x18

#define RF_EVAL_CLASSIC                 0x01
#define RF_EVAL_LE                      0x02
#define RF_EVAL_DUAL_MODE               0x03


#define TX_IS_IN_PROG				              -1
#define TX_IS_NOT_IN_PROG 		   		       1


/***************** END DRIVER DATA STRUCTURE TEMPLATES ******************/
#endif

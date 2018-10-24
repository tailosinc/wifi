/**
 * @file zb_common.h
 * @author 
 * @version 1.0
 *
 * @section LICENSE
 *
 * This software embodies materials and concepts that are confidential to Redpine
 * Signals and is made available solely pursuant to the terms of a written license
 * agreement with Redpine Signals
 *
 * @section DESCRIPTION
 *
 * This file contians the data structures and variables/ macros commonly
 * used in the driver .
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
#include "onebox_common.h"
#include "onebox_zone.h"

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

/* Rx filter word defines */
#define PROMISCOUS_MODE 0x0001
#define ALLOW_ANY_DATA  0x0002  
#define ALLOW_ANY_MGMT  0x0004       
#define ALLOW_ANY_CTRL  0x0008       
#define ALLOW_DATA_ASSOC_PEER 0x0010
#define ALLOW_MGMT_ASSOC_PEER 0x0020 
#define ALLOW_CTRL_ASSOC_PEER 0x0040
#define ALLOW_ANY_BEACON      0x0080 
#define ALLOW_ANY_PROBE_REQ   0x0100 
#define ALLOW_ANY_PROBE_RESP  0x0200 
#define ALLOW_BEACON_ASSOC    0x0400 

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


/* ZIGBEE PER DEFINES  -- START*/
#define BB_READ                         0x0
#define BB_WRITE                        0x1
#define RF_READ                         0x2
#define RF_WRITE                        0x3
#define ZIGB_PER_TRANSMIT               0x4
#define ZB_RECEIVE			            0x5
#define BUFFER_READ                     0x6
#define BUFFER_WRITE                    0x7
#define ZB_PER_STATS 		            0x8
#define ANT_SEL			                0x9
#define SET_ZIGB_CHAN					0xA
#define TX_STATUS     					0xB
#define ZIGB_CW_MODE     				0xC

#define ZIGB_PER        				0xFE 
#define ZIGB_E2E        				0x1

#define TX_IS_IN_PROG				    -1
#define TX_IS_NOT_IN_PROG 		   		 1
/* ZIGBEE PER DEFINES  -- END*/


typedef struct bb_rf_params_s
{ 
	unsigned char value;
	unsigned char no_of_fields;
	unsigned char no_of_values;
	unsigned char soft_reset;
	unsigned short Data[110];

} bb_rf_params_t;


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

#define ZIGB_DEREGISTER 0xff

typedef struct zb_priv *ZB_ADAPTER;

#include "onebox_netbuf.h"
#include "onebox_coex.h"
#include "onebox_mgmt.h"
#include "onebox_zigb_ops.h"
#include "onebox_os_intf_ops.h"
#include "onebox_sdio_intf.h"

/* Adapter structure */
struct zb_priv 
{

	struct net_device     *dev; /* Stores the netdevice pointer */
	/* Network Interface Specific Variables */
	onebox_netbuf_head_t		zigb_rx_queue;
#define NAME_MAX_LENGTH         32
	uint8               name[NAME_MAX_LENGTH];
	struct net_device_stats  stats;
	uint8       mac_addr[6];
	uint32           fsm_state;
	uint32           core_init_done;

	struct onebox_os_intf_operations *os_intf_ops;
	struct onebox_osd_host_intf_operations *osd_host_intf_ops;
	struct onebox_osi_host_intf_operations *osi_host_intf_ops;
	struct onebox_zigb_osd_operations    *zigb_osd_ops;
	struct onebox_osi_zigb_ops     *osi_zigb_ops;
	ONEBOX_STATUS (*onebox_send_pkt_to_coex)(struct driver_assets *, netbuf_ctrl_block_t* netbuf_cb, uint8 hal_queue);

	ONEBOX_EVENT    zb_per_event;
	struct semaphore zigb_gpl_lock;
	/*version related variables*/
	version_st        driver_ver;
	version_st        ta_ver;
	version_st        lmac_ver;
	struct genl_cb *genl_cb;
	/* per related variables start */
	bb_rf_params_t	bb_rf_params;
	uint8             cw_type;
	uint8             cw_sub_type;
	volatile uint8 	  cnfm_flag;
	uint8 			  read_cmd;
	uint8 			  wait_cmd;
	uint8 		      driver_mode;
	uint8 		      tx_is_in_progress;
	struct proc_dir_entry *zb_proc_entry;
	uint8 			  zb_proc_name[20];
	/* per related variables end */
	struct driver_assets *d_assets;
}; 


#define ZB_E2E_MODE_ON     1
#define RF_EVAL_MODE_ON    2
extern uint32 onebox_zigb_zone_enabled;


ONEBOX_STATUS setup_zigb_procfs(ZB_ADAPTER zb_adapter);
void destroy_zigb_procfs(ZB_ADAPTER zb_adapter);
ONEBOX_STATUS zigb_read_pkt(ZB_ADAPTER zb_adapter,
				netbuf_ctrl_block_t *netbuf_cb);
struct onebox_osi_zigb_ops *onebox_get_osi_zigb_ops(void);
struct onebox_zigb_osd_operations *onebox_get_zigb_osd_ops(void);
extern uint32 onebox_zone_enabled;

int32 zigb_register_genl(ZB_ADAPTER zb_adapter);
int32 zigb_unregister_genl(ZB_ADAPTER zb_adapter);
void onebox_common_app_send(ZB_ADAPTER zb_adapter, netbuf_ctrl_block_t *netbuf_cb);
/***************** END DRIVER DATA STRUCTURE TEMPLATES ******************/
#endif

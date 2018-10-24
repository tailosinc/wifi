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


#include "bt_common.h"
#include <net/bluetooth/hci.h>
#include "bt_hci.h"

#define ONEBOX_VAP_RIFS_AVL                0x01
#define ONEBOX_VAP_SHORT_GI_AVL            0x02
#define ONEBOX_VAP_AP_ISOLATION_AVL        0x04
#define ONEBOX_VAP_GREEN_FIELD_AVL         0x08

#define HCI_VERSION		0x06
#define HCI_REVISION		0x0200
#define LMP_VERSION		0x06
#define MANUFACTURER_NAME	0xffff
#define LMP_SUBVERSION		0x0000

#define HC_MAX_ACL_Data_Packet_Length			0x00c0//0x03fd 
#define HC_Synchronous_Data_Packet_Length 		0x0040//0x30
#define HC_Total_Num_ACL_Data_Packets			0x0008
#define HC_Total_Num_Synchronous_Data_Packets	0x0008//0x000a

#define PAGE_NUMBER		0x00
#define MAX_PAGE_NUMBER 	0x00

void print_rx_hci_cmd_type(BT_ADAPTER bt_adapter, uint16 ogcf, uint8 pkt_type)
{
	uint8 ogf;

	ogf = ogcf >> 10;

	switch (pkt_type)
	{
		case HCI_COMMAND_PKT:
		{
			uint8 ocf = ogcf & 0x3FF;
			switch(ogf)
			{
				case HCI_LINK_CNTRL_CMD:    
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s\n"), link_control_cmd_string[ocf]));
					break;
				}
				case HCI_LINK_POLICY_CMD:   
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s\n"), link_policy_cmd_string[ocf]));
					break;
				}
				case HCI_CNTRL_BB_CMD:      
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s\n"), link_bbp_ctrl_cmd_string[ocf]));
					break;
				}
				case HCI_INFO_PARAMS_CMD:   
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s\n"), link_info_cmd_string[ocf]));
					break;
				}
				case HCI_STATUS_PARAMS_CMD: 
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s\n"), link_status_cmd_string[ocf]));
					break;
				}
				case HCI_TESTING_CMD:       
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s\n"), link_testing_cmd_string[ocf]));
					break;
				}
				case HCI_LE_CNTRL_CMD:      
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s\n"), le_cmd_string[ocf]));
					break;
				}
				case HCI_VENDOR_SPECIFIC:
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("VENDOR SPECIFIC COMMAND\n")));
					break;
				}
				default:
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("UNKNOWN OGF\n")));
					break;
				}
			}
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n")));
			break;
		}
		case HCI_ACLDATA_PKT:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ACL_DATA, (TEXT("ACL DATA PACKET\n")));
			break;
		}
		case HCI_SCODATA_PKT:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_SCO_DATA, (TEXT("SCO DATA PACKET\n")));
			break;
		}
		case HCI_VENDOR_PKT:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("VENDOR PACKET\n")));
			break;
		}
		default:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("UNKNOWN PACKET TYPE\n")));
			break;
		}
	}
}

void print_hci_cmd_type(BT_ADAPTER bt_adapter, netbuf_ctrl_block_t *netbuf_cb)
{
  uint16 ogcf; 
	uint8 ogf;
  uint8 pkt_type;

  if(!netbuf_cb) {

			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Netbuf CB is NULL in %s Line %d\n"), __func__, __LINE__));
    return;
  }

  ogcf = *(uint16 *)&netbuf_cb->data[FRAME_DESC_SZ]; 
	ogf = ogcf >> 10;
  pkt_type = netbuf_cb->bt_pkt_type;
	switch (pkt_type)
	{
		case HCI_COMMAND_PKT:
		{
			uint8 ocf = ogcf & 0x3FF;
			switch(ogf)
			{
				case HCI_LINK_CNTRL_CMD:    
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s\n"), link_control_cmd_string[ocf]));
					break;
				}
				case HCI_LINK_POLICY_CMD:   
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s\n"), link_policy_cmd_string[ocf]));
					break;
				}
				case HCI_CNTRL_BB_CMD:      
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s\n"), link_bbp_ctrl_cmd_string[ocf]));
					break;
				}
				case HCI_INFO_PARAMS_CMD:   
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s\n"), link_info_cmd_string[ocf]));
					break;
				}
				case HCI_STATUS_PARAMS_CMD: 
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s\n"), link_status_cmd_string[ocf]));
					break;
				}
				case HCI_TESTING_CMD:       
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s\n"), link_testing_cmd_string[ocf]));
					break;
				}
				case HCI_LE_CNTRL_CMD:      
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("%s\n"), le_cmd_string[ocf]));
					break;
				}
				case HCI_VENDOR_SPECIFIC:
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("VENDOR SPECIFIC COMMAND\n")));
					break;
				}
				default:
				{
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("UNKNOWN OGF\n")));
					break;
				}
			}
      bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_INFO, netbuf_cb->data, netbuf_cb->len);
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\n")));
			break;
		}
		case HCI_ACLDATA_PKT:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ACL_DATA, (TEXT("ACL DATA PACKET\n")));
			break;
		}
		case HCI_SCODATA_PKT:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_SCO_DATA, (TEXT("SCO DATA PACKET\n")));
			break;
		}
		case HCI_VENDOR_PKT:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("VENDOR PACKET\n")));
      bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_INFO, netbuf_cb->data, netbuf_cb->len);
			break;
		}
		default:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("UNKNOWN PACKET TYPE\n")));
      bt_adapter->osi_bt_ops->onebox_dump(ONEBOX_ZONE_INFO, netbuf_cb->data, netbuf_cb->len);
			break;
		}
	}
}

/* Sending BlueTooth packet to device */
int32 send_bt_pkt(BT_ADAPTER bt_adapter, 
		   netbuf_ctrl_block_t *netbuf_cb)
{
	uint16 *frame_desc;
	uint32 status;
	uint16 pkt_len;

	if(bt_adapter->os_intf_ops->onebox_change_hdr_size(netbuf_cb, FRAME_DESC_SZ) < 0)
	{
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("%s: onebox_change_hdr_size failed\n"),__func__));
		return ONEBOX_STATUS_FAILURE;
	}
	frame_desc = (uint16 *)netbuf_cb->data;

	/* packet length without descriptor */
	pkt_len = netbuf_cb->len - FRAME_DESC_SZ;

	/* Assigning packet length */
	frame_desc[0] = pkt_len & 0xFFF;

	/* Assigning queue number */
	frame_desc[0] |= (ONEBOX_CPU_TO_LE16(BT_DATA_Q) & 0x7) << 12;

	/* Assigning packet type in first word */
	frame_desc[7] = ONEBOX_CPU_TO_LE16(netbuf_cb->bt_pkt_type);

	netbuf_cb->tx_pkt_type = BT_TX_Q;
	{
		/* Transmit packet dump */
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\nTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTX\n")));
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("TX PACKET LENGTH = %d\n"), netbuf_cb->len));
		//print_rx_hci_cmd_type(bt_adapter, *(uint16 *)&netbuf_cb->data[FRAME_DESC_SZ], netbuf_cb->bt_pkt_type);
		print_hci_cmd_type(bt_adapter, netbuf_cb);
		ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("\nTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTXTX\n")));
	}
	/* Sending packet to the hal interface layer */
	status = bt_adapter->onebox_send_pkt_to_coex(bt_adapter->d_assets, netbuf_cb, BT_Q);
	if (status != ONEBOX_STATUS_SUCCESS) 
	{ 
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		             (TEXT("%s: Failed To Write The Packet\n"),__func__));
	}
	return status;    
}


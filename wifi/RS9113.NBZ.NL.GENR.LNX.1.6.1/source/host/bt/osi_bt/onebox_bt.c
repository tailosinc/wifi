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
#include "onebox_bt_core.h"
#include "onebox_bt_per.h"

int32 bt_core_pkt_recv(BT_ADAPTER bt_adapter, netbuf_ctrl_block_t *netbuf_cb)
{
	ONEBOX_STATUS status;
	
	ONEBOX_DEBUG(ONEBOX_ZONE_INFO,
	    (TEXT("%s: sending to APP len %d, pkttype %d\n"),
	    __func__, netbuf_cb->len, netbuf_cb->bt_pkt_type));

  if( bt_adapter->driver_mode == RF_EVAL_MODE_ON )
  {
    status = bt_adapter->osd_bt_ops->onebox_send_pkt_to_genl(bt_adapter, netbuf_cb);
  }
  else
  {
#ifdef USE_BLUEZ_BT_STACK
    status = bt_adapter->osd_bt_ops->onebox_send_pkt_to_btstack(bt_adapter, netbuf_cb);
#else
    status = bt_adapter->osd_bt_ops->onebox_send_pkt_to_genl(bt_adapter, netbuf_cb);
#endif
  }
 	return status;
}

int32 core_bt_init(BT_ADAPTER bt_adapter)
{
  int32 rc;

  bt_adapter->os_intf_ops->onebox_netbuf_queue_init(&bt_adapter->bt_tx_queue);
#ifdef USE_BLUEZ_BT_STACK
  if( bt_adapter->driver_mode == BT_E2E_MODE_ON )
  {
    rc = bt_adapter->osd_bt_ops->onebox_btstack_init(bt_adapter);
    if (rc) {
      ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
          (TEXT("%s: btstack_init fail %d\n"),__func__, rc));
      return ONEBOX_STATUS_FAILURE;
    }
  }
#endif
#ifdef RSI_CONFIG_ANDROID
  bt_adapter->osd_bt_ops->onebox_bdroid_init(bt_adapter);
#endif

  if (bt_adapter->driver_mode == RF_EVAL_MODE_ON) {
	  rc = bt_adapter->osd_bt_ops->onebox_btgenl_init(bt_adapter);
	  if (rc) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, 
			(TEXT("%s: genl_init fail %d\n"),__func__, rc));
		return ONEBOX_STATUS_FAILURE;
	  }
  }
  bt_adapter->core_init_done = 1;

  return ONEBOX_STATUS_SUCCESS;
}

int32 core_bt_deinit(BT_ADAPTER bt_adapter)
{
  bt_adapter->core_init_done = 0;
  bt_adapter->os_intf_ops->onebox_queue_purge(&bt_adapter->bt_tx_queue);
#ifdef RSI_CONFIG_ANDROID
  bt_adapter->osd_bt_ops->onebox_bdroid_deinit(bt_adapter);
#endif
#ifdef USE_BLUEZ_BT_STACK
  if( bt_adapter->driver_mode == BT_E2E_MODE_ON )
  {
    bt_adapter->osd_bt_ops->onebox_btstack_deinit(bt_adapter);
  }
#endif

  if (bt_adapter->driver_mode == RF_EVAL_MODE_ON)
	  bt_adapter->osd_bt_ops->onebox_btgenl_deinit(bt_adapter);

  ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Uninitialized Procfs\n"), __func__));   

  return ONEBOX_STATUS_SUCCESS;	
}

int32 bt_xmit(BT_ADAPTER bt_adapter, netbuf_ctrl_block_t *netbuf_cb)
{

  	/* Drop Zero Length Packets */
	if (!netbuf_cb->len) {
    		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		    (TEXT("%s:Zero Length packet\n"),__func__));
      		goto xmit_fail;
  	}

  	/* Drop Packets if FSM state is not open */
  	if (bt_adapter->fsm_state != FSM_DEVICE_READY) {
    		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
			(TEXT("%s: FSM state not open\n"),__func__));
      		goto xmit_fail;
  	}
	
	bt_adapter->osi_bt_ops->onebox_send_pkt(bt_adapter, netbuf_cb);
	return ONEBOX_STATUS_SUCCESS;
xmit_fail:

	bt_adapter->stats.tx_dropped++;
  	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
	    (TEXT("%s:Failed to xmit packet\n"),__func__));
  	if (netbuf_cb) {
		bt_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb,0);
  	}
  	return ONEBOX_STATUS_SUCCESS;
}

int core_bt_deque_pkts(BT_ADAPTER bt_adapter)
{
	netbuf_ctrl_block_t *netbuf_cb;

	while(bt_adapter->os_intf_ops->onebox_netbuf_queue_len(&bt_adapter->bt_tx_queue))
	{
		netbuf_cb = bt_adapter->os_intf_ops->onebox_dequeue_pkt((void *)&bt_adapter->bt_tx_queue);
		if(netbuf_cb == NULL)
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("queue is empty but length is not zero in %s"), __func__));
			return ONEBOX_STATUS_FAILURE;
		}
		bt_adapter->osi_bt_ops->onebox_send_pkt(bt_adapter, netbuf_cb);
	}

	return ONEBOX_STATUS_SUCCESS;
}

/**
 * This routine dump the given data through the debugger..
 *
 * @param  Debug zone.  
 * @param  Pointer to data that has to dump.  
 * @param  Length of the data to dump.  
 * @return VOID. 
 */
void onebox_print_dump(int32 zone, UCHAR *vdata, int32 len)
{
	uint16 ii;

	if(!zone || !(zone & onebox_bt_zone_enabled))
		return;

	for (ii=0; ii< len; ii++) {
		if (!(ii % 16) && ii) {
			//ONEBOX_DEBUG(zone, (TEXT("\n%04d: "), ii));
			ONEBOX_DEBUG(zone, (TEXT("\n")));
		}
		ONEBOX_DEBUG(zone,(TEXT("%02x "),(vdata[ii])));
	}
	ONEBOX_DEBUG(zone, (TEXT("\n")));
}

int32 bt_cw(BT_ADAPTER bt_adapter, uint8 *data, int32 len)
{
  ONEBOX_STATUS rc;
  netbuf_ctrl_block_t *netbuf_cb;
  int i = 0;
  int j = 0;
  uint16 *cw_mode_buf_write_array;

  if(!data) {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Line %d Data is NULL\n"), __func__, __LINE__));   
    return ONEBOX_STATUS_FAILURE;
  }

  bt_adapter->os_intf_ops->onebox_memcpy(&bt_adapter->bb_rf_params, data, len);
  bt_adapter->os_intf_ops->onebox_memset(data,0, len);

  bt_adapter->read_cmd = 0;
  bt_adapter->wait_cmd = 0;

  switch( bt_adapter->bb_rf_params.value)
  {
    case BT_CW_MODE:
      {
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s: CW_MODE Request\n"), __func__));   
        data[0] = BT_CW_MODE;
        data[1] = bt_adapter->bb_rf_params.soft_reset;
        bt_adapter->cw_type = data[2] = bt_adapter->bb_rf_params.no_of_values; //cw_type
        bt_adapter->cw_sub_type = data[3] = bt_adapter->bb_rf_params.no_of_fields; //sub_type

        netbuf_cb = bt_adapter->os_intf_ops->onebox_alloc_skb(len + 
            REQUIRED_HEADROOM_FOR_BT_HAL);
        if (!netbuf_cb) {
          rc = -ENOMEM;
          if (rc)
          {
            ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
                (TEXT("%s: error(%d) occured \n"), __func__, rc)); 
            return rc;
          }
        }
        bt_adapter->os_intf_ops->onebox_reserve_data(netbuf_cb, 
            REQUIRED_HEADROOM_FOR_BT_HAL);
        bt_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len);
        bt_adapter->os_intf_ops->onebox_memcpy(netbuf_cb->data, data, len);
        netbuf_cb->bt_pkt_type = BT_PER; 
        rc = bt_adapter->osi_bt_ops->onebox_bt_xmit(bt_adapter, netbuf_cb);
        if (rc)
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
              (TEXT("%s: error(%d) occured \n"), __func__, rc)); 
          return rc;
        }
        bt_adapter->wait_cmd = 1;
        bt_adapter->os_intf_ops->onebox_reset_event(&(bt_adapter->bt_per_event));
        if(bt_adapter->os_intf_ops->onebox_wait_event(&(bt_adapter->bt_per_event), 3000)  == 0)
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
              (TEXT("%s:NO RESPONSE BEFORE SPECIFIED TIME \n"), __func__)); 
          return ONEBOX_STATUS_FAILURE;
        }
#if 1
        if( !bt_adapter->cw_type )
        {
          bt_adapter->bb_rf_params.value = BB_WRITE;
          bt_adapter->bb_rf_params.no_of_values = 6;
          bt_adapter->bb_rf_params.soft_reset = 0;
          bt_adapter->bb_rf_params.Data[0] = 0x311;
          bt_adapter->bb_rf_params.Data[1] = 0x0;
          bt_adapter->bb_rf_params.Data[2] = 0x321;
          bt_adapter->bb_rf_params.Data[3] = 0x10;
          bt_adapter->bb_rf_params.Data[4] = 0x322;
          bt_adapter->bb_rf_params.Data[5] = 0x3;
          bt_adapter->os_intf_ops->onebox_memcpy(&data[0],&bt_adapter->bb_rf_params,sizeof(bt_adapter->bb_rf_params));
          netbuf_cb = bt_adapter->os_intf_ops->onebox_alloc_skb(len + 
              REQUIRED_HEADROOM_FOR_BT_HAL);
          if (!netbuf_cb) {
            rc = -ENOMEM;
            if (rc)
            {
              ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
                  (TEXT("%s: error(%d) occured \n"), __func__, rc)); 
              return rc;
            }
          }
          bt_adapter->os_intf_ops->onebox_reserve_data(netbuf_cb, 
              REQUIRED_HEADROOM_FOR_BT_HAL);
          bt_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len);
          bt_adapter->os_intf_ops->onebox_memcpy(netbuf_cb->data, data, len);
          netbuf_cb->bt_pkt_type = BT_PER; 
          rc = bt_adapter->osi_bt_ops->onebox_bt_xmit(bt_adapter, netbuf_cb);
          if (rc)
          {
            ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
                (TEXT("%s: error(%d) occured \n"), __func__, rc)); 
            return rc;
          }
          bt_adapter->os_intf_ops->onebox_reset_event(&(bt_adapter->bt_per_event));
#endif
          ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("CW_mode %d\n"),bt_adapter->cw_sub_type));
          switch (bt_adapter->cw_sub_type)
          {
            case 0:
            case 1:
              cw_mode_buf_write_array = cw_mode_buf_write_dc_tone;
              break;
            case 2:
              cw_mode_buf_write_array = cw_mode_buf_write_array_2;
              break;
            case 3:
              cw_mode_buf_write_array = cw_mode_buf_write_array_3;
              break;
            case 4:
              cw_mode_buf_write_array = cw_mode_buf_write_array_4;
              break;
            case 5:
              cw_mode_buf_write_array = cw_mode_buf_write_array_5;
              break;
            case 6:
              cw_mode_buf_write_array = cw_mode_buf_write_array_6;
              break;
            case 7:
              cw_mode_buf_write_array = cw_mode_buf_write_array_1;
              break;
            default:
              {
                cw_mode_buf_write_array = cw_mode_buf_write_array_1;
                break;
              }
          } /* end switch */
          i = 6;
          while (i< (258*3))
          {
            bt_adapter->wait_cmd = 0;
            memset (&bt_adapter->bb_rf_params, 0, sizeof (bb_rf_params_t));
            bt_adapter->bb_rf_params.Data[0] = 0x315;
            bt_adapter->bb_rf_params.Data[1] = 0x316;
            bt_adapter->bb_rf_params.Data[2] = 0x317;
            bt_adapter->bb_rf_params.value = 7; //BUFFER_WRITE
            bt_adapter->bb_rf_params.no_of_values = 34;
            bt_adapter->bb_rf_params.soft_reset = 3;
            data[0] = BUFFER_WRITE;
            data[1] = 0;
            data[2] = 34;
            data[3] = 3;
           // data[3] = 0x315;
           // data[4] = 0x316;
           // data[5] = 0x317;
            for (j =3; j< (35*3);)
            {
              if (i >= 258*3 )
                break;	
              //   data[j] = cw_mode_buf_write_array[i];
              //   data[j++] = cw_mode_buf_write_array[i++];
              //   data[j++] = cw_mode_buf_write_array[i++];
              bt_adapter->bb_rf_params.Data[j] = cw_mode_buf_write_array[i];
              bt_adapter->bb_rf_params.Data[j++] = cw_mode_buf_write_array[i++];
              bt_adapter->bb_rf_params.Data[j++] = cw_mode_buf_write_array[i++];
            }	
            bt_adapter->os_intf_ops->onebox_memcpy(&data[4],&bt_adapter->bb_rf_params.Data[0],(data[2] + 1)*2*3);
            netbuf_cb = bt_adapter->os_intf_ops->onebox_alloc_skb(len + 
                REQUIRED_HEADROOM_FOR_BT_HAL);
            if (!netbuf_cb) {
              rc = -ENOMEM;
              return rc;
            }
            bt_adapter->os_intf_ops->onebox_reserve_data(netbuf_cb, 
                REQUIRED_HEADROOM_FOR_BT_HAL);
            bt_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len);
            bt_adapter->os_intf_ops->onebox_memcpy(netbuf_cb->data, data, len);
            netbuf_cb->bt_pkt_type = BT_PER; 
            rc = bt_adapter->osi_bt_ops->onebox_bt_xmit(bt_adapter, netbuf_cb);
            if (rc) 
            {
              ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
                  (TEXT("%s: error(%d) occured \n"), __func__, rc)); 
              return rc;
            }
            bt_adapter->wait_cmd = 1;
            bt_adapter->os_intf_ops->onebox_reset_event(&(bt_adapter->bt_per_event));
            if(bt_adapter->os_intf_ops->onebox_wait_event(&(bt_adapter->bt_per_event), 3000)  == 0)
            {
              ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
                  (TEXT("%s:NO RESPONSE BEFORE SPECIFIED TIME \n"), __func__)); 
              return ONEBOX_STATUS_FAILURE;
            }
          }	
        }
      }
      break;

    default:
      break;
  }
  bt_adapter->wait_cmd = 0;
  return ONEBOX_STATUS_SUCCESS;
}
uint8 num_pkt_inx =0;
int32 bt_ber(BT_ADAPTER bt_adapter, uint8 *data, int32 length)
{
  bt_ber_params_t *param;
  netbuf_ctrl_block_t *netbuf_cb;
  uint8 cur_pkt_index = 0;
  uint16 num_of_pkts = 0;
  uint8 buffer[3];
  uint8 *pkt;
  uint8 k = 0;
  uint16 len;
  ONEBOX_STATUS status = ONEBOX_STATUS_FAILURE, rc = ONEBOX_STATUS_FAILURE;

  bt_adapter->os_intf_ops->onebox_memcpy(&bt_adapter->bb_rf_params, data, length);
  switch( bt_adapter->bb_rf_params.value)
  {
    case BT_BER_PKT_CNT:
      {
        bt_adapter->bt_ber_pkts = bt_adapter->bb_rf_params.Data[1];
        param = &bt_adapter->ber_info;		
        for(k = 0; k< bt_adapter->bt_ber_pkts ; k++)
        {
          param->ber_pkts[k].data = NULL;
        }	
        param->push_loc = 0;
        param->pop_loc = 0;
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("BER MODE ==> NO of packets : %d\n"),bt_adapter->bt_ber_pkts));
        num_pkt_inx = 0;
      }
      break;
    case BT_BER_RECEIVE:
      {
        param = &bt_adapter->ber_info;
        cur_pkt_index = param->pop_loc;
        num_of_pkts = param->num_pkts_avail;
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("num_of_packets in if = %d %d \n"), num_of_pkts, param->num_pkts_avail));
        if (num_of_pkts <= 0) {
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("====>>>>>>>EXIT_BECAUSE LESS NUM NO OF PKTS<<<<<<==== num_of_packets in if = %d\n"), num_of_pkts));
          return status;
        }
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d  after num of pkt checking and gng to get data\n"), __func__, __LINE__));
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Cur pkt indx = %d\n"),cur_pkt_index));
        pkt = param->ber_pkts[cur_pkt_index].data;
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("In %s Line %d pkt %p data %p\n"), __func__, __LINE__, pkt, param->ber_pkts[cur_pkt_index].data));
        if (!pkt) {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Packet is not available\n")));
          return status;
        }
        bt_adapter->os_intf_ops->onebox_memset(buffer, 0, 3);
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("param->length = %d\n"), param->ber_pkts[cur_pkt_index].len));
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("num_of_packets  = %d\n"), param->num_pkts_avail));
        *(uint8 *)&buffer[0] = (uint8)num_pkt_inx;
        *(uint16 *)&buffer[1] = (param->ber_pkts[cur_pkt_index].len + 3);
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("THE MODIFIED BUFFER LENGTH is %d \n"),*(uint16 *)&buffer[1]));
        num_pkt_inx++;

     //   bt_adapter->os_intf_ops->onebox_memcpy((buffer+3), pkt, param->ber_pkts[cur_pkt_index].len );

        bt_adapter->get_ber_pkts.len = ((param->ber_pkts[cur_pkt_index].len) + 3);
        if( bt_adapter->get_ber_pkts.len > 1035)
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("packet length greater than 1035\n")));
          return ONEBOX_STATUS_FAILURE;
        }
        bt_adapter->os_intf_ops->onebox_memcpy(&bt_adapter->get_ber_pkts.data[0], buffer, 3);
        bt_adapter->os_intf_ops->onebox_memcpy(&bt_adapter->get_ber_pkts.data[3], pkt, param->ber_pkts[cur_pkt_index].len);

        len = bt_adapter->get_ber_pkts.len + 2; /* 2 bytes to include the length parameter of ger_ber_pkts */

        bt_adapter->get_ber_pkts.num_pkts = param->num_pkts_avail;
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("location in = %d\n"), cur_pkt_index));

        param->ber_pkts[cur_pkt_index].data = NULL;
        param->ber_pkts[cur_pkt_index].len = 0;
        cur_pkt_index = ((cur_pkt_index + 1) % 256);

        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("location after = %d\n"), cur_pkt_index));

        param->pop_loc = cur_pkt_index;
        param->num_pkts_avail--;

        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("no of pkts available = %d\n"), param->num_pkts_avail));

        if (param->num_pkts_avail < 0)
          param->num_pkts_avail = 0;


        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Freeing pkt buffer\n")));
        bt_adapter->os_intf_ops->onebox_mem_free(pkt); 
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("Freed pkt buffer\n")));

        netbuf_cb = bt_adapter->os_intf_ops->onebox_alloc_skb(len +16+ 
            REQUIRED_HEADROOM_FOR_BT_HAL);
        if (!netbuf_cb) {
          rc = -ENOMEM;
        }

        bt_adapter->os_intf_ops->onebox_reserve_data(netbuf_cb, 
            REQUIRED_HEADROOM_FOR_BT_HAL);

        bt_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len +16);

        bt_adapter->os_intf_ops->onebox_memcpy(netbuf_cb->data + 16, &bt_adapter->get_ber_pkts, len + 16);

        bt_core_pkt_recv(bt_adapter, netbuf_cb);
        bt_adapter->os_intf_ops->onebox_free_pkt(netbuf_cb, 0);
      }
      break;
    default: ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Unknown Packet Type in BER\n")));
             break;
  }
  return ONEBOX_STATUS_SUCCESS;
}

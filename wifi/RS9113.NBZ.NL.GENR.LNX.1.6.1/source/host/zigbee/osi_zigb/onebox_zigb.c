#include "zb_common.h"
#include "onebox_zigbee_core.h"
#include "onebox_zb_per.h"

int32 core_zigb_init(ZB_ADAPTER zb_adapter)
{
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Entered core_bt_init in %s function\n"),__func__));
	
	zb_adapter->os_intf_ops->onebox_netbuf_queue_init(&zb_adapter->zigb_rx_queue);
	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Finished netbuf_queue_init in %s function\n"),__func__));

	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Before onebox_init_proc in %s function\n"),__func__));
	/********************************************************************************
	      			adding  for Netlink sockets
		  *********************************************************************/
	if (zb_adapter->zigb_osd_ops->onebox_zigb_register_genl(zb_adapter)) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
		    (TEXT("Finished Zigbeestack_init in %s function\n"), __func__));
		return ONEBOX_STATUS_FAILURE;
	}

	zb_adapter->core_init_done = 1;

	return ONEBOX_STATUS_SUCCESS;
}

int32 core_zigb_deinit(ZB_ADAPTER zb_adapter)
{
	zb_adapter->core_init_done = 0;
	zb_adapter->os_intf_ops->onebox_queue_purge(&zb_adapter->zigb_rx_queue);

	ONEBOX_DEBUG(ONEBOX_ZONE_INIT,(TEXT("%s: Uninitialized Procfs\n"), __func__));   
	/**********************************************************************
   				adding for Netlink sockets
  		*******************************************************************/
	if(zb_adapter->zigb_osd_ops->onebox_zigb_deregister_genl(zb_adapter)) {
		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Finished Zigbeestack_ deinit in %s function\n"),__func__));
		return ONEBOX_STATUS_FAILURE;
	}

	return 0;	
}

/**
 * This routine dump the given data through the debugger..
 *
 * @param  Debug zone.  
 * @param  Pointer to data that has to dump.  
 * @param  Length of the data to dump.  
 * @return VOID. 
 */
void onebox_print_dump(int32 zone,UCHAR *vdata, int32 len )
{
	uint16 ii;

	if(!zone || !(zone & onebox_zigb_zone_enabled))
		return;

	for(ii=0; ii< len; ii++)
	{
		if(!(ii % 16) && ii)
		{
			//ONEBOX_DEBUG(zone, (TEXT("\n%04d: "), ii));
			ONEBOX_DEBUG(zone, (TEXT("\n")));
		}
		ONEBOX_DEBUG(zone,(TEXT("%02x "),(vdata[ii])));
	}
	ONEBOX_DEBUG(zone, (TEXT("\n")));
}

int32 zigb_cw(ZB_ADAPTER zb_adapter, uint8 *data_org, int32 len)
{
	ONEBOX_STATUS rc;
	netbuf_ctrl_block_t *netbuf_cb;
	int i = 0;
	int j = 0;
	uint16 *cw_mode_buf_write_array;
	uint8* data;

	if(!data_org) {
			ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("%s: Line %d Data is NULL\n"), __func__, __LINE__));   
			return ONEBOX_STATUS_FAILURE;
	}
	data = data_org + 16; 
	zb_adapter->os_intf_ops->onebox_memcpy(&zb_adapter->bb_rf_params, data, len);
	zb_adapter->os_intf_ops->onebox_memset(data,0, len);

	zb_adapter->read_cmd = 0;
	zb_adapter->wait_cmd = 0;

	switch( zb_adapter->bb_rf_params.value)
	{
		case ZIGB_CW_MODE:
		{
			ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("%s: CW_MODE Request\n"), __func__));
			data[0] = ZIGB_CW_MODE;
			zb_adapter->cw_type = data[1] = zb_adapter->bb_rf_params.no_of_values; //cw_type
			zb_adapter->cw_sub_type = data[2] = zb_adapter->bb_rf_params.no_of_fields; //sub_type
			data[3] = zb_adapter->bb_rf_params.soft_reset;

			netbuf_cb = zb_adapter->os_intf_ops->onebox_alloc_skb(len + 16);
			if (!netbuf_cb) {
				rc = -ENOMEM;
				if (rc)
				{
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
										(TEXT("%s: error(%d) occured \n"), __func__, rc)); 
						return rc;
				}
			}
			zb_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len +16 );
			zb_adapter->os_intf_ops->onebox_memcpy(netbuf_cb->data, data_org, len + 16);
			netbuf_cb->zigb_pkt_type = ZIGB_PER; 
			zb_adapter->wait_cmd = 1;
			zb_adapter->os_intf_ops->onebox_reset_event(&(zb_adapter->zb_per_event));
			rc = zb_adapter->osi_zigb_ops->onebox_send_pkt(zb_adapter, netbuf_cb);
			if (rc)
			{
				ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
								(TEXT("%s: error(%d) occured \n"), __func__, rc)); 
				return rc;
			}
			if( !zb_adapter->cnfm_flag )
			{
				if((zb_adapter->os_intf_ops->onebox_wait_event(&(zb_adapter->zb_per_event), 10000)  == 0))
				{
						ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
										(TEXT("%s:NO RESPONSE BEFORE SPECIFIED TIME \n"), __func__)); 
						return ONEBOX_STATUS_FAILURE;
				}
			}
			zb_adapter->cnfm_flag = 0;

			if( !zb_adapter->cw_type )
			{
				zb_adapter->bb_rf_params.value = BB_WRITE;
				zb_adapter->bb_rf_params.no_of_values = 6;
				zb_adapter->bb_rf_params.soft_reset = 0;
				zb_adapter->bb_rf_params.Data[0] = 0x311;
				zb_adapter->bb_rf_params.Data[1] = 0x0;
				zb_adapter->bb_rf_params.Data[2] = 0x321;
				zb_adapter->bb_rf_params.Data[3] = 0x10;
				zb_adapter->bb_rf_params.Data[4] = 0x322;
				zb_adapter->bb_rf_params.Data[5] = 0x3;
				zb_adapter->os_intf_ops->onebox_memcpy(&data[0],&zb_adapter->bb_rf_params,sizeof(zb_adapter->bb_rf_params));
				netbuf_cb = zb_adapter->os_intf_ops->onebox_alloc_skb(len + 16 );
				if (!netbuf_cb) {
					rc = -ENOMEM;
					if (rc)
					{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
											(TEXT("%s: error(%d) occured \n"), __func__, rc)); 
							return rc;
					}
				}
				zb_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len +16 );
				zb_adapter->os_intf_ops->onebox_memcpy(netbuf_cb->data, data_org, len + 16 );
				netbuf_cb->bt_pkt_type = ZIGB_PER; 
				rc = zb_adapter->osi_zigb_ops->onebox_send_pkt(zb_adapter, netbuf_cb);
				if (rc) {
					ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
									(TEXT("%s: error(%d) occured \n"), __func__, rc)); 
					return rc;
				}

				switch (zb_adapter->cw_sub_type)
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
					zb_adapter->wait_cmd = 0;
					memset (&zb_adapter->bb_rf_params, 0, sizeof (bb_rf_params_t));
					zb_adapter->bb_rf_params.Data[0] = 0x315;
					zb_adapter->bb_rf_params.Data[1] = 0x316;
					zb_adapter->bb_rf_params.Data[2] = 0x317;
					zb_adapter->bb_rf_params.value = 7; //BUFFER_WRITE
					zb_adapter->bb_rf_params.no_of_values = 34;
					zb_adapter->bb_rf_params.soft_reset = 3;
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
							zb_adapter->bb_rf_params.Data[j] = cw_mode_buf_write_array[i];
							zb_adapter->bb_rf_params.Data[j++] = cw_mode_buf_write_array[i++];
							zb_adapter->bb_rf_params.Data[j++] = cw_mode_buf_write_array[i++];
					}	
					zb_adapter->os_intf_ops->onebox_memcpy(&data[4],&zb_adapter->bb_rf_params.Data[0],(data[2] + 1)*2*3);
					netbuf_cb = zb_adapter->os_intf_ops->onebox_alloc_skb(len + 16); 
					if (!netbuf_cb) {
							rc = -ENOMEM;
							return rc;
					}
					//adapter->os_intf_ops->onebox_reserve_data(netbuf_cb, 
					//  REQUIRED_HEADROOM_FOR_BT_HAL);
					zb_adapter->os_intf_ops->onebox_add_data_to_skb(netbuf_cb, len + 16);
					zb_adapter->os_intf_ops->onebox_memcpy(netbuf_cb->data, data_org, len + 16);
					netbuf_cb->zigb_pkt_type = ZIGB_PER; 
					zb_adapter->os_intf_ops->onebox_reset_event(&(zb_adapter->zb_per_event));
					zb_adapter->wait_cmd = 1;
					rc = zb_adapter->osi_zigb_ops->onebox_send_pkt(zb_adapter, netbuf_cb);
					if (rc) 
					{
							ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
											(TEXT("%s: error(%d) occured \n"), __func__, rc)); 
							return rc;
					}
					if( !zb_adapter->cnfm_flag )
					{
							if((zb_adapter->os_intf_ops->onebox_wait_event(&(zb_adapter->zb_per_event), 10000)  == 0))
							{
									ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,
													(TEXT("%s:NO RESPONSE BEFORE SPECIFIED TIME \n"), __func__)); 
									return ONEBOX_STATUS_FAILURE;
							}
					}
					zb_adapter->cnfm_flag = 0;
				}	
			}
		}
		break;

		default:
				break;
	}
	zb_adapter->wait_cmd = 0;
	return ONEBOX_STATUS_SUCCESS;
}


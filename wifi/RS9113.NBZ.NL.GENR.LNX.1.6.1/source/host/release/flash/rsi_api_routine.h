

typedef struct _test_params
{
	int vendor_id;
	int start_mac_id;
	int end_mac_id; 
	int bt_start_mac_id;
	int bt_end_mac_id; 
	int zigbee_start_mac_id;
	int zigbee_end_mac_id; 
	int wlan_mac_id;
	int bt_mac_id;
	int zigbee_mac_id;
	int flash_size;
	int num_wlan_macs;
	unsigned char server_mac[16];
}test_params_t;

#define RSI_STATUS_SUCCESS 0
#define RSI_STATUS_FAILURE -1

#if 0
void rsi_read_config(void );
void rsi_save_mac(void );
void rsi_get_token(FILE *info_file,const char *check_str,char *res);
#endif

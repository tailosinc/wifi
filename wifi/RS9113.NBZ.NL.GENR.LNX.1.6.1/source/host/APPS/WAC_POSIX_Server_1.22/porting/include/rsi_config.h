/**
 * @file           rsi_config.h
 * @version        3.6
 * @date           2013-May-16
 *
 * Copyright(C) Redpine Signals 2013
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief CONFIG INIT contains the default configurations used in the api's
 *
 * @section Description
 * USER is supposed to configure the module/API's  by using the following defines 
 *
 *
 */

#ifndef _INITSTRUCT_H_
#define _INITSTRUCT_H_


#include "rsi_api.h"
/*==============================================*/
/**
 * Global Defines
 */



#define MFI_DEVICE_TO_CONFIGURE      "MFI"

//! OPerating band 2.4GHZ or 5GHZ 
#define RSI_BAND                   RSI_BAND_2P4GHZ             //@ RSI_BAND_2P4GHZ or RSI_BAND_5GHZ or RSI_DUAL_BAND

#define RSI_FEATURE_BIT_MAP        FEAT_SECURITY_OPEN          //@ To set wlan feature select bit map
#define RSI_TCP_IP_FEATURE_BIT_MAP TCP_IP_FEAT_BYPASS
#define RSI_CUSTOM_FEATURE_BIT_MAP  0                          //@ To set custom feature select bit map 

#define RSI_SECURITY_MODE          SECURITY_MODE_OPEN          //@ For selecting different security mode APs.
#define RSI_PSK_SUPPORT            DISABLE                     //@ ENABLE or DISABLE PSK support
#define RSI_PSK                    ""                          //@ PSK.If we are using WPA2, this is the key, In open mode NULL
#define RSI_PSK_TYPE               RSI_PSK_FROM_HOST           //@ RSI_PSK_FROM_HOST or RSI_PMK_FROM_HOST or RSI_GENERATE_PMK

//! SCAN parameters 
#define RSI_SCAN_CHANNEL                0                      //@ scan channel number.0 to scans all channels
#define RSI_SCAN_FEAT_BITMAP            0                      //@ scan_feature_bitmap ,valid only if specific channel and ssid are given
#define RSI_SCAN_CHANNEL_BIT_MAP_2_4    0                      //@ scan channle bit map,valid if RSI_SCAN_CHANNEL is 0
#define RSI_SCAN_CHANNEL_BIT_MAP_5      0                      //@ scan channle bit map,valid if RSI_SCAN_CHANNEL is 0


#define SEND_PSK_IN_JOIN           1                           //@ 1-to send PSK in join frame, 0 - not to send PSK in join
#define RSI_JOIN_FEAT_BIT_MAP	     0						               //@ RSI_JOIN_FEAT_STA_BG_ONLY_MODE_ENABLE or RSI_JOIN_FEAT_LISTEN_INTERVAL_VALID
#define RSI_LISTEN_INTERVAL		     0
#define RSI_DATA_RATE              RSI_DATA_RATE_AUTO          //@ RSI_DATA_RATE_AUTO or RSI_DATA_RATE_(1, 2, 5P5, 11, 6, 9, 12)
#define RSI_POWER_LEVEL            RSI_POWER_LEVEL_HIGH        //@ RSI_POWER_LEVEL_LOW or RSI_POWER_LEVEL_MEDIUM or RSI_POWER_LEVEL_HIGH




//!  AP mode Configurations 
#define RSI_AP_SSID                    "REDPINE_AP"            //@ SSID to Accesspoint
#define RSI_DTIM_PERIOD                4                       //@ AP DTIM Period to configure 
#define RSI_BEACON_INTERVAL            100                     //@ AP beacon interval
#define RSI_AP_CHANNEL_NUM             1                       //@ AP channel number
#define RSI_SECURITY_TYPE              RSI_SECURITY_NONE       //@ AP security type
#define RSI_ENCRYPTION_MODE            RSI_ENCRYPTION_NONE     //@ AP Encryption type 
#define MAX_NO_OF_CLIENTS              4                       //@ AP:Maximum number of clients can connect.Possible values are 1 to 4
#define AP_KEEPALIVE_TYPE              0                       //@ AP:Keep alive type 0 -disable keep alive ,  1 - enables deauth based keep alive ,3 - enables null data based keep alive
#define AP_KEEPALIVE_PERIOD            100                     //@ AP:Keep alive period (KEEP_ALIVE_PERIOD*32*Beacon_interval time duartion AP waits for packet form connected stations)



#define   SECURITY_MODE_OPEN      0                            //@ Open mode or Dont care
#define   SECURITY_MODE_WPA       1                            //@ Selects only WPA mode AP
#define   SECURITY_MODE_WPA2      2                            //@ Selects only WPA2 mode AP
#define   SECURITY_MODE_WEP       3                            //@ For WEP security mode
#define   SECURITY_MODE_WPA_EAP   4                            //@ For EAP-WPA
#define   SECURITY_MODE_WPA2_EAP  5                            //@ For EAP-WPA2
#define   SECURITY_MODE_MIXED     6                            //@ For Mixed Mode AP
#endif

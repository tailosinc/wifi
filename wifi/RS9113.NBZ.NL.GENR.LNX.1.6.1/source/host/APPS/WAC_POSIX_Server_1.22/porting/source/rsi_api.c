/**
 * @file     rsi_api.c
 * @version  3.6
 * @date     2013-May-10
 *
 * Copyright(C) Redpine Signals 2012
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief API: Definitions of various data structures and variables
 * 
 * @section Description
 * This file contain definition of different mangament, control & data commands variables.
 * These definition are used to construct frames. 
 *
 * @section Improvements
 * New command frames are added.
 *
 */


/**
 * Includes
 */
#include "rsi_global.h"
#include "rsi_api.h"


/**
 * Global Variables
 */

/* In the 3 bytes,  
 * BYTE0[0:7],BYTE1[0:3] together represents length of the payload for the frame command
 * BYTE1[4:6], indicates queue number 100 - Management,101 - Data
 * BYTE1[7], Reserved
 * BYTE2 represents the Frame command type. 
 */

/*Management Commands */
/* Debug prints enable */
const uint8   rsi_frameCmdDebug[RSI_BYTES_3] = {0x08,  0x40, 0x26};
/* Bootup_cmds */
const uint8   rsi_frameCmdBootInsn[RSI_BYTES_3] = {0x00,  0x00, 0xFF};
/* Update Info cmd/message */
const uint8   rsi_frameCmdUpdateInfo[RSI_BYTES_3] = {0x00,  0x00, 0xE1};
/* set Operating mode frame */ 
const uint8   rsi_frameCmdOperMode[RSI_BYTES_3] = {0x10,  0x40, 0x10};
/*Band frame*/
const uint8   rsi_frameCmdBand[RSI_BYTES_3] = {0x01,  0x40, 0x11};
/*Init frame*/
const uint8   rsi_frameCmdInit[RSI_BYTES_3] = {0x00, 0x40, 0x12};
/* Scan frame */
const uint8   rsi_frameCmdScan[RSI_BYTES_3] = {0x32, 0x40, 0x13 };    
/* Scan frame */
const uint8   rsi_frameCmdBGScan[RSI_BYTES_3] = {0x0F, 0x40, 0x6a };    
/* Join Frame */
const uint8   rsi_frameCmdJoin[RSI_BYTES_3] = {0x6F, 0x40, 0x14 };


/* PSK mode frame */
const uint8   rsi_frameCmdPsk[RSI_BYTES_3] = {0x63, 0x40, 0xa5 };

/* Power mode frame */
const uint8   rsi_frameCmdPower[RSI_BYTES_3] = {0x03, 0x40, 0x15 };    
/* Set Mac Address frame */
const uint8   rsi_frameCmdSetMacAddr[RSI_BYTES_3] = {0x06, 0x40, 0x17};    
/* Query Network Parameters frame */
const uint8   rsi_frameCmdQryNetParms[RSI_BYTES_3] = {0x00, 0x40, 0x18};    
/* Disconnect frame */
const uint8   rsi_frameCmdDisconnect[RSI_BYTES_3] = {0x08, 0x40, 0x19};    
/* RSSI frame */
const uint8   rsi_frameCmdRssi[RSI_BYTES_3] = {0x00, 0x40, 0x3A}; 
/* Antenna selection Frame */
const uint8   rsi_frameCmdAntSel[RSI_BYTES_3] = {0x03, 0x40, 0x1B};
/* Sleep timer Frame */
const uint8   rsi_frameCmdSleepTimer[RSI_BYTES_3] = {0x02, 0x40, 0x16};   
/* AP configuration frame */
const uint8   rsi_frameCmdAPconf[RSI_BYTES_3] = {0x6E, 0x40, 0x24};
/* WEP Key configuration frame*/
const uint8   rsi_frameCmdWepkey[RSI_BYTES_3] = {0x82, 0x40, 0x25};
/* Reset the module */
const uint8   rsi_frameCmdReset[RSI_BYTES_3] = {0x00, 0x40, 0x1C};
/* Ping Request */
const uint8   rsi_frameCmdPingRequest[RSI_BYTES_3]={0x14, 0x40, 0x29};

/*Gpio configuation request*/
const uint8   rsi_frameCmdGpioconf[RSI_BYTES_3]={0x04, 0x40, 0x28};

/* IP parameters frame */
const uint8   rsi_frameCmdIpparam[RSI_BYTES_3] = {0x2C, 0x40, 0x41};    
/* Socket creation frame */
const uint8   rsi_frameCmdSocket[RSI_BYTES_3] = {0x89, 0x40, 0x42};    
/* Socket close frame */
const uint8   rsi_frameCmdSocketClose[RSI_BYTES_3] = {0x04, 0x40, 0x43};    
/* Connection status query frame */
const uint8   rsi_frameCmdConnStatus[RSI_BYTES_3] = {0x00, 0x40, 0x48};    
/* Firmware version query frame */
const uint8   rsi_frameCmdQryFwVer[RSI_BYTES_3] = {0x00, 0x40, 0x49};    
/*Query mac adress frame */ 
const uint8   rsi_frameCmdQryMacAddress[RSI_BYTES_3]= {0x00 , 0x40, 0x4A};
/* P2p Config frame */
const uint8   rsi_frameCmdP2pConfig[RSI_BYTES_3] = {0xC4, 0x40, 0x4B};
/* Set eap parameters frame */
const uint8   rsi_frameCmdSetEap[RSI_BYTES_3]    = {0x04, 0x41, 0x4C};
/* Certificate loading frame */
const uint8   rsi_frameCmdCert[RSI_BYTES_3] = {0x00, 0x40, 0x4D};
/*Query GO parametes frame */
const uint8   rsi_frameCmdQryGoParms[RSI_BYTES_3] = {0x00, 0x40, 0x4E};
/* set Web  fields frame */
const uint8   rsi_frameCmdWebFields[RSI_BYTES_3] = {0xA8, 0x42, 0x4F};
/* DNS server */
const uint8   rsi_frameCmdDnsServer[RSI_BYTES_3] = {0X24, 0x40, 0X55};
/* DNS query */
const uint8   rsi_frameCmdDnsQuery[RSI_BYTES_3] = {0x5E , 0x40 , 0x44 }; 
/* Cfg save frame */
const uint8   rsi_frameCmdCfgSave[RSI_BYTES_3] = {0x00, 0x40, 0x20};
/*Cfg enable frame*/
const uint8   rsi_frameCmdCfgEnable[RSI_BYTES_3] = {0x01,  0x40, 0x21};
/* Cfg get frame */
const uint8   rsi_frameCmdCfgGet[RSI_BYTES_3] = {0x00, 0x40, 0x22};
/* Feature select */
const uint8   rsi_frameCmdFeatsel[RSI_BYTES_3] = {0x04,  0x40, 0x1E};
/* SNR query */
const uint8   rsi_frameCmdSnr[RSI_BYTES_3] = {0x00, 0x40, 0x1F};
/* LTCP Connection status query*/
const uint8   rsi_frameCmdLtcpConnStatus[RSI_BYTES_3] = {0x2, 0x40, 0x46};
/* Sent bytes count query*/
const uint8   rsi_frameCmdSentBytesCount[RSI_BYTES_3] = {0x2, 0x40, 0xB2};
/* URL response Frame */
const uint8   rsi_frameCmdUrlRsp[RSI_BYTES_3] = {0x05, 0x44, 0x56};
/* Multicast filter response Frame */
const uint8   rsi_frameCmdMcastFilter[RSI_BYTES_3] = {0x02, 0x40, 0x40};

/*Wireless firmware upgrade frame */
const uint8   rsi_frameCmdWirelessFwUpgrade[RSI_BYTES_3] = {0x00, 0x40, 0x59};

/*SNMP enable Response frame*/
const uint8   rsi_frameCmdSnmpEnable[RSI_BYTES_3] = {0x01, 0x40, 0x85};
/*SNMP get response Response frame*/
const uint8   rsi_frameCmdSnmpGetRsp[RSI_BYTES_3] = {0xCC, 0x40, 0x83};
/*SNMP get next response Response frame*/
const uint8   rsi_frameCmdSnmpGetNextRsp[RSI_BYTES_3] = {0xCC, 0x40, 0x84};
/* IPCONFIG6 for IPV6 */
const uint8   rsi_frameCmdIPconf6[RSI_BYTES_3] = {0x24, 0x40, 0x90};
/* Rejoin Params response Frame */
const uint8   rsi_frameCmdRejoinParams[RSI_BYTES_3] = {0x10, 0x40, 0x6F};

/* Data Commands */
const uint8   rsi_frameCmdSend[RSI_BYTES_3] = {0x00, 0x50, 0x00};     
const uint8   rsi_frameCmdRecv[RSI_BYTES_3] = {0x00, 0x50, 0x00};    
/* Length comes from the received frame descriptor */

/* Roaming parameter frame command */
const uint8   rsi_frameCmdRoamParams[RSI_BYTES_3] = {0x0C, 0x40, 0x7B};

//HT CAPS parameters frame 
const uint8   rsi_frameCmdHtCaps[RSI_BYTES_3] = {0x04, 0x40, 0x6D};

//WMM PS parameters frame 
const uint8   rsi_frameCmdWmmPs[RSI_BYTES_3] = {0x09, 0x40, 0x97};

//WPS method parameters frame 
const uint8   rsi_frameCmdWpsMethod[RSI_BYTES_3] = {0x0C, 0x40, 0x72};

//Multicast method parameters frame 
const uint8   rsi_frameCmdMulticast[RSI_BYTES_3] = {0x14, 0x40, 0xB1};

// PER MODE
const uint8   rsi_frameCmdPerMode[RSI_BYTES_3] = {0x1c, 0x40, 0x7c};

// PER STATS 
const uint8   rsi_frameCmdPerStats[RSI_BYTES_3] = {0x4, 0x40, 0xA2};
//! UART flow control Enable
const uint8   rsi_frameCmdUartFlowControl[RSI_BYTES_3] = {0x01, 0x40, 0xA4};

/* Power save sleep ack */
const uint8   rsi_sleepack[RSI_BYTES_3] = {0x00, 0x40, 0xDE}; 

/* Management Response list */
const uint8   rsi_mgmtRspList[] = {0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x64}; 
/* 9 bytes, list of management frame response codes */

// JSON & Webpage related
const uint8  rsi_frameCmdWebpageClearFiles[RSI_BYTES_3] = { 0x01, 0x40, 0x7F};
const uint8  rsi_frameCmdWebpageEraseFile[RSI_BYTES_3]  = { 0x18, 0x40, 0x9A};
const uint8  rsi_frameCmdJSONCreate[RSI_BYTES_3]        = { 0x1C, 0x44, 0x9C};
const uint8  rsi_frameCmdJSONEraseFile[RSI_BYTES_3]     = { 0x18, 0x40, 0x9B};

/*Set region frame*/
const uint8   rsi_frameSetRegion[RSI_BYTES_3]           = {0x02, 0x40, 0x1D};

const uint8	  rsi_frameCmdUserStoreCfg[RSI_BYTES_3]     = {0x6b, 0x44, 0x23};

/*Set Region frame for AP mode*/
const uint8   rsi_frameSetRegionAp[RSI_BYTES_3] = {0x50, 0x40, 0xBD};

/* RF Current Mode Configuration */
const uint8   rsi_frameCmdRFCurConfig[RSI_BYTES_3] = {0x04, 0x40, 0xAD};

/* Socket configuration */
const uint8   rsi_frameSocketConfig[RSI_BYTES_3] = {0x8, 0x40, 0xA7};

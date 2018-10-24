#pragma once
//
//  Configuration.h
//  Workbench
//
//  Created by Wai Man Chan on 10/27/14.
//
//

#define HomeKitLog 1
#define HomeKitReplyHeaderLog 1
#define PowerOnTest 0
#define IAP_CHIP 1
#if IAP_CHIP == 1
#define kMFiSAP_AESKeyLen           16
#define kMFiSAP_ECDHKeyLen          32
#define kMFiSAP_VersionLen          1
#endif

#define FAN_ATTRIBUTE         1
#define GDO_ATTRIBUTE         1
#define LOCKMECH_ATTRIBUTE    1
#define SWITCH_ATTRIBUTE      1
#define WINDOW_ATTRIBUTE      0
#define WINDOWCVR_ATTRIBUTE   0
#define LOCKMGMT_ATTRIBUTE    0
#define OUTLET_ATTRIBUTE      0
#define THERMOSTAT_ATTRIBUTE  0

//Device Setting
#define deviceName "House Light"    //Name
#define deviceIdentity "12:10:34:23:51:12"  //ID
#define _manufactuerName "ET Chan"   //Manufactuer
#define devicePassword "523-12-643" //Password
#define deviceUUID "62F47751-8F26-46BF-9552-8F4238E67D60"   //UUID, for pair verify
#define controllerRecordsAddress "/var/PHK_controller" //Where to store the client keys

//Number of client
/*
 * BEWARE: Never set the number of client to 1
 * iOS HomeKit pair setup socket will not release until the pair verify stage start
 * So you will never got the pair corrected, as it is incomplete (The error require manually reset HomeKit setting
 */
#define numberOfClient 20
//Number of notifiable value
/*
 * Count how many notifiable value exist in your set
 * For dynamic add/drop model, please estimate the maximum number (Too few->Buffer overflow)
 */
#define numberOfNotifiableValue 1

//If you compiling this to microcontroller, set it to 1
#define MCU 0

#include <openssl/sha.h>
#include <stdint.h>
#include <unistd.h>

typedef SHA512_CTX SHACTX;
#define SHAInit SHA512_Init
#define SHAUpdate SHA512_Update
#define SHAFinal SHA512_Final
#define SHA_DIGESTSIZE 64
#define SHA_BlockSize 128
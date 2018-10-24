#ifndef _RSI_FRAME_H_
#define _RSI_FRAME_H_

#include "rsi_common_types.h"
#include <stdlib.h>
#define RSI_ZIGB_FRAME_DESC_LEN         16

#define rsi_malloc(ptr)  malloc(ptr)
#define rsi_free(ptr)    free(ptr)

#define g_BASIC_CLUSTER_c                             0x0000
#define g_IDENTIFY_CLUSTER_c                          0x0003
#define g_ON_OFF_SWITCH_CONFIGURATION_CLUSTER_c       0x0007

//out Cluster id defines. Remove these once profile porting completes.
#define g_ON_OFF_CLUSTER_c        0x0006

enum
{
  g_OFF_c,
  g_ON_c,
  g_TOGGLE_c
};

typedef struct lightDeviceInfo_tag{

  uint16_t shortaddress;
  uint8_t endpoint;

}lightDeviceInfo_t;

typedef enum FrameControl_Direction
{
  g_Client_To_Server_c = 0x00,
  g_Server_To_Client_c = 0x08

}Frame_Control_direction;

/*=========================================================================*
 * Frame Descriptor
 *=========================================================================*/

//typedef struct {
//    uint8_t   uFrmDscBuf[RSI_ZIGB_FRAME_DESC_LEN];       //@ byte format for spi interface, 16 bytes
//} rsi_zigb_uFrameDsc;

#endif

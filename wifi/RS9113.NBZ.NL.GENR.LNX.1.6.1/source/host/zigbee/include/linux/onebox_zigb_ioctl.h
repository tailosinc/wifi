/**
 * @file onebox_zb_ioctl.h
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
 * This file contians the ioctl related prototypes and macros
 *  
 */

#ifndef __ONEBOX_ZB_IOCTL_H__
#define __ONEBOX_ZB_IOCTL_H__

#define ONEBOX_ZIGB_SEND        SIOCIWLASTPRIV - 0x1  
#define ONEBOX_ZIGB_RECV        SIOCIWLASTPRIV - 0x2  
int zigb_ioctl(struct net_device *dev,struct ifreq *ifr, int cmd);
#endif

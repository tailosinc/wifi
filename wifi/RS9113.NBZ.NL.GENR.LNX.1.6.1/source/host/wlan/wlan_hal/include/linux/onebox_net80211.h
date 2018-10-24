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

#define HT_SUPPORT
#include "ieee80211_linux.h"
#include <net80211/_ieee80211.h>
#include <net80211/ieee80211.h>
#include <net80211/ieee80211_ageq.h>
#include <net80211/ieee80211_crypto.h>
#include <net80211/ieee80211_dfs.h>
#include <net80211/ieee80211_ioctl.h>        /* for ieee80211_stats */
#include <net80211/ieee80211_phy.h>
#include <net80211/ieee80211_power.h>
#include <net80211/ieee80211_node.h>
#include <net80211/ieee80211_proto.h>
#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_scan.h>
#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_input.h>
#include <net80211/ieee80211_regdomain.h>

/*#define HT_SUPPORT
//#include <net/route.h>
//#include <net/if_var.h>
#include "../../net80211/ieee80211_freebsd.h"
#include "../../net80211/_ieee80211.h"
#include "../../net80211/ieee80211.h"
#include "../../net80211/ieee80211_action.h"
#include "../../net80211/ieee80211_adhoc.h"
//#include "ieee80211_ageq.h"
#include "../../net80211/ieee80211_amrr.h"
#include "../../net80211/ieee80211_crypto.h"
//#include "ieee80211_dfs.h"
#include "../../net80211/ieee80211_hostap.h"
//#include "ieee80211_ht.h"
#include "../../net80211/ieee80211_ioctl.h"
#include "../../net80211/ieee80211_mesh.h"
#include "../../net80211/ieee80211_power.h"
#include "../../net80211/ieee80211_node.h"
#include "../../net80211/ieee80211_phy.h"
#include "../../net80211/ieee80211_proto.h"
#include "../../net80211/ieee80211_radiotap.h"
#include "../../net80211/ieee80211_var.h"
//#include "ieee80211_ratectl.h"
//#include "ieee80211_regdomain.h"
#include "../../net80211/ieee80211_rssadapt.h"
#include "../../net80211/ieee80211_scan.h"
#include "../../net80211/ieee80211_sta.h"
#include "../../net80211/ieee80211_superg.h"
#include "../../net80211/ieee80211_tdma.h"
#include "../../net80211/ieee80211_wds.h"
#include "../../net80211/ieee80211_input.h"*/

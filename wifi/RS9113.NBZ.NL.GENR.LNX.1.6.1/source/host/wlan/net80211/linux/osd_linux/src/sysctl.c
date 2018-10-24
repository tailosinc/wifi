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

#include <net80211/ieee80211_var.h>
#include "sysctl.h"

/* All these variables are exported to user via sysctl */
extern int ieee80211_nol_timeout;
extern int ieee80211_cac_timeout;
extern int ieee80211_recv_bar_ena;
#ifdef IEEE80211_AMPDU_AGE
extern int ieee80211_ampdu_age;
#endif
extern int ieee80211_addba_timeout;
extern int ieee80211_addba_backoff;
extern int ieee80211_addba_maxtries;
extern int ieee80211_hwmp_targetonly;
extern int ieee80211_hwmp_replyforward;
extern int ieee80211_hwmp_pathtimeout;
extern int ieee80211_hwmp_roottimeout;
extern int ieee80211_hwmp_rootint;
extern int ieee80211_hwmp_rannint;
extern int ieee80211_mesh_retrytimeout;
extern int ieee80211_mesh_holdingtimeout;
extern int ieee80211_mesh_confirmtimeout;
extern int ieee80211_mesh_maxretries;
extern int ieee80211_ffppsmin;	
extern int ieee80211_ffagemax;
int ieee80211_debug = 0;

/**
 * This function provides control to read/modify the debug level 
 * variable
 *
 * @param 
 *  SYSCTL_HANDLER_ARGS Standard arguments of the Linux sysctl handler
 *
 * @returns 
 *  0 on success, or corresponding negative error code on failure
 */

static int
ieee80211_sysctl_debug(SYSCTL_HANDLER_ARGS)
{
	struct ieee80211vap *vap = ctl->extra1;
	unsigned int  val; 
	int ret; 
	
	ctl->data = &val;
	ctl->maxlen = sizeof(val);
	if (write)
	{
	/* Write into the field */
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		        lenp, ppos);
		if (ret == 0)
		{
			vap->iv_debug = val; 
		}
	} 
	else 
	{
	/* Read from the field */
		val = vap->iv_debug;
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
	}
	return ret; 
}

/**
 * This function provides control to read/write driver caps field
 *
 * @param 
 *  SYSCTL_HANDLER_ARGS Standard arguments of the Linux sysctl handler
 *
 * @returns 
 *  0 on success, or corresponding negative error code on failure
 */
static int
ieee80211_sysctl_driver_caps(SYSCTL_HANDLER_ARGS)
{
	struct ieee80211vap *vap = ctl->extra1;
	unsigned int  val; 
	int  ret; 
	
	ctl->data = &val;
	ctl->maxlen = sizeof(val);
	if (write) 
	{
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
		if (ret == 0)
		{
			vap->iv_caps = val; 
		}
	}
	else
	{
		val = vap->iv_caps;
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
	}
	return ret; 
}

/**
 * This function provides control to read/write bmiss_max variable,
 * which is the count of consecutive beacons missed before scanning.
 *
 * @param 
 *  SYSCTL_HANDLER_ARGS Standard arguments of the Linux sysctl handler
 *
 * @returns 
 *  0 on success, or corresponding negative error code on failure
 */
static int
ieee80211_sysctl_bmiss_max(SYSCTL_HANDLER_ARGS)
{
	struct ieee80211vap *vap = ctl->extra1;
	unsigned int  val; 
	int ret; 
	
	ctl->data = &val;
	ctl->maxlen = sizeof(val);
	if (write) 
	{
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
		if (ret == 0)
		{
			vap->iv_bmiss_max = val; 
		}
	}
	else
	{
		val = vap->iv_bmiss_max;
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
	}
	return ret; 
}

/**
 * This function provides control to read/write station inactivity 
 * timeout field.
 *
 * @param 
 *  SYSCTL_HANDLER_ARGS Standard arguments of the Linux sysctl handler
 *
 * @returns 
 *  0 on success, or corresponding negative error code on failure
 */
static int
ieee80211_sysctl_inact_run(SYSCTL_HANDLER_ARGS)
{
	struct ieee80211vap *vap = ctl->extra1;
	unsigned int  val; 
	int ret; 
	
	ctl->data = &val;
	ctl->maxlen = sizeof(val);
	if (write) 
	{
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
		if (ret == 0)
		{
			vap->iv_inact_run = val; 
		}
	}
	else
	{
		val = vap->iv_inact_run;
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
	}
	return ret; 
}

/**
 * This function provides control to read/write station inactivity 
 * probe timeout field.
 *
 * @param 
 *  SYSCTL_HANDLER_ARGS Standard arguments of the Linux sysctl handler
 *
 * @returns 
 *  0 on success, or corresponding negative error code on failure
 */
static int
ieee80211_sysctl_inact_probe(SYSCTL_HANDLER_ARGS)
{
	struct ieee80211vap *vap = ctl->extra1;
	unsigned int val; 
	int ret; 
	
	ctl->data = &val;
	ctl->maxlen = sizeof(val);
	if (write)
	{
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
		if (ret == 0)
		{
			vap->iv_inact_probe = val; 
		}
	} 
	else 
	{
		val = vap->iv_inact_probe;
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
	}
	return ret; 
}

/**
 * This function provides control to read/write station inactivity
 * field
 *
 * @param 
 *  SYSCTL_HANDLER_ARGS Standard arguments of the Linux sysctl handler
 *
 * @returns 
 *  0 on success, or corresponding negative error code on failure
 */
static int
ieee80211_sysctl_inact_auth(SYSCTL_HANDLER_ARGS)
{
	struct ieee80211vap *vap = ctl->extra1;
	unsigned int  val; 
	int ret; 
	
	ctl->data = &val;
	ctl->maxlen = sizeof(val);
	if (write) 
	{
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
		if (ret == 0)
		{
			vap->iv_inact_auth = val; 
		}
	}
	else 
	{
		val = vap->iv_inact_auth;
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
	}
	return ret; 
}

/**
 * This function provides control to read/write station initial state
 * timeout field
 *
 * @param 
 *  SYSCTL_HANDLER_ARGS Standard arguments of the Linux sysctl handler
 *
 * @returns 
 *  0 on success, or corresponding negative error code on failure
 */
static int
ieee80211_sysctl_inact_init(SYSCTL_HANDLER_ARGS)
{
	struct ieee80211vap *vap = ctl->extra1;
	unsigned int val; 
	int ret; 
	
	ctl->data = &val;
	ctl->maxlen = sizeof(val);
	if (write)
	{
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
		if (ret == 0)
		{
			vap->iv_inact_init = val; 
		}
	} 
	else
	{
		val = vap->iv_inact_init;
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
	}
	return ret; 
}

/**
 * This function provides control to read/write bk traffic tx aggr threshold
 * field
 *
 * @param 
 *  SYSCTL_HANDLER_ARGS Standard arguments of the Linux sysctl handler
 *
 * @returns 
 *  0 on success, or corresponding negative error code on failure
 */
static int
ieee80211_sysctl_ampdu_traffic_bk(SYSCTL_HANDLER_ARGS)
{
	struct ieee80211vap *vap = ctl->extra1;
	unsigned int val; 
	int ret; 
	
	ctl->data = &val;
	ctl->maxlen = sizeof(val);
	if (write) 
	{
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
		if (ret == 0)
		{
			vap->iv_ampdu_mintraffic[WME_AC_BK] = val; 
		}
	}
	else
	{
		val = vap->iv_ampdu_mintraffic[WME_AC_BK];
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
	}
	return ret; 
}

/**
 * This function provides control to read/write be traffic tx aggr threshold
 * field
 *
 * @param 
 *  SYSCTL_HANDLER_ARGS Standard arguments of the Linux sysctl handler
 *
 * @returns 
 *  0 on success, or corresponding negative error code on failure
 */

static int
ieee80211_sysctl_ampdu_traffic_be(SYSCTL_HANDLER_ARGS)
{
	struct ieee80211vap *vap = ctl->extra1;
	unsigned int val; 
	int ret; 
	
	ctl->data = &val;
	ctl->maxlen = sizeof(val);
	if (write)
	{
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
		if (ret == 0)
		{
			vap->iv_ampdu_mintraffic[WME_AC_BE] = val; 
		}
	}
	else
	{
		val = vap->iv_ampdu_mintraffic[WME_AC_BE]; 
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
	}
	return ret; 
}

/**
 * This function provides control to read/write vo traffic tx aggr threshold
 * field
 *
 * @param 
 *  SYSCTL_HANDLER_ARGS Standard arguments of the Linux sysctl handler
 *
 * @returns 
 *  0 on success, or corresponding negative error code on failure
 */
static int
ieee80211_sysctl_ampdu_traffic_vo(SYSCTL_HANDLER_ARGS)
{
	struct ieee80211vap *vap = ctl->extra1;
	unsigned int val; 
	int ret; 
	
	ctl->data = &val;
	ctl->maxlen = sizeof(val);
	if (write) 
	{
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
		if (ret == 0)
		{
			vap->iv_ampdu_mintraffic[WME_AC_VO] = val; 
		}
	}
	else
	{
		val = vap->iv_ampdu_mintraffic[WME_AC_VO];
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
	}
	return ret; 
}

/**
 * This function provides control to read/write vi traffic tx aggr threshold
 * field
 *
 * @param 
 *  SYSCTL_HANDLER_ARGS Standard arguments of the Linux sysctl handler
 *
 * @returns 
 *  0 on success, or corresponding negative error code on failure
 */
static int
ieee80211_sysctl_ampdu_traffic_vi(SYSCTL_HANDLER_ARGS)
{
	struct ieee80211vap *vap = ctl->extra1;
	unsigned int  val; 
	int ret; 
	
	ctl->data = &val;
	ctl->maxlen = sizeof(val);
	if (write) 
	{
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
		if (ret == 0)
		{
			vap->iv_ampdu_mintraffic[WME_AC_VI] = val; 
		}
	}
	else 
	{
		val = vap->iv_ampdu_mintraffic[WME_AC_VI];
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
	}
	return ret; 
}

/**
 * This function provides control to read/write various timing 
 * parameters
 *
 * @param 
 *  SYSCTL_HANDLER_ARGS Standard arguments of the Linux sysctl handler 
 *
 * @returns 
 *  0 on success, or corresponding negative error code on failure
 */
static int
ieee80211_sysctl_msecs_ticks(SYSCTL_HANDLER_ARGS)
{
	int msecs = ticks_to_msecs(*((int *)ctl->extra1));
	int val = 0;
	int ret;
	
	ctl->data   = &val;
        ctl->maxlen = sizeof(int);

	if (write) 
	{
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
		if (ret == 0)
		{
			*((int *)ctl->extra1) = val;
		}
	}
	else
	{
		val = msecs;
		ret = IEEE80211_SYSCTL_PROC_DOINTVEC(ctl, write, filp, buffer,
		                                     lenp, ppos);
	}

	return ret;
}

/**
 * This function provides control to simulate Radar event
 *
 * @param 
 *  SYSCTL_HANDLER_ARGS Standard arguments of the Linux sysctl handler 
 *
 * @returns 
 *  0 on success, or corresponding negative error code on failure
 */
static int
ieee80211_sysctl_radar(SYSCTL_HANDLER_ARGS)
{
	struct ieee80211com *ic  = ctl->extra1;
	
	IEEE80211_LOCK(ic);
	ieee80211_dfs_notify_radar(ic, ic->ic_curchan);
	IEEE80211_UNLOCK(ic); 
	return 0;
}

void
ieee80211_sysctl_attach(struct ieee80211com *ic)
{
}

void
ieee80211_sysctl_detach(struct ieee80211com *ic)
{
}

/*
 * List of 802.11 mesh variables exported to the user via sysctl
 */
static struct ctl_table ieee80211_mesh_template[] = 
{
#if KERNEL_VERSION_BTWN_2_6_(18,30)
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "maxretries",
		.mode         = 0644,
		.data         = &ieee80211_mesh_maxretries,
		.maxlen       = sizeof(int),
		.proc_handler = &proc_dointvec
	},
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "confirmtimeout",
		.mode         = 0644,
		.extra1       = &ieee80211_mesh_confirmtimeout,
		.proc_handler = ieee80211_sysctl_msecs_ticks
	},
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "holdingtimeout",
		.mode         = 0644,
		.extra1       = &ieee80211_mesh_holdingtimeout,
		.proc_handler = ieee80211_sysctl_msecs_ticks
	},
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "retrytimeout",
		.mode         = 0644,
		.extra1       = &ieee80211_mesh_retrytimeout,
		.proc_handler = ieee80211_sysctl_msecs_ticks
	},
	{
		 .ctl_name    = 0 
	}
#elif KERNEL_VERSION_GREATER_THAN_2_6_(32)
	{ 
		.procname     = "maxretries",
		.mode         = 0644,
		.data         = &ieee80211_mesh_maxretries,
		.maxlen       = sizeof(int),
		.proc_handler = &proc_dointvec
	},
	{ 
		.procname     = "confirmtimeout",
		.mode         = 0644,
		.extra1       = &ieee80211_mesh_confirmtimeout,
		.proc_handler = (proc_handler *)ieee80211_sysctl_msecs_ticks
	},
	{ 
		.procname     = "holdingtimeout",
		.mode         = 0644,
		.extra1       = &ieee80211_mesh_holdingtimeout,
		.proc_handler = (proc_handler *)ieee80211_sysctl_msecs_ticks
	},
	{ 
		.procname     = "retrytimeout",
		.mode         = 0644,
		.extra1       = &ieee80211_mesh_retrytimeout,
		.proc_handler = (proc_handler *)ieee80211_sysctl_msecs_ticks
	}
#endif
};
 
/*
 * List of 802.11 HWMP variables exported to the user via sysctl
 */
static struct ctl_table ieee80211_hwmp_template[] = 
{
#if KERNEL_VERSION_BTWN_2_6_(18,30)
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "pathlifetime",
		.mode         = 0644,
		.extra1       = &ieee80211_hwmp_pathtimeout,
		.proc_handler = ieee80211_sysctl_msecs_ticks
	},
	{ .ctl_name     = CTL_AUTO,
		.procname     = "rootimeout",
		.mode         = 0644,
		.extra1       = &ieee80211_hwmp_roottimeout,
		.proc_handler = ieee80211_sysctl_msecs_ticks
	},
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "rootint",
		.mode         = 0644,
		.extra1       = &ieee80211_hwmp_rootint,
		.proc_handler = ieee80211_sysctl_msecs_ticks
	},
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "rannint",
		.mode         = 0644,
		.extra1       = &ieee80211_hwmp_rannint,
		.proc_handler = ieee80211_sysctl_msecs_ticks
	},
	{ 
		.ctl_name     = CTL_AUTO,
		.procname     = "targetonly",
		.mode         = 0644,
		.data         = &ieee80211_hwmp_targetonly,
		.maxlen       = sizeof(int),
		.proc_handler = &proc_dointvec
	},
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "replytoforward",
		.mode         = 0644,
		.data         = &ieee80211_hwmp_replyforward,
		.maxlen       = sizeof(int),
		.proc_handler = &proc_dointvec
	},
	{
		.ctl_name     = 0 
	}
#elif KERNEL_VERSION_GREATER_THAN_2_6_(35)
	{ 
		.procname     = "pathlifetime",
		.mode         = 0644,
		.extra1       = &ieee80211_hwmp_pathtimeout,
		.proc_handler = (proc_handler *)ieee80211_sysctl_msecs_ticks
	},
	{ 
		.procname     = "rootimeout",
		.mode         = 0644,
		.extra1       = &ieee80211_hwmp_roottimeout,
		.proc_handler = (proc_handler *) ieee80211_sysctl_msecs_ticks
	},
	{ 
		.procname     = "rootint",
		.mode         = 0644,
		.extra1       = &ieee80211_hwmp_rootint,
		.proc_handler = (proc_handler *)ieee80211_sysctl_msecs_ticks
	},
	{ 
		.procname     = "rannint",
		.mode         = 0644,
		.extra1       = &ieee80211_hwmp_rannint,
		.proc_handler = (proc_handler *)ieee80211_sysctl_msecs_ticks
	},
	{ 
		.procname     = "targetonly",
		.mode         = 0644,
		.data         = &ieee80211_hwmp_targetonly,
		.maxlen       = sizeof(int),
		.proc_handler = &proc_dointvec
	},
	{ 
		.procname     = "replytoforward",
		.mode         = 0644,
		.data         = &ieee80211_hwmp_replyforward,
		.maxlen       = sizeof(int),
		.proc_handler = &proc_dointvec
	}
#endif
};

/*
 * List of generic 802.11 variables exported to the user via sysctl
 */
static struct ctl_table ieee80211_parent_sysctl_template[] = 
{
#if KERNEL_VERSION_BTWN_2_6_(18,30)
	{
		.ctl_name       = CTL_AUTO,
		.procname       = "%parent",
		.mode           = 0444,
		.maxlen         = IFNAMSIZ,
		.proc_handler   = proc_dostring
	},
	{
		.ctl_name       = CTL_AUTO,
		.procname       = "driver_caps",
		.mode           = 0644,
		.proc_handler = &ieee80211_sysctl_driver_caps
	},
	{
		.ctl_name	= CTL_AUTO,
		.procname	= "debug",
		.mode		= 0644,
		.proc_handler = &ieee80211_sysctl_debug
	},
	{
		.ctl_name	= CTL_AUTO,
		.procname	= "bmiss_max",
		.mode		= 0644,
		.proc_handler = &ieee80211_sysctl_bmiss_max
	},
	{
		.ctl_name	= CTL_AUTO,
		.procname	= "inact_run",
		.mode		= 0644,
		.proc_handler = &ieee80211_sysctl_inact_run
	},
	{
		.ctl_name	= CTL_AUTO,
		.procname	= "inact_probe",
		.mode		= 0644,
		.proc_handler = &ieee80211_sysctl_inact_probe
	},
	{
		.ctl_name	= CTL_AUTO,
		.procname	= "inact_auth",
		.mode		= 0644,
		.proc_handler = &ieee80211_sysctl_inact_auth
	},
	{
		.ctl_name	= CTL_AUTO,
		.procname	= "inact_init",
		.mode		= 0644,
		.proc_handler = &ieee80211_sysctl_inact_init
	},
	{
		.ctl_name	= CTL_AUTO,
		.procname	= "ampdu_mintraffic_bk",
		.mode		= 0644,
		.proc_handler = &ieee80211_sysctl_ampdu_traffic_bk
	},
	{
		.ctl_name	= CTL_AUTO,
		.procname	= "ampdu_mintraffic_be",
		.mode		= 0644,
		.proc_handler = &ieee80211_sysctl_ampdu_traffic_be
	},
	{
		.ctl_name	= CTL_AUTO,
		.procname	= "ampdu_mintraffic_vo",
		.mode		= 0644,
		.proc_handler = &ieee80211_sysctl_ampdu_traffic_vo
	},
	{
		.ctl_name	= CTL_AUTO,
		.procname	= "ampdu_mintraffic_vi",
		.mode		= 0644,
		.proc_handler = &ieee80211_sysctl_ampdu_traffic_vi
	},
	{
		.ctl_name	= CTL_AUTO,
		.procname	= "radar",
		.mode		= 0644,
		.proc_handler	= ieee80211_sysctl_radar
	},
	{ 0 }
#elif KERNEL_VERSION_GREATER_THAN_2_6_(32)
	{ 
		.procname	= "%parent",
		.mode		= 0444,
		.maxlen       = IFNAMSIZ,
		.proc_handler = proc_dostring
	},
	{ 
		.procname	= "driver_caps",
		.mode		= 0644,
		.proc_handler = (proc_handler *)&ieee80211_sysctl_driver_caps
	},
	{ 
		.procname	= "debug",
		.mode		= 0644,
		.proc_handler = (proc_handler *)&ieee80211_sysctl_debug
	},
	{ 
		.procname	= "bmiss_max",
		.mode		= 0644,
		.proc_handler =(proc_handler *) &ieee80211_sysctl_bmiss_max
	},
	{ 
		.procname	= "inact_run",
		.mode		= 0644,
		.proc_handler = (proc_handler *)&ieee80211_sysctl_inact_run
	},
	{ 
		.procname	= "inact_probe",
		.mode		= 0644,
		.proc_handler = (proc_handler *)&ieee80211_sysctl_inact_probe
	},
	{ 
		.procname	= "inact_auth",
		.mode		= 0644,
		.proc_handler = (proc_handler *)&ieee80211_sysctl_inact_auth
	},
	{ 
		.procname       = "inact_init",
		.mode           = 0644,
		.proc_handler = (proc_handler *)&ieee80211_sysctl_inact_init
	},
	{ 
		.procname       = "ampdu_mintraffic_bk",
		.mode           = 0644,
		.proc_handler = (proc_handler *)&ieee80211_sysctl_ampdu_traffic_bk
	},
	{ 
		.procname       = "ampdu_mintraffic_be",
		.mode           = 0644,
		.proc_handler = (proc_handler *)&ieee80211_sysctl_ampdu_traffic_be
	},
	{ 
		.procname       = "ampdu_mintraffic_vo",
		.mode           = 0644,
		.proc_handler =(proc_handler *) &ieee80211_sysctl_ampdu_traffic_vo
	},
	{ 
		.procname       = "ampdu_mintraffic_vi",
		.mode           = 0644,
		.proc_handler = (proc_handler *)&ieee80211_sysctl_ampdu_traffic_vi
	},
	{ 
		.procname       = "radar",
		.mode           = 0644,
		.proc_handler   = (proc_handler *)ieee80211_sysctl_radar
	},
	{       .procname       = NULL,
		.mode           = 0,
		.proc_handler   = NULL,
	}

#endif
};



/*
 * List of generic 802.11 variables exported to the user via sysctl
 */
static struct ctl_table ieee80211_sysctl_template[] = 
{
#if KERNEL_VERSION_BTWN_2_6_(18,30)
	{ 
		.ctl_name     = CTL_AUTO,
		.mode         = 0555,
		.procname     = "generic",
		.child	    = ieee80211_parent_sysctl_template,
	},
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "mesh",
		.mode         = 0555,
		.child	= ieee80211_mesh_template,
	},
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "hwmp",
		.mode         = 0555,
		.child	= ieee80211_hwmp_template,
	},
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "debug",
		.mode         = 0644,
		.proc_handler = ieee80211_sysctl_debug
	},
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "addba_maxtries",
		.mode         = 0644,
		.data         = &ieee80211_addba_maxtries,
		.maxlen       = sizeof(int),
		.proc_handler = &proc_dointvec
	},
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "addba_backoff",
		.mode         = 0644,
		.extra1       = &ieee80211_addba_backoff,
		.proc_handler = ieee80211_sysctl_msecs_ticks
	},
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "addba_timeout",
		.mode         = 0644,
		.extra1       = &ieee80211_addba_timeout,
		.proc_handler = ieee80211_sysctl_msecs_ticks
	},
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "recv_bar",
		.mode         = 0644,
		.data         = &ieee80211_recv_bar_ena,
		.maxlen       = sizeof(int),
		.proc_handler = &proc_dointvec
	},
#ifdef IEEE80211_AMPDU_AGE
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "ampdu_age",
		.mode         = 0644,
		.extra1       = &ieee80211_ampdu_age,
		.proc_handler = &ieee80211_sysctl_msecs_ticks
	},
#endif
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "nol_timeout",
		.mode         = 0644,
		.data         = &ieee80211_nol_timeout,
		.maxlen       = sizeof(int),
		.proc_handler = &proc_dointvec
	},
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "cac_timeout",
		.mode         = 0644,
		.data         = &ieee80211_cac_timeout,
		.maxlen       = sizeof(int),
		.proc_handler = &proc_dointvec
	},
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "ffppsmin",
		.mode         = 0644,
		.data	        = &ieee80211_ffppsmin,
		.maxlen       = sizeof(int),
		.proc_handler = &proc_dointvec
	},
	{
		.ctl_name     = CTL_AUTO,
		.procname     = "ffagemax",
		.mode         = 0644,
		.extra1	    = &ieee80211_ffagemax,
		.proc_handler = &ieee80211_sysctl_msecs_ticks
	},
	{
		.ctl_name     = 0
	}

#elif KERNEL_VERSION_GREATER_THAN_2_6_(35)
	{ 
		.mode         = 0555,
		.procname     = "generic",
		.child	    = ieee80211_parent_sysctl_template,
	},
	{ 
		.procname     = "mesh",
		.mode         = 0555,
		.child	= ieee80211_mesh_template,
	},
	{ 
		.procname     = "hwmp",
		.mode         = 0555,
		.child        = ieee80211_hwmp_template,
	},
	{ 
		.procname     = "debug",
		.mode         = 0644,
		.proc_handler = (proc_handler *)ieee80211_sysctl_debug
	},
	{ 
		.procname     = "addba_maxtries",
		.mode         = 0644,
		.data	        = &ieee80211_addba_maxtries,
		.maxlen       = sizeof(int),
		.proc_handler = &proc_dointvec
	},
	{ 
		.procname     = "addba_backoff",
		.mode         = 0644,
		.extra1       = &ieee80211_addba_backoff,
		.proc_handler = (proc_handler *)ieee80211_sysctl_msecs_ticks
	},
	{ 
		.procname     = "addba_timeout",
		.mode         = 0644,
		.extra1	    = &ieee80211_addba_timeout,
		.proc_handler = (proc_handler *)ieee80211_sysctl_msecs_ticks
	},
	{ 
		.procname     = "recv_bar",
		.mode         = 0644,
		.data    	    = &ieee80211_recv_bar_ena,
		.maxlen       = sizeof(int),
		.proc_handler = &proc_dointvec
	},
#ifdef IEEE80211_AMPDU_AGE
	{ 
		.procname     = "ampdu_age",
		.mode         = 0644,
		.extra1       = &ieee80211_ampdu_age,
		.proc_handler = (proc_handler *)&ieee80211_sysctl_msecs_ticks
	},
#endif
	{ 
		.procname     = "nol_timeout",
		.mode         = 0644,
		.data         = &ieee80211_nol_timeout,
		.maxlen       = sizeof(int),
		.proc_handler = &proc_dointvec
	},
	{
		.procname     = "cac_timeout",
		.mode         = 0644,
		.data         = &ieee80211_cac_timeout,
		.maxlen       = sizeof(int),
		.proc_handler = &proc_dointvec
	},
	{ 
		.procname     = "ffppsmin",
		.mode         = 0644,
		.data	        = &ieee80211_ffppsmin,
		.maxlen       = sizeof(int),
		.proc_handler = &proc_dointvec
	},
	{ 
		.procname     = "ffagemax",
		.mode         = 0644,
		.extra1	    = &ieee80211_ffagemax,
		.proc_handler = (proc_handler *)&ieee80211_sysctl_msecs_ticks
	}
#endif
};

/**
 * This function initalizes various ctl_table entries and registers ctl_table
 * structures with the kernel 
 *
 * @param 
 *  ieee80211vap 
 *
 * @returns 
 *  void
 */
void
ieee80211_sysctl_vattach(struct ieee80211vap *vap)
{
	int space = 0, index = 0,i; 
	char *devname = NULL;
	/* index keeps track of number of sysctl entries */
	
	return; //: Currently not handling this feature.
	/* Allocate memory based on the size of the various tables listed above */
	space = (sizeof(struct sysctl_ctx_list) +
	         sizeof(ieee80211_sysctl_template) + 
	         (2 * sizeof(ieee80211_mesh_template)) +
	         sizeof(ieee80211_parent_sysctl_template) + 
	         sizeof(ieee80211_hwmp_template));
	
	vap->iv_sysctl = kmalloc(space, GFP_KERNEL);
	if (vap->iv_sysctl == NULL)
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s: no memory for sysctl table!\n", __func__));
		return; 
	}
	
	/*   
	 * Reserve space for the device name outside the net_device structure
	 * so that if the name changes we know what it used to be. 
	 */
	devname = kmalloc((strlen(vap->iv_ifp->name) + 1) * sizeof(char), GFP_KERNEL);
	if (devname == NULL) 
	{
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("%s: no memory for VAP name!\n", __func__));
		return;
	}
	strncpy(devname, vap->iv_ifp->name, strlen(vap->iv_ifp->name) + 1);
	
	 /* setup the table */
	memset(vap->iv_sysctl, 0, space);
#if KERNEL_VERSION_BTWN_2_6_(18,30)
	vap->iv_sysctl->sysctl_list[0].ctl_name = CTL_NET;
#endif
	vap->iv_sysctl->sysctl_list[0].procname = "net";
	vap->iv_sysctl->sysctl_list[0].mode = 0555;
	vap->iv_sysctl->sysctl_list[0].child = &vap->iv_sysctl->sysctl_list[2];

     /* [1] is NULL terminator */
#if KERNEL_VERSION_BTWN_2_6_(18,30)
	vap->iv_sysctl->sysctl_list[2].ctl_name = CTL_AUTO;
#endif
	vap->iv_sysctl->sysctl_list[2].procname = devname; 
	vap->iv_sysctl->sysctl_list[2].mode = 0555;
	vap->iv_sysctl->sysctl_list[2].child = &vap->iv_sysctl->sysctl_list[4];
	/* [3] is NULL terminator */
	
	index += 4;
	/* copy the sysctl_tempate into [4] */
	memcpy(&vap->iv_sysctl->sysctl_list[4], ieee80211_sysctl_template,
	    sizeof(ieee80211_sysctl_template));
	
	vap->iv_debug = ieee80211_debug;
	vap->iv_sysctl->sysctl_list[7].extra1 = vap;
	
	/* Increment index by sizeof syctl template */
	index += (sizeof(ieee80211_sysctl_template) / sizeof(struct ctl_table));
	
	/* child of [5] should point to the mesh_template table */
	vap->iv_sysctl->sysctl_list[5].child = &vap->iv_sysctl->sysctl_list[index];
	
	/* Copy mesh related variables to be exported using sysctl */
	memcpy(&vap->iv_sysctl->sysctl_list[index], ieee80211_mesh_template,
	   sizeof(ieee80211_mesh_template));
	
	/* Increment index by mesh template size */
	index += (sizeof(ieee80211_mesh_template) / sizeof(struct ctl_table));
	
	/* The child of [6] should be initialized to hwmp table */
	vap->iv_sysctl->sysctl_list[6].child = &vap->iv_sysctl->sysctl_list[index];
	
	memcpy(&vap->iv_sysctl->sysctl_list[index], ieee80211_hwmp_template,
	   sizeof(ieee80211_hwmp_template));
	index += (sizeof(ieee80211_hwmp_template) / sizeof(struct ctl_table));
	
	vap->iv_sysctl->sysctl_list[4].child = &vap->iv_sysctl->sysctl_list[index];
	memcpy(&vap->iv_sysctl->sysctl_list[index], ieee80211_parent_sysctl_template,
	   8 * sizeof(struct ctl_table));
	
	/* tack on back-pointer to parent device */
	vap->iv_sysctl->sysctl_list[index].data = vap->iv_ic->ic_ifp->name;   
	
	for (i = index; vap->iv_sysctl->sysctl_list[i].procname; i++) 
	{
		if (vap->iv_sysctl->sysctl_list[i].extra1 == NULL)
		{
			/* Initialize all entries to contain vap pointer */
			vap->iv_sysctl->sysctl_list[i].extra1 = vap; 
		}
	} /* End for loop */
	index += 8;

	if (vap->iv_htcaps & IEEE80211_HTC_HT) 
	{
		/* If HT is enabled, add entries for AMPDU-BE/BK/VO/VI fields */
		memcpy(&vap->iv_sysctl->sysctl_list[index], &ieee80211_parent_sysctl_template[8],
		       4 * sizeof(struct ctl_table));
		for (i = index; vap->iv_sysctl->sysctl_list[i].procname; i++) 
		{
			if (vap->iv_sysctl->sysctl_list[i].extra1 == NULL)
			{
				/* Initialize all entries to contain vap pointer */
				vap->iv_sysctl->sysctl_list[i].extra1 = vap; 
			}
		} /* End for loop */
		index += 4;
	}

	if (vap->iv_caps & IEEE80211_C_DFS) 
	{
		/* If DFS is enabled, add an entry for radar */
		memcpy(&vap->iv_sysctl->sysctl_list[index], &ieee80211_parent_sysctl_template[12],
		       2 * sizeof(struct ctl_table)); 
		vap->iv_sysctl->sysctl_list[i].extra1 = vap->iv_ic; 
		index += 2;
	}
	
	/* Finally now register the sysctl list with the kernel */
	vap->iv_sysctl->iv_sysctl_header = sysctl_ctx_init(&vap->iv_sysctl->sysctl_list[0]);
	return;
}

/**
 * This function deinitalizes already registered ctl_table structures with 
 * the kernel and frees up previously allocated dynamic memory
 *
 * @param 
 *  ieee80211vap 
 *
 * @returns 
 *  void
 */
void
ieee80211_sysctl_vdetach(struct ieee80211vap *vap)
{
	return; //Currently not handling this feature.
	if (vap->iv_sysctl != NULL) 
	{
		/* De-register sysctl table */
		sysctl_ctx_free(vap->iv_sysctl->iv_sysctl_header);
		/* Free previously allocated memory */
		free(vap->iv_sysctl->sysctl_list[2].procname, M_DEVBUF);
		free(vap->iv_sysctl, M_DEVBUF);
		vap->iv_sysctl = NULL;
	} /* End if <condition> */
	return;
}

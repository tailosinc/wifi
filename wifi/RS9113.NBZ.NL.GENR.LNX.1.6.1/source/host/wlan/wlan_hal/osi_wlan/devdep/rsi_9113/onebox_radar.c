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
#include <linux/slab.h>

#include "wlan_common.h"
#include "onebox_linux.h"
#include "onebox_hal.h"

/* This file contains all the functions related to radar detection */


/*Minimum pulse width*/
const unsigned int min_pw [MAX_PATTERNS] = { 0, 0, 4, 8, 40, 0, 0, 0, 0, 0, 0, 0, 17, 0, 0}; 
/*Maximum pulse width*/
const unsigned int max_pw [MAX_PATTERNS] = { 4, 9, 14, 22, 110, 2, 7, 5, 5, 15, 23, 23, 40, 9, 9};
/*Minimum pulse repetation interval in microseconds*/
const unsigned int min_pri [MAX_PATTERNS] = {(1428-20), (150-10), (200-10), (200 -10), (1000-50), (333-10), (3846-200), (1389-50), (4000-100), (1000-200), (625-31), (250-13), (250-13), (833-42), (2500-32)};
/*Maximum pulse repetation interval in microseconds*/
const unsigned int max_pri [MAX_PATTERNS] = {(1428+20), (230+10), (500+10), (500 +10), (2000+50), (333+10), (3846+200), (1389+50), (4000+100), (5000+200), (5000+200), (434+20), (500+25), (2500+50), (3333+167)};
/*Minimum number of valid radar pulses in one message (for a particular radar pattern) required to take detection decision*/
const unsigned int min_expected_pulses [MAX_PATTERNS] = {7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 7, 7, 7, 6, 6};
/*The number of iteration for PRI test. This MUST be at least 1. */
const unsigned int pri_iteration_depth [MAX_PATTERNS] = {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};
/*The number of maximum allowed missed pulses for a particular radar pattern*/
const unsigned int max_allowed_missed_pulses [MAX_PATTERNS] = {3, 3, 2, 2, 2, 3, 3, 3, 3, 3, 2, 4, 3, 2, 2};
/*Maximum burst length in microseconds*/
const unsigned int max_burst_length [MAX_PATTERNS] = {18*(1428+20), 29*(230+10), 18*(500+10), 18*(500+10), 4*(2000+100), 9*(333+10), 18*(3846+200), 18*(1389+50), 18*(4000+100), 18*(5000+200), 18* (5000 + 200), 18 * (434+20), 25 * (500+25), 15 * (2500 + 50), 15 * (3333+167) };
const unsigned int pulses_in_burst[MAX_PATTERNS] = {18, 29, 18, 18, 3, 9, 18, 18, 18, 18, 18, 25, 20, 15, 15};

#define MAX_MULTIPLES 10
unsigned int multiples_pri[MAX_MULTIPLES] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned short int staggered_min_pulses[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned char staggered_min_pw[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

unsigned int large_pw = 0;

#ifdef RADAR_AUTO
unsigned short radar_count = 0;
#endif
/* To get the minimum pulse repetition interval */

struct pulse_sample find_next_min(struct pulse_sample *radar, uint8 count, struct pulse_sample min_radar_pulse, int pattern)
{
  uint8 i = 0; 
  struct pulse_sample min_radar_sample ;
  struct pulse_sample *radar_org;
  uint8 flag  = 0;

  radar_org = radar;

  min_radar_sample.time = max_pri[pattern];
  min_radar_sample.pw   = max_pw[pattern];
  for (i = 0; i < count; i++ , radar++)
  {
    if( (radar->time >= min_pri[pattern]) && (radar->time <= max_pri[pattern])) 
    {
      if((radar->pw >= min_pw[pattern] ) && (radar->pw <= max_pw[pattern]))
      {
        if( radar->time < min_radar_sample.time )
        {
          if(( radar->time <= min_radar_pulse.time))
          {
            continue;
          }
          flag = 1;
          min_radar_sample.time = radar->time;
          min_radar_sample.pw   = radar->pw;
        }
      }
    }
  }
  if( flag == 1)
    return min_radar_sample;
  else
    return min_radar_pulse;
}
void populate_multiple_pri( uint16 minimum_radar_pulse)
{
  int i = 0;

  for( i=1; i <= MAX_MULTIPLES; i++ )
  {
    multiples_pri[i -1] = minimum_radar_pulse*i;
  }
  return;
}
struct pulse_sample optimize_min_pri( struct pulse_sample *radar, uint8 count, struct pulse_sample min_radar_pulse, int pattern)
{
  struct pulse_sample *radar_org;
  int i = 0, multiple_match = 0, j = 0, k = 0, max_multiple_match = 0, diff_threshold = 3, avg_pw = 0;
  unsigned int avg_match_time = 0;
  struct pulse_sample next_min_radar, max_match, avg_match;
  uint8 flag = 0;

  if( (pattern != 13) && (pattern != 14 ) )
  {
    diff_threshold = 10;
  }
   
  radar_org = radar;

  for(k = 0; k< count; k++ ) 
  {
    multiple_match = 0;
    for( i = 0; i<MAX_MULTIPLES; i++ )
    {
      radar = radar_org;
      for(j = 0; j < count; j++, radar++)
      {
        if( (radar->time >= (multiples_pri[i] - diff_threshold)) && (radar->time <= (multiples_pri[i] + diff_threshold )))
        {
          multiple_match += i+1;
        }
      }
    }
    if( multiple_match > 6 )
    {
      if( multiple_match > max_multiple_match )
      {
        max_match.time = min_radar_pulse.time;
        max_match.pw   = min_radar_pulse.pw;
        max_multiple_match = multiple_match;
        flag = 1;
      }
    }
    {
      next_min_radar = find_next_min(radar_org, count, min_radar_pulse, pattern);
      populate_multiple_pri(next_min_radar.time);
      if( next_min_radar.time == min_radar_pulse.time )
      {
        break;
      }
      min_radar_pulse.time = next_min_radar.time;
      min_radar_pulse.pw   = next_min_radar.pw;
    }
  }
  if( flag )
  {
    if( (pattern == 13) || (pattern == 14 ) )
    {
      return max_match;
    }
    radar = radar_org;
    avg_match_time = 0;
    avg_pw = 0;
    multiple_match = 0;
    for(j = 0; j < count; j++, radar++)
    {
      if((radar->pw >= min_pw[pattern] ) && (radar->pw <= max_pw[pattern]))
      {
        if( (radar->time >= (max_match.time - diff_threshold)) && (radar->time <= (max_match.time + diff_threshold )))
        {
          avg_match_time += radar->time;
          avg_pw   += radar->pw;
          multiple_match++;
        }
      }
    }
    if( multiple_match )
    {
      avg_match.time = avg_match_time/multiple_match;
      avg_match.pw   = avg_pw/multiple_match;
    }
    else
    {
      printk("No Multiples...Not expected\n");
    }
    return avg_match;
  }
  return min_radar_pulse;
}
struct pulse_sample get_min_pri( struct pulse_sample *radar, uint8 count, int pattern)
{
  uint8 i = 0; 
  struct pulse_sample min_radar_sample;
  struct pulse_sample *radar_org;
  uint8 flag =0;

  radar_org = radar;
  min_radar_sample.time = max_pri[pattern];
  min_radar_sample.pw   = max_pw[pattern];
  for (i = 0; i < count; i++ , radar++)
  {
    if( (radar->time >= min_pri[pattern]) && (radar->time <= max_pri[pattern])) 
    {
      if( radar->time < min_radar_sample.time )
      {
        min_radar_sample.time = radar->time;
        min_radar_sample.pw = radar->pw;
      }
      flag = 1;
    }
  }
  if( flag != 1)
  {
    min_radar_sample.time = 99;
  }
  return min_radar_sample;
}
int check_split_pulses( struct pulse_sample *radar_org, int count, uint8 staggered_depth )
{

  struct pulse_sample *radar, *radar_prv_shift, *radar_nxt_shift;
  uint8 i = 0, j = 0, k = 0, shift_pulse = 0;
  uint32 added_time = 0;
  for( i = 0; i < staggered_depth; i ++ )
  {
    radar = radar_org;
    radar++;
    for(j = 0; j < count; j++, radar++)
    {
      added_time = radar->time + (radar - 1 )->time;
      if((added_time > staggered_min_pulses[i] - 3 ) && (added_time < staggered_min_pulses[i] + 3 ))
      {
        printk("split_pulse found\n");
        printk(" radar->time = %d (radar - 1)->time = %d\n", radar->time, (radar-1)->time);
        (radar -1 )->time = staggered_min_pulses[i];
        radar_prv_shift = radar;
        radar_nxt_shift = radar + 1;
        for(shift_pulse = j; shift_pulse < (count - 1); shift_pulse++) 
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("prev pulse_time %u\n"), radar_prv_shift->time));
          *radar_prv_shift = *radar_nxt_shift;
          ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("Last pulse_time %u\n"), radar_nxt_shift->time));
          radar_prv_shift++;
          radar_nxt_shift++;
        }
        count--;
        (radar -1 )->pw = staggered_min_pw[i];
      }
    }
  }
  for(k=0, radar = radar_org; k < count; k++, radar++)
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("index = %d, time=%u, pwr=%d, width=%d\n"), k, radar->time, radar->debug, radar->pw));
  }
  return count;
}
ONEBOX_STATUS check_staggered_pattern(struct pulse_sample *radar_org, uint8 count, struct pulse_sample min_radar_pulse, int pattern )
{
  uint8 i = 0, j = 0, k = 0, multiple_match = 0, repete_pulse = 0; 
  struct pulse_sample next_min_radar, min_radar;
  struct pulse_sample *radar;
  uint8 flag =0, staggered_depth = 0;
  uint8 expected_multiples = 6;
  uint8 multiple_dist = 0;

  min_radar.time = min_radar_pulse.time;
  min_radar.pw = min_radar_pulse.pw;

  while( expected_multiples >= 2)
  {
    if( !flag )
    {
      multiple_match = 0;
      multiple_dist = 0;
      //for( i = 0; i<3; i++ )
      {
        radar = radar_org;
        for(j = 0; j < count; j++, radar++)
        {
          if( (radar->time >= (multiples_pri[i] - 3)) && (radar->time <= (multiples_pri[i] + 3 )))
          {
            multiple_match++;
            if( multiple_dist > 8 )
            {
              multiple_match--;
              multiple_dist = 0;
            }
          }
          if( multiple_match )
          {
            multiple_dist++;
          }
        }
      }
      if( multiple_match >= expected_multiples )
      {
        for( k =0; k<staggered_depth; k++ )
        {
          if((min_radar_pulse.time >= staggered_min_pulses[k] - 9 ) && (min_radar_pulse.time <= staggered_min_pulses[k] + 9)) 
          {
            repete_pulse = 1;
            break;
          }
        }
        if( !repete_pulse )
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Next pulse = %d\n"), min_radar_pulse.time));
          staggered_min_pulses[staggered_depth] = min_radar_pulse.time;
          staggered_min_pw[staggered_depth]   = min_radar_pulse.pw;
          staggered_depth++;
          if( staggered_depth >= 3 )
            break;
        }
        repete_pulse = 0;
      }
    }

    flag = 0;
    next_min_radar = find_next_min(radar_org, count, min_radar_pulse, pattern);
    if( next_min_radar.time == min_radar_pulse.time )
    {
      expected_multiples--;
      min_radar_pulse.time = min_radar.time;
      min_radar_pulse.pw   = min_radar.pw;

      populate_multiple_pri(min_radar_pulse.time);
      continue;
    }
#if 0
    else if((next_min_radar.time >=  (min_radar_pulse.time - 3 )) && (next_min_radar.time <=  (min_radar_pulse.time + 3 )))
    {
      min_radar_pulse.time = next_min_radar.time;
      min_radar_pulse.pw   = next_min_radar.pw;
      flag = 1;
      continue;
    }
#endif
    min_radar_pulse.time = next_min_radar.time;
    min_radar_pulse.pw   = next_min_radar.pw;

    populate_multiple_pri(min_radar_pulse.time);
  }

  if( staggered_depth )
    return staggered_depth;
  else
    return 0;

}
uint8 check_for_clubbed_pulses(uint16 pulse, uint16 *staggered_array, uint8 staggered_depth)
{
  uint8 i = 0, j = 0, k = 0;
  uint16 summation;

  for(i = 0; i <= 2; i++ )
  {
    for(j = 0; j <= 2; j++ )
    {
      if( staggered_array[2] )
      {
        for(k = 0; k <= 2; k++ )
        {
          summation = i*staggered_array[0] + j*staggered_array[1] + k*staggered_array[2]; 
          if( (pulse >= (summation - 3)) && (pulse <= (summation + 3 )))
          {
            //ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Clubbed pulse found %d no_of_pulses %d\n", pulse, i+j+k)));
            return i+j+k;
          }
        }
      }
      else
      {
          summation = i*staggered_array[0] + j*staggered_array[1]; 
          if( (pulse >= (summation - 3)) && (pulse <= (summation + 3 )))
          {
            //ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Clubbed pulse found %d no_of_pulses %d\n", pulse, i+j)));
            return i+j;
          }
      }
    }
  }
  return 0;
}
ONEBOX_STATUS onebox_radar_detect_algo(WLAN_ADAPTER w_adapter, uint8* msg, uint32 length)
{
  struct ieee80211com *ic = NULL;
  int i=0, j=0, count, shift_pulse, index, x=0, y=0, st_inx;
  int missed = 0;
  struct pulse_sample *radar, *radar_org, *radar_prv_shift, *radar_nxt_shift;
  struct pulse_history pls_history[MAX_PATTERNS];
  unsigned int rcv_time_stamp;
  unsigned int start_time_stamp;
  unsigned int next_pulse_pri = 0;
  uint32 min_index = 0; 
  uint32 max_index = 0;
  struct pulse_sample minimum_radar_pulse;
  uint8 ignore_time_stamp = 0;
  uint8 cont_pulse = 0;
  uint8 cons_pulse = 0;
  uint8 staggered_depth = 0, valid_pulses = 0, flag = 0, large_flag = 0, first_pulse = 0;
  uint16 store_pw = 0;
  uint16 temp_pw = 0;
  uint16 store_pri = 0;
#ifdef RADAR_AUTO
  uint16 radar_detected = 0xffff;
  uint16 radar_fail = 0x0000;
#endif
  w_adapter->radar_detected_flag = 0;


  if( w_adapter->Driver_Mode == WIFI_MODE_ON )
  {
    ic = &w_adapter->vap_com;

    if( !ic )
    {
      return ONEBOX_STATUS_FAILURE;
    }

    switch( ic->ic_regdomain.country )
    {
      case CTRY_CANADA:
      case CTRY_MEXICO:
      case CTRY_UNITED_STATES:
        w_adapter->regdomain = REG_DOMAIN_US;
        break;
      case CTRY_BELGIUM:
      case CTRY_FRANCE:
      case CTRY_GERMANY:
      case CTRY_ITALY:
        w_adapter->regdomain = REG_DOMAIN_EU;
        break;
      case CTRY_JAPAN:
        w_adapter->regdomain = REG_DOMAIN_JP;
        break;
      default: 
        w_adapter->regdomain = REG_DOMAIN_WORLD;
#ifndef RADAR_AUTO
        return onebox_send_radar_req_frame( w_adapter, 0 , 1 );/*Disabling Radar */
#else
        return 0;
#endif
    }
  }
  if( msg[6] == 0xaa )
  {
    w_adapter->flush_pkts++;
  }
  else
  {
    w_adapter->full_pkts++;
  }
  w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_INFO, msg, length + 16);
  w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_INFO, &msg[20], length -4 );

  w_adapter->no_of_pkts++;

  radar_org = (struct pulse_sample*)&msg[20];

  w_adapter->os_intf_ops->onebox_memcpy(w_adapter->radar_pkt, &msg[20] , 256);
  /*As PacketLen contains 16 bytes header and each pulse sample will oppcupy 4 bytes each*/
  //	count = (length - 16) / 4;
  count = (length) / 4;
  /*Subtract 4 bytes since first 4 bytes are receive time stamp */
  count -= 1;
  /* First 8 byte is time stamp (First 4 are lsb and second 4 are msb)*/
  rcv_time_stamp = ONEBOX_CPU_TO_LE32(*(uint32 *)&msg[16]);
  /*loop for travering through all the collected pulse samples*/
  radar = radar_org;

  if(w_adapter->regdomain == REG_DOMAIN_US)
  {
    min_index = MIN_INDEX_US;
    max_index = MAX_INDEX_US;
  }
  else if(w_adapter->regdomain == REG_DOMAIN_JP)
  {
    min_index = MIN_INDEX_JP;
    max_index = MAX_INDEX_JP;
  }
  else if(w_adapter->regdomain == REG_DOMAIN_EU)
  {
    min_index = MIN_INDEX_EU;
    max_index = MAX_INDEX_EU;
  }
  for (j=0; j<count; j++, radar++)
  {
    if((!radar) || (j && (!radar->time)) || (LAST_PULSE_INDICATION == radar->time))
    {
      count = j;
      break;
    }
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\tpulse_index %d \tpulse_time %u \tpulse_width %u \tpulse_power %u\n "), 
          j, radar->time, radar->pw, radar->debug));
    next_pulse_pri += radar->time;
  }
  radar = (radar_org);

  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("Collective delta value %d\n"), next_pulse_pri));

  start_time_stamp = rcv_time_stamp - next_pulse_pri;

  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("No of collected pulses %d end time stamp %u start time stamp %u\n\n"),
        count, rcv_time_stamp,start_time_stamp));

  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("1st pulse also considered \n")));
  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("pulse_time %u\t"), radar->time));
  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("pulse_width %u\t"), radar->pw));
  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("pulse_power %u\t\n"), radar->debug));

  first_pulse = 0;
  store_pw = 0;
  store_pri = 0;
  for (j = 0; j < (count);radar++, j++)
  {
    if( ETSI_MIN_PRI > radar->time)
    {
      if( first_pulse == 0 )
      {
        store_pw = radar->pw;
        first_pulse = 1;
      }
      else
      {
        store_pri += radar->time;
      }
    }
    else
    {
      temp_pw = radar->pw;
      radar->time += store_pri;
      radar->pw  = store_pri + store_pw;
      store_pw = temp_pw;
      store_pri = 0;
    }
  }
  radar = radar_org;
  for(index =0;index < 2; index++)
  {
    radar = radar_org;
    for (j=0; j<count; j++, radar++)
    {
      if( radar->time < ETSI_MIN_PRI)
      {
        radar_prv_shift = radar;
        radar_nxt_shift = radar + 1;
        for(shift_pulse = j; shift_pulse < (count - 1); shift_pulse++) 
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("prev pulse_time %u\n"), radar_prv_shift->time));
          *radar_prv_shift = *radar_nxt_shift;
          ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("Last pulse_time %u\n"), radar_nxt_shift->time));
          radar_prv_shift++;
          radar_nxt_shift++;
        }
        count--;
        //radar = radar_org;
      }
    }
  }
  radar = radar_org;

  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("No of collected pulses %d \n"), count));

  for(j=0, radar = radar_org; j < count; j++, radar++)
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("index = %d, time=%u, pwr=%d, width=%d\n"), j, radar->time, radar->debug, radar->pw));
  }
  if( max_index <= MAX_INDEX_JP )
  {
    for(j=0, radar = radar_org; j < count; j++, radar++)
    {
      if ( (radar->pw >= 40) && (radar->pw <= 100 ))
      {
        if( (radar->time >= 1000) && (radar->time <= 2000 ))
        {
          if( !w_adapter->init_radar_timer )
          {
            w_adapter->os_intf_ops->onebox_init_sw_timer(&w_adapter->long_pulse_timer, (unsigned long)w_adapter,
                (void *)&radar_timer_callback, msecs_to_jiffies(12000));
            w_adapter->init_radar_timer = 1;
          }
          large_flag = 1;
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("LARGE PULSE FOUND\n")));
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("\t\t\t\t\t Pulse No = %d") ,j));
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("\t\ttime=%u, width=%d, pwr=%d\n\n") 
                , radar->time, radar->pw, radar->debug));
        }
      }
    }
  }
  if( large_flag )
  {
    w_adapter->large_pulse++;
    large_flag = 0;
  }
  /*loop for traversing through all the radar patterns*/
#if 0
  if( count < 3 )
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("\t\tInsufficient No Pulses\n")));
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("\t\t  NOT A RADAR PULSE\n")));
    return ONEBOX_STATUS_SUCCESS;
  }
#endif

  for (i = min_index; i <= max_index; i++)
  {
    radar = radar_org;

    minimum_radar_pulse = get_min_pri( radar, count, i);

    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(" OLD Minimum PRI = %d pattern = %d\n"), minimum_radar_pulse.time, i));

    if( minimum_radar_pulse.time < 100 )
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\t\tVery Low PRI\n")));
      continue;
    }
    else if( minimum_radar_pulse.time > 10000)
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\t\tVery high PRI\n")));
      continue;
    }

    populate_multiple_pri(minimum_radar_pulse.time);
    
    if( (i == 13) || (i == 14))
    {
      staggered_depth = check_staggered_pattern( radar_org, count, minimum_radar_pulse, i );
    }
    else
    {
      staggered_depth = 0;
    }

    count = check_split_pulses( radar_org, count, staggered_depth );



    populate_multiple_pri(minimum_radar_pulse.time);

    radar = radar_org;
    minimum_radar_pulse = optimize_min_pri( radar, count, minimum_radar_pulse, i);

    populate_multiple_pri(minimum_radar_pulse.time);


    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(" NEW Minimum PRI = %d PW = %d pattern = %d\n"), minimum_radar_pulse.time, minimum_radar_pulse.pw, i));
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\t\t\tPRI test iteration for pattern %d\n"), i));
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\t\t\t  total no of Puses = %d\n"),count));

    if( !staggered_depth )
    {
      staggered_min_pulses[0] = minimum_radar_pulse.time;
      staggered_min_pw[0] = minimum_radar_pulse.pw; 
      populate_multiple_pri(staggered_min_pulses[0]); 
      staggered_depth = 1;
    }


    pls_history[i].count = 0;
    pls_history[i].missed_pls_cnt = 0;
    cont_pulse = 0;
    cons_pulse = 0;
    /*loop for travering through all the collected pulse samples*/
    for (j = 0, radar = radar_org; j < count; j++, radar++)
    {
      if(!(radar->time || j)) 
      {
        continue;
      }
      if( radar->time < ETSI_MIN_PRI )
	      continue;
      ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\t\t\t\t\t Pulse No = %d") ,j));
      ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\t\ttime=%u, width=%d, pwr=%d\n") 
            , radar->time, radar->pw, radar->debug));

      /*If pulse width is out of range, look for next pulse sample*/
      if ( (radar->pw > max_pw[i] || radar->pw < min_pw[i]) )
      {

        ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(
                "\tpulse width is outof range min pw %d recieved %d max pw %d\n\n"), min_pw[i], radar->pw, max_pw[i]));

        pls_history[i].missed_pls_cnt++;
#if 0
        if( cons_pulse < 4 )
          cons_pulse = 0;
        if(cont_pulse < pri_iteration_depth[i] )
          cont_pulse = 0;
#endif

        ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("RDR: missed pulses=%d\n"), pls_history[i].missed_pls_cnt));
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("RDR: pulse count=%d\n"), pls_history[i].count));
        if( pls_history[i].missed_pls_cnt > max_allowed_missed_pulses[i] )
        {
          pls_history[i].count = 0;
          pls_history[i].missed_pls_cnt = 0;
          cont_pulse = 0;
          cons_pulse = 0;
        }
        continue;
      }
      /*If this is 1st pulse OR the pulse arrival is out of maximum burst length of the pattern, restart count*/
      /*As no radar pattern will have less than 100 or greater than 10000 microseconds PRI*/
      else 
      {
        if( (minimum_radar_pulse.time < min_pri[i]) || (minimum_radar_pulse.time > max_pri[i]) )
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("No PRI falls within range of %d pattern, minimum pulse time = %d\n"),i,minimum_radar_pulse.time));
          pls_history[i].count = 0;
          pls_history[i].missed_pls_cnt = 0;
          cont_pulse = 0;
          cons_pulse = 0;
          break;
        }
        missed = 0;
        missed += radar->time/minimum_radar_pulse.time;
        if( missed == 0 )
        {
          missed = 1;
        }
        for( x=0; x<MAX_MULTIPLES; x++ )
        {
          if( missed - x )
          {
            continue;
          }
          else
          {
            if(( x >= pri_iteration_depth[i] ) && (staggered_depth < 2 ))
            {
              pls_history[i].missed_pls_cnt += missed;
#if 0
              if( cons_pulse < 4 )
                cons_pulse = 0;
              if(cont_pulse < pri_iteration_depth[i] )
                cont_pulse = 0;
#endif
              break;
            }
            for( y = 0; y < pri_iteration_depth[i] - 1; y++ )
            {
              for(st_inx =0; st_inx < staggered_depth; st_inx++ )
              {
                if( staggered_depth )
                {
                  populate_multiple_pri(staggered_min_pulses[st_inx]); 
                  minimum_radar_pulse.time = staggered_min_pulses[st_inx];
                  minimum_radar_pulse.pw   = staggered_min_pw[st_inx]; 
                }
                if( (radar->time >= (multiples_pri[y] - 4)) && (radar->time <= (multiples_pri[y] + 4 )))
                {
                  if(( radar->pw >= minimum_radar_pulse.pw - 5 ) && ( radar->pw <= minimum_radar_pulse.pw +5 ))
                  {
                    pls_history[i].count += y+1;
                    cont_pulse += y+1;
                    cons_pulse++;
                    pls_history[i].start_time = rcv_time_stamp - next_pulse_pri; //radar_with_tstamp->start_time;
                    pls_history[i].end_time = rcv_time_stamp;//radar->time;
                    pls_history[i].pw = radar->pw;
                    pls_history[i].pwr = radar->debug;
                    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\t\t\t\t\t Pulse No = %d") ,j));
                    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\t\ttime=%u, width=%d, pwr=%d\n") 
                          , radar->time, radar->pw, radar->debug));
                    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("matched min PRI = %d multiple = %d\n, radar->time = %d"),minimum_radar_pulse.time, multiples_pri[y], radar->time));
                    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("RDR: missed pulses=%d\n"), pls_history[i].missed_pls_cnt));
                    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("RDR: pulse count=%d\n"), pls_history[i].count));
                    missed = 0;
                    flag = 1;
                  }
                  else
                  {
                    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("PW of multiples dont match with Min pulse\n")));
                  }
                  break;
                }
              }
              if( flag )
              {
                flag = 0;
                break;
              }
              if( staggered_depth >= 2 )
              {
                break; //For staggered patterns, the consucutive pulses are different, hence they cannot be perfect multiples
              }
            } 
            if( missed )
            {
              if( staggered_depth > 1 )
              {
                ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("checking for clubbed_pulses\n")));
                valid_pulses = check_for_clubbed_pulses( radar->time, staggered_min_pulses, staggered_depth);
                if( (valid_pulses > 0) && (valid_pulses < 5 ) )
                {
                  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\t\t\t\t\t Pulse No = %d") ,j));
                  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\t\ttime=%u, width=%d, pwr=%d\n") 
                        , radar->time, radar->pw, radar->debug));
                  pls_history[i].count += valid_pulses;
                  cont_pulse += valid_pulses;
                  cons_pulse++;
                  missed = 0;
                  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("RDR: missed pulses=%d\n"), pls_history[i].missed_pls_cnt));
                  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("RDR: pulse count=%d\n"), pls_history[i].count));
                }
                if( valid_pulses )
                {
                  missed = 0;
                }
                valid_pulses = 0;
              }
#if 0
              if( missed )
              {
                if( cons_pulse < 4 )
                  cons_pulse = 0;
                if(cont_pulse < pri_iteration_depth[i] )
                  cont_pulse = 0;
              }
#endif
            }
            pls_history[i].missed_pls_cnt += missed;
            ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("RDR: missed pulses=%d\n"),pls_history[i].missed_pls_cnt));
            ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("RDR: pulse count=%d\n"), pls_history[i].count));
          }
        }
        if( missed >= MAX_MULTIPLES)
        {
            pls_history[i].missed_pls_cnt += missed;
        }
      }
      if( pls_history[i].missed_pls_cnt > max_allowed_missed_pulses[i] )
      {
        pls_history[i].count = 0;
        pls_history[i].missed_pls_cnt = 0;
        cont_pulse = 0;
        cons_pulse = 0;
        continue;
      }
      else if( !pls_history[i].missed_pls_cnt )
      {
        ignore_time_stamp = 1;
      }
      else
      {
        ignore_time_stamp = 0;
      }
      if ((pls_history[i].count >= min_expected_pulses[i]) &&
          (pls_history[i].missed_pls_cnt <= max_allowed_missed_pulses[i]))
      {
        if( cons_pulse >= 4 )
        {
          if( ignore_time_stamp )
          {
            w_adapter->radar_detected_flag = 1;
            ignore_time_stamp = 0;
          }
          else if( cont_pulse >= pri_iteration_depth[i])
          {
            w_adapter->radar_detected_flag = 1;
            ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(" RADAR detected due to cont pulse\n")));
          }
          else if((rcv_time_stamp - start_time_stamp) <=  max_burst_length[i])
          {
            w_adapter->radar_detected_flag = 1;
          }
          else
          {
            w_adapter->radar_detected_flag = 0;
          }
        }
        else
        {
        	ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Radar failed due to less no of consecutive pulse\n")));
          	continue;
        }
      }
      if( w_adapter->radar_detected_flag )
      {
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("Radar detected at %d pulse\n"),j));
        for(j=0, radar = radar_org; j < count; j++, radar++)
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("index = %d, time=%u, pwr=%d, width=%d\n"), j, radar->time, radar->debug, radar->pw));
        }
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(" NEW Minimum PRI = %d PW = %d pattern = %d\n"), minimum_radar_pulse.time, minimum_radar_pulse.pw, i));
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("expected pules = %d   receive = %d\n"), min_expected_pulses[i], pls_history[i].count));
        ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("max_missed  = %d   received missed = %d\n"), max_allowed_missed_pulses[i], pls_history[i].missed_pls_cnt));
        for(j=0, radar = radar_org; j < count; j++, radar++)
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("time=%u, pwr=%d, width=%d\n"), radar->time, radar->debug, radar->pw));
        }

        if( (i >= MIN_INDEX_US) && (i <= MAX_INDEX_US) ) 
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\n******************US RADAR detected : %d*****************\n"), i));
        }
        else if( (i >= MIN_INDEX_JP) && (i <= MAX_INDEX_JP) )
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\n******************JAPAN RADAR detected : %d*****************\n"), i));
        }
        else if( (i >= MIN_INDEX_EU) && (i <= MAX_INDEX_EU) )
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("\n******************EUROPE RADAR detected: %d*****************\n"), i));
        }
        w_adapter->radar_detected++;
        if( w_adapter->Driver_Mode == WIFI_MODE_ON )
	{
#ifdef RADAR_AUTO
		onebox_send_radar_packets_to_matlab(w_adapter,radar_detected);	
#else
		w_adapter->net80211_ops->onebox_radar_notify(ic, ic->ic_curchan);
#endif
	}
	else
	{
#ifdef RADAR_AUTO
		onebox_send_radar_packets_to_matlab(w_adapter,radar_detected);	
#else
		w_adapter->os_intf_ops->onebox_set_event(&(w_adapter->radar_event));
#endif
	}
        break;
      }
      else if(w_adapter->large_pulse >= 8 )
      {
        ONEBOX_DEBUG(ONEBOX_ZONE_INFO, (TEXT("LONG PULSE RADAR DETECTED\n")));
        if( w_adapter->Driver_Mode == WIFI_MODE_ON )
	{
#ifdef RADAR_AUTO
		onebox_send_radar_packets_to_matlab(w_adapter,radar_detected);	
#else
		w_adapter->net80211_ops->onebox_radar_notify(ic, ic->ic_curchan);
#endif
	}
        else
        {
#ifdef RADAR_AUTO
		onebox_send_radar_packets_to_matlab(w_adapter,radar_detected);	
#else
          w_adapter->net80211_ops->onebox_radar_notify(ic, ic->ic_curchan);
#endif
        }
        w_adapter->long_radar_detected++;
        w_adapter->radar_detected_flag = 1;
        w_adapter->large_pulse = 0;
        w_adapter->init_radar_timer = 0;
        w_adapter->os_intf_ops->onebox_remove_timer(&w_adapter->long_pulse_timer);
        break;
      }
      if ((pls_history[i].count >= min_expected_pulses[i]) &&
          (pls_history[i].missed_pls_cnt <= max_allowed_missed_pulses[i]))
      {
        if((rcv_time_stamp - start_time_stamp) >  max_burst_length[i])
        {
          ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Time stamp Failure -pattern no :%d\n"), i));
          w_adapter->time_stamp_fail++;
        }
      }
      ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("rcv_time_stamp - start_time_stamp = %d,max_burst_length[%d] = %d\n\n"),
            rcv_time_stamp - start_time_stamp, i, max_burst_length[i]));

    }  /*End of the for loop for travering through all the collected pulse samples*/


    if (w_adapter->radar_detected_flag)
    {
      ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("RDR: pulse count=%d, missed pulses=%d, index=%d, start_time=%u, end_time=%u\n\n"),
            pls_history[i].count, pls_history[i].missed_pls_cnt, i, pls_history[i].start_time, pls_history[i].end_time));
      ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT(" PATTERN PROPERTY : MIN_PRI = %d\tMAX_PRI = %d\n"), min_pri[i], max_pri[i]));
#ifndef RADAR_AUTO
      return onebox_send_radar_req_frame( w_adapter, 0 , 1 );/*Disabling Radar */
#else
      return 0;
#endif
    }
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("RDR: missed pulses=%d\t"),	pls_history[i].missed_pls_cnt));
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("allowed missed pulse =%d\n"), max_allowed_missed_pulses[i]));
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("RDR: pulse count=%d\t"), pls_history[i].count));
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("min_expected_pulses =%d\n"), max_allowed_missed_pulses[i]));
    ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("rcv_time_stamp - start_time_stamp = %d,max_burst_length[%d] = %d\n\n\n"),
          rcv_time_stamp - start_time_stamp, i, max_burst_length[i]));

  } /*End of the for loop for traversing through all the radar patterns*/

  if( w_adapter->radar_detected_flag == 0 )
  {
	  ONEBOX_DEBUG(ONEBOX_ZONE_ERROR,(TEXT("Not a radar Pulse\n\n\n")));
#ifdef RADAR_AUTO
	  onebox_send_radar_packets_to_matlab(w_adapter,radar_fail);	
#endif
  }
  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("min_index = %d, max_index = %d\n"), min_index, max_index));
  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("radar detection count  = %d\n"),w_adapter->radar_detected));
  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("Long pulse RADAR = %d\n"),w_adapter->long_radar_detected));
  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("time_stamp_failures    = %d\n"),w_adapter->time_stamp_fail));
  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("no_of_pkts  = %d\n"),w_adapter->no_of_pkts));
  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("no_of_flush = %d\n"),w_adapter->flush_pkts));
  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("no_of_full  = %d\n"),w_adapter->full_pkts));
  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("large_pulse = %d\n"),w_adapter->large_pulse));
  //return onebox_send_radar_req_frame( w_adapter, 1 , 1 );/*enabling Radar */
  return ONEBOX_STATUS_SUCCESS;

}


/**
 * This function prepares radar request frame and send it to LMAC.
 *
 * @param  Pointer to Adapter structure.  
 * @param request to process (1- to enable radar detection/ 0- disable radar detection)
 * @return 0 if success else -1. 
 */
ONEBOX_STATUS onebox_send_radar_req_frame(WLAN_ADAPTER w_adapter, uint8 radar_req, uint8 intr_clr)
{
  onebox_mac_frame_t *mgmt_frame;
  ONEBOX_STATUS status;
  uint8  pkt_buffer[FRAME_DESC_SZ];

  FUNCTION_ENTRY (ONEBOX_ZONE_MGMT_SEND);

  ONEBOX_DEBUG(ONEBOX_ZONE_DEBUG,
      (TEXT("===> Send Radar Request frame <===\n")));
  ONEBOX_DEBUG(ONEBOX_ZONE_INFO,(TEXT("Radar Request is %d\n"), radar_req));

  mgmt_frame = (onebox_mac_frame_t *)pkt_buffer;
  w_adapter->os_intf_ops->onebox_memset(mgmt_frame, 0, FRAME_DESC_SZ);

  if( radar_req )
  {
    w_adapter->radar_detected_flag = 0;
  }

  mgmt_frame->desc_word[0] = ONEBOX_CPU_TO_LE16(ONEBOX_WIFI_MGMT_Q << 12);
  mgmt_frame->desc_word[1] = ONEBOX_CPU_TO_LE16(RADAR_REQUEST);
  mgmt_frame->desc_word[4] = ONEBOX_CPU_TO_LE16(radar_req);
  mgmt_frame->desc_word[5] = ONEBOX_CPU_TO_LE16(intr_clr);

  w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_MGMT_SEND, (PUCHAR)mgmt_frame, (FRAME_DESC_SZ));
  status = onebox_send_internal_mgmt_frame(w_adapter, (uint16 *)mgmt_frame, FRAME_DESC_SZ);
  return status;
}
void radar_timer_callback( WLAN_ADAPTER w_adapter)
{
  struct ieee80211com *ic = NULL;
  if( w_adapter->Driver_Mode == WIFI_MODE_ON )
  {
    ic = &w_adapter->vap_com;

    if( !ic )
    {
      return;
    }
  }
  if( w_adapter->large_pulse >= 5 )
  {
    ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("LONG PULSE RADAR DETECTED\n")));
    if( w_adapter->Driver_Mode == WIFI_MODE_ON )
    {
#ifndef RADAR_AUTO
    w_adapter->net80211_ops->onebox_radar_notify(ic, ic->ic_curchan);
#endif
    }
    w_adapter->long_radar_detected++;
#ifndef RADAR_AUTO
    onebox_send_radar_req_frame( w_adapter, 0 , 1 );/*Disabling Radar */
#endif
  }
  ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("TIMER EXPIRED Large pulses = %d\n"), w_adapter->large_pulse));
  w_adapter->large_pulse = 0;
  w_adapter->os_intf_ops->onebox_remove_timer(&w_adapter->long_pulse_timer);
  w_adapter->init_radar_timer = 0;
  return;
}
#ifdef RADAR_AUTO
ONEBOX_STATUS onebox_send_radar_packets_to_matlab(WLAN_ADAPTER w_adapter, uint16 radar_indication)
{
	uint64 radar_intr_state;
	uint16 *radar_packet;
	struct radar_app *radar_temp;
	radar_count = radar_count + 1;
	if (radar_count == 1)
	{
		w_adapter -> radar_front = NULL;
		w_adapter -> radar_rear = NULL;
	}
	w_adapter->os_intf_ops->onebox_memcpy(&w_adapter->radar_pkt[128], &radar_indication , 2);
	radar_packet = (uint16 *)kmalloc(258,GFP_ATOMIC);
	w_adapter->os_intf_ops->onebox_memcpy(radar_packet,(uint16 *)w_adapter->radar_pkt, 258);
	w_adapter->core_ops->onebox_dump(ONEBOX_ZONE_ERROR,radar_packet, 258);
	radar_temp = kmalloc(sizeof(struct radar_app),GFP_ATOMIC);
	if (!radar_temp) {
    		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("NO MEMORY\n")));
	}
	radar_temp-> ptr = (uint16 *) radar_packet;
	radar_temp->next = NULL;
	w_adapter->os_intf_ops->onebox_acquire_spinlock(&w_adapter->radar_lock,&radar_intr_state);
	if ((w_adapter->radar_front == NULL) && (w_adapter->radar_rear == NULL)) 
	{
		w_adapter->radar_front = radar_temp;
		w_adapter->radar_rear = radar_temp;
  		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Front Address = %p\n"), w_adapter->radar_front));
  		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Rear Address = %p\n"), w_adapter->radar_rear));
	}
	else {
		w_adapter -> radar_rear -> next = radar_temp;
		w_adapter -> radar_rear = radar_temp;
  		ONEBOX_DEBUG(ONEBOX_ZONE_ERROR, (TEXT("Rear Address = %p\n"), w_adapter->radar_rear));
	}
	w_adapter->os_intf_ops->onebox_release_spinlock(&w_adapter->radar_lock,radar_intr_state);
}
#endif

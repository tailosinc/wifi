#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_regdomain.h>
#ifdef IEEE80211_SUPPORT_SUPERG
#include <net80211/ieee80211_superg.h>
#endif
#include <net80211/ieee80211_ratectl.h>

void ieee80211_process_quiet_element(struct ieee80211vap *vap, uint8_t *pkt, uint16_t node_num)
{

  /* Extract Quiet parameters from beacon indication received */
  if (node_num >= MAX_QUIET_INFO_POOL_SIZE) // memory allocated is sufficient only for MAX_QUIET_INFO_POOL_SIZE elments, so ignoring remaining elements. Hoping they can be retrieved in next TBTT.
  {
    IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("MAXIMUM Quiet IE Level is reached \n"));
    return;
  }

  if(node_num == 0) // reseting memory only once when first element is found.
  {
    vap->quiet_list = vap->quiet_info_pool; /*Initializing quiet_list to the first element of quiet pool*/
    memset((uint8_t *)vap->quiet_list, 0, sizeof(struct quiet_info_s) * MAX_QUIET_INFO_POOL_SIZE); 
  }
  /*Add quiet element to the linked list of quiet structure*/
  ieee80211_add_to_ascending_linkedlist(vap, &vap->quiet_list, pkt, 0, node_num); 
}

#define SWAP_BYTE(x) (x << 8 | x >> 8)
void ieee80211_add_to_ascending_linkedlist (struct ieee80211vap *vap, struct quiet_info_s **quiet_list, uint8_t *pkt, uint32_t residue, uint16_t node_num)
{
  struct quiet_info_s *quiet_ele = &(vap->quiet_info_pool[node_num]);
  struct quiet_info_s *temp = *quiet_list;

  /*Copying the quiet element from the packet*/
  memcpy((uint8_t *)quiet_ele, (uint8_t *)pkt, (2 + QUIET_LENGTH));

  quiet_ele->next = 0; /*Making the next pointer to zero*/

  quiet_ele->quiet_duration = SWAP_BYTE(quiet_ele->quiet_duration); //Convert from Network byte to Host
  quiet_ele->quiet_offset = SWAP_BYTE(quiet_ele->quiet_offset);

  if(quiet_ele->quiet_duration == 0) // so that this kind of quiet ele is not given to lmac.
  {
    quiet_ele->element_id = 0;
    return;
  }
  
  /*Calculating the timeout tsf of the received quiet element*/
  quiet_ele->quiet_tsf_tout = ((quiet_ele->quiet_cnt)*(vap->iv_bss->ni_intval)) + (quiet_ele->quiet_offset);

  /*New node being added for the fresh list*/
  if (node_num == 0)
  {
    *quiet_list = quiet_ele;
    (*quiet_list)->next = 0; 
  }
  /*New node quiet start time is before the quiet start time of header node of the list*/
  else if ((((*quiet_list)->quiet_cnt == quiet_ele->quiet_cnt) && ((*quiet_list)->quiet_offset > quiet_ele->quiet_offset)) || ( (*quiet_list)->quiet_cnt > quiet_ele->quiet_cnt))
  {
    *quiet_list = quiet_ele;
    (*quiet_list)->next	= temp;
  } 
  else
  { // atleast one element is already present.
    while (temp->element_id != 0)
    {
      if(temp->next == 0) /*If there is only one previous node*/ /* Expecting start of quiet_info_pool is not zero */
      { 
        temp->next = quiet_ele;
        return;
      }
      /*When the new element falls in between two existing nodes of the linked list*/ /* when an elment comes till this point is has proven that it is greater than the node pointed by temp, it should be compared with temp->next */
      else if (((temp->next->quiet_cnt == quiet_ele->quiet_cnt) && (temp->next->quiet_offset > quiet_ele->quiet_offset)) || (temp->next->quiet_cnt > quiet_ele->quiet_cnt))
      {
        quiet_ele->next = temp->next;
        temp->next = quiet_ele;
        return;
      }
      temp = temp->next; /*Goto the next node*/
    }
  }
  return; 
}

static void ieee80211_quiet_end(unsigned long arg)
{
	struct ieee80211vap *vap = (struct ieee80211vap *)arg;

	if(!vap) {
		IEEE80211_DBG_PRINT(vap, IEEE80211_MSG_ERROR, ("Null Vap pointer in %s Line %d \n", __func__, __LINE__));
		return;
	}
	
	IEEE80211_LOCK_ASSERT(vap->iv_ic);
	vap->iv_block(vap, STA_CONNECTED, 0);
	if (vap->hal_priv_vap->bgscan_params_ioctl.bgscan_periodicity)
		vap->iv_mod_bgscan_params(vap,(uint16_t *)&vap->hal_priv_vap->bgscan_params_ioctl, 0);
}

static void ieee80211_quiet_start(unsigned long arg)
{
  struct ieee80211vap *vap = (struct ieee80211vap *)arg;
  struct ieee80211com *ic = vap->iv_ic;
  struct quiet_info_s *quiet_list = vap->quiet_list;
	struct ieee80211_dfs_state *dfs = &ic->ic_dfs;
  struct quiet_info_s *temp = vap->quiet_list;

  vap->iv_block(vap, STA_DISCONNECTED, 1);
  if (vap->hal_priv_vap->bgscan_params_ioctl.bgscan_periodicity)
    vap->iv_mod_bgscan_params(vap,(uint16_t *)&vap->hal_priv_vap->bgscan_params_ioctl, 2);
  callout_reset(&dfs->quiet_timer, msecs_to_jiffies(quiet_list->quiet_duration), ieee80211_quiet_end, vap);

  if(temp->next != 0)
  {
    vap->quiet_list = temp->next;
  }

  /*Free the data memory*/
  temp->element_id = 0;	//Make this list invalid
  ieee80211_process_quiet(vap);
  return;
}

/*FUNCTION************************************************************
 *
 * Function Name : ieee80211_process_quiet
 *
 * Description   : This function handles 11h quiet processing
 *                 
 * Return Value  :
 *
 * Parameters    :
 * ---------------+-----+-----+-----+-----------------------------
 *  Name          | I/P | O/P | I/O |               Purpose
 * ---------------+-----+-----+-----+-----------------------------
 * *app           |  X  |     |     | Pointer to the application structre 
 *END*****************************************************************/
void ieee80211_process_quiet(struct ieee80211vap *vap)
{
  struct quiet_info_s *quiet_list = vap->quiet_list;
	struct ieee80211com *ic = vap->iv_ic;
	struct ieee80211_dfs_state *dfs = &ic->ic_dfs;
  uint32_t timeout = 1; // so that timeout configured into timer is not zero

  /*End Quiet processing when the Quiet list contains nothing*/
  if (quiet_list->element_id == 0)
  {
    return;
  }

  /*When the Quiet interval is immediately indicated to LMAC without configuring the timer*/
  if( quiet_list->quiet_tsf_tout == 0 )
  {
    /* Send Quiet confirm frame to LMAC */
    ieee80211_quiet_start((unsigned long)vap);
#if 0
    /*Traversing to the next node and Reset the current node*/
    if(temp->next != 0)
    {
      vap->quiet_list = temp->next;
    }
    /*Reset the current node*/
    temp->element_id = 0;	//Make this list invalid
    ieee80211_process_quiet(vap); /*Access the next quiet node*/
#endif    
    return;
  }

  timeout = quiet_list->quiet_tsf_tout; //Normal Case /*Same equation handles the case of wrap mathematically if operand sizes are same */

  /* Configure the timer with calculated timeout */
  callout_reset(&dfs->quiet_start_timer, msecs_to_jiffies(timeout), ieee80211_quiet_start, vap);
}

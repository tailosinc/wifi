#!/bin/sh

echo -e "\033[31mRunning Upgrade Script...\033[0m"

#Driver Mode 1 WiFi mode, 2 for Eval/PER Mode, 3 for Firmware_upgrade
rm firmware/flash_content
#cp RS9113_RS8111_calib_values.txt RS9113_RS8111_calib_values_copy.txt
sed -e 's/,//g' RS9113_RS8111_calib_values.txt > RS9113_RS8111_calib_values_copy.txt
xxd -r -ps RS9113_RS8111_calib_values_copy.txt firmware/flash_content
rm RS9113_RS8111_calib_values_copy.txt 
sh remove_all.sh
sleep 2;
echo -e "\033[31mRmmoding any left *.ko....\033[0m"
touch flash.sh
#sed 's/=2/=5/' /home/rsi/release/insert.sh > /home/rsi/release/insert_up.sh 
echo -e "\033[31mEntered into Upgrade Mode....\033[0m"
echo -e "\033[31mInserting .kos with Driver Mode 5....\033[0m"

cat /dev/null > /var/log/messages
insmod onebox_common_gpl.ko

insmod wlan.ko
insmod wlan_wep.ko
insmod wlan_tkip.ko
insmod wlan_ccmp.ko
insmod wlan_aes_cmac.ko
insmod wlan_acl.ko
insmod wlan_xauth.ko
insmod wlan_scan_sta.ko
insmod onebox_wlan_nongpl.ko
insmod onebox_wlan_gpl.ko

#Power_mode type
# 0 - HIGH  POWER MODE
# 1 - MEDIUM POWER MODE 
# 2 - LOW POWER MODE
BT_RF_TX_POWER_MODE=0
BT_RF_RX_POWER_MODE=0

PARAMS=$PARAMS" bt_rf_tx_power_mode=$BT_RF_TX_POWER_MODE"
PARAMS=$PARAMS" bt_rf_rx_power_mode=$BT_RF_RX_POWER_MODE"

insmod onebox_bt_nongpl.ko $PARAMS 
insmod onebox_bt_gpl.ko

insmod onebox_zigb_nongpl.ko
insmod onebox_zigb_gpl.ko

sh flash.sh
sleep 45
echo -e "\033[31mUpgrade Successful...\033[0m"
sh remove_all.sh
sleep 1
cd flash
./flash 4 5
sleep 1
cd ..
exit
#state=`cat /proc/onebox-hal/stats | grep "COMMON HAL FSM_STATE" | cut -d ':' -f 2 | cut -d ':' -f 1`
#state=FSM_MAC_INIT_DONE
error=0
while [ 1 ] 
do
state=`cat /var/log/messages | grep "FIRMWARE UPGRADED TO FLASH SUCCESSFULLY" | cut -d ':' -f 2 | cut -d ':' -f 1`
  if [ "$state" ==  "13" ]
  then 
    sh remove_all.sh
    echo -e "\033[31mUpgrade Successful...\033[0m"
  else
    if [ "$error" == 5 ]
    then
      echo -e "\033[31m !!!!! DANGER !!!!\033[0m"
      echo -e "\033[31m Upgrade Failed\033[0m"
      echo -e "\033[31m Upgrade Failed\033[0m"
    else
      error=`expr $error + 1`
      sleep 10
    fi
  fi
done


if [ "$1" = "" ] || [ "$2" = "" ] || [ "$3" = "" ] || [ "$4" = "" ] || [ "$5" = "" ] || [ "$6" = "" ] || [ "$7" = "" ] || [ "$8" = "" ] || [ "$9" = "" ] || [ "${10}" = "" ] || [ "${11}" = "" ] || [ "${12}" = "" ] || [ "${13}" = "" ] || [ "${14}" = "" ]
then
  echo -e "\033[31m       Error : 1st Argument  : EEPROM VERSION\033[0m"
  echo -e "\033[31m               2nd Argument  : EEPROM SIZE \033[0m"
  echo -e "\033[31m               3rd Argument  : EEPROM TYPE \033[0m"
  echo -e "\033[31m               4th Argument  : MODULE TYPE \033[0m"
  echo -e "\033[31m               5th Argument  : MODULE NUMBER \033[0m"
  echo -e "\033[31m               6th Argument  : APPEND PROTOCOL \033[0m"
  echo -e "\033[31m               7th Argument  : DIGITAL CHIP VERSION \033[0m"
  echo -e "\033[31m               8th Argument  : MODULE VERSION \033[0m"
  echo -e "\033[31m               9th Argument  : RF_CHIP VERSION \033[0m"
  echo -e "\033[31m               10th Argument : FLASH CHECK OPTION \033[0m"
  echo -e "\033[31m               11th Argument : BL RELEASE \033[0m"
  echo -e "\033[31m               12th Argument : FIPS KEY INCLUDED \033[0m"
  echo -e "\033[31m               13th Argument : MFG SW Version \033[0m"
  echo -e "\033[31m               14th Argument : ANTENNA TYPE \033[0m"
  exit 1
else
  cd ../release/flash/
  rm flash non_rf_values* pmem* -rf
  gcc -o flash rsi_calib_flash.c rsi_api_routine.c -lpthread
  S=$(date +%s)
  #	echo -e "Return value in sh file:$?"
  #	echo -e "sending $1 $2 $3 $4 $6 $7 $8 $9 arguments"
  status=`./flash 1 $1 $2 $3 $4 ${12} $7 $8 $9 ${13} ${14}| grep -i -m 1 "WARNING" | cut -d ' ' -f 1`
  if [ "$status" ==  "WARNING" ]
  then 
    echo -e "\033[31m !!!!! DANGER1 !!!!\033[0m"
    echo -e "\033[31m UNABLE TO CREATE FLASH FILe\033[0m"
    echo -e "\033[31m Upgrade Failed\033[0m"
    exit 1

  else
    echo -e "\033[35m Creating flash file...\033[0m"
  fi
#  gcc -o check_mac check_mac.c
#  status=`./check_mac`
#  if [ "$status" ==  "WARNING" ]
#  then 
#    echo -e "\033[31m !!!!! DANGER2 !!!!\033[0m"
#    echo -e "\033[31m DUP MAC Detected\033[0m"
#    echo -e "\033[31m Upgrade Failed\033[0m"
#    exit 1
#  else
#    echo -e "\033[35m Creating flash file...\033[0m"
#  fi

  cat RS9113_RS8111_calib_values.txt >> non_rf_values.txt
  sed -e '/0x.,/s/0x/0x0/g' non_rf_values.txt > non_rf_values2.txt 
  cp non_rf_values2.txt RS9113_RS8111_calib_values.txt
  cat WC/dump_zero.txt >> RS9113_RS8111_calib_values.txt 
  sed -e '4097,$d' RS9113_RS8111_calib_values.txt > non_rf_values3.txt

#  if [ "${11}" == 0 ]
#  then
#    cat WC/RS9113_APPEND_BL_HOST >> non_rf_values3.txt
#    touch -m WC/RS9113_APPEND_BL_HOST
#  else
#    cat ../../../../../release/flash/WC/RS9113_WC_BL_0_5_hex_8 >> non_rf_values3.txt
#    touch -m ../../../../../release/flash/WC/RS9113_WC_BL_0_5_hex_8
#  fi
#  echo -e "\033[31m Using File APPEND_BL_HOST\033[0m"

  cp non_rf_values3.txt RS9113_RS8111_calib_values.txt
  rm p1 non_rf_values* -rf
  gcc -o checksum checksum.c 
  status=`./checksum | grep -i -m 1 "WARNING" | cut -d ' ' -f 1`
  if [ "$status" ==  "WARNING" ]
  then 
    echo -e "\033[31m UNABLE TO APPEND CRC\033[0m"
    exit 1

  else
    echo -e "\033[31mAppending CRC...\033[0m"
  fi

  cp RS9113_RS8111_calib_values.txt ../ -rf
  cd ../
  exit
  S_v1=$(date +%s)
  if [ "${10}" == 1 ]
  then
    status=`./onebox_util rpine0 verify_flash 0 | grep -i -m 1 "Failed" | cut -d ' ' -f 2`
    E_v1=$(date +%s)
    dif_time=$((E_v1 - S_v1))
    echo -e "\033[35m Time elapsed for verify flash is $dif_time  Seconds \033[0m"
    if [ "$status" ==  "Failed" ]
    then 
      echo -e "\033[35m Already Flashed so erasing & Writing\033[0m"
      S_u=$(date +%s)
      sh upgrade.sh 1

      E_u=$(date +%s)
      diff_time=$((E_u - S_u))
      echo -e "\033[35m Time elapsed for Upgrade script is $diff_time  Seconds \033[0m"
    else
      echo -e "\033[35m Flash is clean,Ready to write\033[0m"
      S_u=$(date +%s)

      sh upgrade.sh 0
      #	if [ "$?" == 1 ]
      #	then 
      #		echo -e "\033[31m \t\tError returning from upgrade script\033[0m"
      #		exit 1
      #	fi



      E_u=$(date +%s)
      diff_time=$((E_u - S_u))
      echo -e "\033[35m \t\tTime elapsed for Upgrade script is $diff_time  Seconds \033[0m"
    fi
  else
    echo -e "\033[35m Not verifying Status of Flash, Assumption: Flash is Clean\033[0m"
    sh upgrade.sh 0
	exit
    E_u=$(date +%s)
    diff_time=$((E_u - S_u))
    echo -e "\033[35m Time elapsed for Upgrade script is $diff_time  Seconds \033[0m"

  fi
  MODE=`grep "DRIVER_MODE=" common_insert.sh | cut -d '=' -f 2| cut -d ' ' -f 2`

  if [ "$MODE" ==  "" ]
  then 
    echo -e "\033[31m !!!!! DANGER2 !!!!\033[0m"
    echo -e "\033[31m common_insert.sh file not found\033[0m"
    exit 1
  fi
  echo -e " \n\033[35m Verifying FLASH ....\033[0m"
  S_v=$(date +%s)

 # status=`./onebox_util rpine0 verify_flash 1 | grep -i -m 1 "Failed" | cut -d ' ' -f 2`
 # E_v=$(date +%s)
 # dif_time=$((E_v - S_v))
 # echo -e "\033[35m Time elapsed for verify flash is $dif_time  Seconds \033[0m"
  #sleep 2 
 # if [ "$status" ==  "Failed" ]
 # then 
 #   echo -e "\033[31m !!!!! DANGER !!!!\033[0m"
 #   echo -e "\033[31m Flash Verification Failed\033[0m"
 #   echo -e "\033[31m Check FLASH_CHECK flag in case of already calibrated module\033[0m"
 #   sed 's/='$MODE'/=3/' insert.sh > insert_temp.sh
 #   cp insert_temp.sh insert.sh
 #   rm insert_temp.sh -rf
 #   exit 2
 # else
 #   echo -e "\033[35m Upgrade Successful...\033[0m"
 #   cd  flash/
 #   #creating Log file
 #   #		echo -e " card_num : $5 "
 #   ./flash 4 $5 
 #   cd ../
 # fi

 # E=$(date +%s)
 # diff_time4=$((E - S))
 # echo -e "\033[35m Time elapsed for run_calib.sh is $diff_time4  Seconds \033[0m"

 # S_r=$(date +%s)
 # echo -e " \n\033[31m WLAN MAC_ID ....\033[0m"
 # ./onebox_util rpine0 eeprom_read 6 45
 # echo -e " \n\033[31m BT MAC_ID ....\033[0m"
 # ./onebox_util rpine0 eeprom_read 6 56
 # echo -e " \n\033[31m Zigbee MAC_ID ....\033[0m"
 # ./onebox_util rpine0 eeprom_read 8 67
 # echo -e " \n\033[31m MBR ....\033[0m"
 # ./onebox_util rpine0 eeprom_read 8 0
 # echo -e " \n\033[31m FLASH TYPE and size....\033[0m"
 # ./onebox_util rpine0 eeprom_read 4 20
 # echo -e " \n\033[31m EEPROM VERSION ....\033[0m"
 # ./onebox_util rpine0 eeprom_read 4 75
 # echo -e " \n\033[31m DPD POWER MAGIC WORD ....\033[0m"
 # ./onebox_util rpine0 eeprom_read 4 888
 # echo -e " \n\033[31m BB/RF CALIB MAGIC WORD....\033[0m"
 # ./onebox_util rpine0 eeprom_read 6 424
 # echo -e " \n\033[31m Bootloader Code ....\033[0m"
 # ./onebox_util rpine0 eeprom_read 6 4096
 # echo -e " \n\033[31m Boot descriptor ....\033[0m"
 # ./onebox_util rpine0 eeprom_read 6 65536
 # echo -e " \n\033[31m Channel PWR values ....\033[0m"
 # ./onebox_util rpine0 eeprom_read 8 1130
  #echo -e " \n\033[31m Bootloader Code1 ....\033[0m"
  #./onebox_util rpine0 eeprom_read 6 4096

  #echo -e " \n\033[31m BB TX_BCOS value1 ....\033[0m "
  #./onebox_util rpine0 bb_read 319
  #echo -e " \n\033[31m BB TX_BSIN value1 ....\033[0m"
  #./onebox_util rpine0 bb_read 31a
  #echo -e " \n\033[31m BB DPD 0x0F03 value1 ....\033[0m"
  #./onebox_util rpine0 bb_read f03
  #echo -e " \n\033[31m BB DPD 0x0F04 value1 ....\033[0m"
  #./onebox_util rpine0 bb_read f04
  #echo -e " \n\033[31m BB DPD 0x0F05 value1 ....\033[0m"
  #./onebox_util rpine0 bb_read f05
  #echo -e " \n\033[31m BB DPD 0x0F06 value1 ....\033[0m"
  #./onebox_util rpine0 bb_read f06
  #echo -e " \n\033[31m BB DPD 0x0F10 value1 ....\033[0m"
  #./onebox_util rpine0 bb_read f10
  #echo -e " \n\033[31m BB DPD 0x0F02 value1 ....\033[0m"
  #./onebox_util rpine0 bb_read f02

  echo -e "\033[31m Successfully Burnt the following...\033[0m"
  echo -e "\033[31m 1.Calibration Values\033[0m"
  echo -e "\033[31m 2.MAC ID\033[0m"
  sh remove_all.sh
  #		sed 's/=2/='$MODE'/' insert.sh > insert_temp.sh
 # sed 's/='$MODE'/=3/' insert.sh > insert_temp.sh
 # cp insert_temp.sh insert.sh
 # rm insert_temp.sh -rf
  E_r=$(date +%s)
  diff_time3=$((E_r - S_r))
  echo -e "\033[31m \t\tTime elapsed for eeprom_reads is $diff_time3  Seconds \033[0m"
  exit 0
  cd -
  cd -
fi

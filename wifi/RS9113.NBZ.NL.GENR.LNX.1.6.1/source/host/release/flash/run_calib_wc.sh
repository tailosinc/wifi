if [ "$1" = "" ] || [ "$2" = "" ] || [ "$3" = "" ] || [ "$4" = "" ] || [ "$5" = "" ] || [ "$6" = "" ] || [ "$7" = "" ] || [ "$8" = "" ] || [ "$9" = "" ] || [ "${10}" = "" ] || [ "${11}" = "" ] || [ "${12}" = "" ] || [ "${13}" = "" ] || [ "${14}" = "" ]
then
  echo -e "\033[31m       Error : 1st Argument : EEPROM VERSION\033[0m"
  echo -e "\033[31m               2nd Argument : EEPROM SIZE \033[0m"
  echo -e "\033[31m               3rd Argument : EEPROM TYPE \033[0m"
  echo -e "\033[31m               4th Argument : MODULE TYPE \033[0m"
  echo -e "\033[31m               5th Argument : MODULE NUMBER \033[0m"
  echo -e "\033[31m               6th Argument : RS9113_APPEND PROTOCOL 0/1-7:Nlink/Wise_connect\033[0m"
  echo -e "\033[31m               7th Argument : DIGITAL CHIP VERSION \033[0m"
  echo -e "\033[31m               8th Argument : MODULE VERSION \033[0m"
  echo -e "\033[31m               9th Argument : RF_CHIP VERSION \033[0m"
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

  S_wc=$(date +%s)

  #	echo -e "$1"
  #	echo -e "$2"
  #	echo -e "$3"
  #	echo -e "$4"
  #	echo -e "$5"
  #	echo -e "$6"
  #	echo -e "$7"
  #	echo -e "$8"
  #	echo -e "$9"
  status=`./flash 2 $1 $2 $3 $4 ${12} $7 $8 $9 ${13} ${14}| grep -i -m 1 "WARNING" | cut -d ' ' -f 1`
  if [ "$status" ==  "WARNING" ]
  then 
    echo -e "\033[31m !!!!! DANGER2 !!!!\033[0m"
    echo -e "\033[31m UNABLE TO CREATE FLASH FILe\033[0m"
    echo -e "\033[31m Upgrade Failed\033[0m"
    exit 1
  else
    echo -e "\033[35m Creating flash file...\033[0m"
  fi
  gcc -o check_mac check_mac.c
  status=`./check_mac`
  if [ "$status" ==  "WARNING" ]
  then 
    echo -e "\033[31m !!!!! DANGER2 !!!!\033[0m"
    echo -e "\033[31m DUP MAC Detected\033[0m"
    echo -e "\033[31m Upgrade Failed\033[0m"
    exit 1
  else
    echo -e "\033[35m Creating flash file...\033[0m"
  fi
  cat RS9113_RS8111_calib_values.txt >> non_rf_values.txt
  sed -e '/0x.,/s/0x/0x0/g' non_rf_values.txt > non_rf_values2.txt 
  cp non_rf_values2.txt RS9113_RS8111_calib_values.txt
  cat WC/dump_zero.txt >> RS9113_RS8111_calib_values.txt 
  sed -e '4097,$d' RS9113_RS8111_calib_values.txt > non_rf_values3.txt

  if [ "$6" == 1 ]
  then
    cat WC/RS9113_APPEND_FILE_WBZ >> non_rf_values3.txt
    touch -m WC/RS9113_APPEND_FILE_WBZ	
    echo -e "\033[35m Using File _WBZ\033[0m"
  elif [ "$6" == 2 ]
  then
    cat WC/RS9113_APPEND_FILE_W >> non_rf_values3.txt 
    touch -m WC/RS9113_APPEND_FILE_W	
    echo -e "\033[35m Using File _W\033[0m"
  elif [ "$6" == 3 ]
  then
    cat WC/RS9113_APPEND_FILE_B >> non_rf_values3.txt 
    touch -m WC/RS9113_APPEND_FILE_B	
    echo -e "\033[35m Using File _B\033[0m"
  elif [ "$6" == 4 ]
  then
    cat WC/RS9113_APPEND_FILE_Z >> non_rf_values3.txt 
    touch -m WC/RS9113_APPEND_FILE_Z	
    echo -e "\033[35m Using File _Z\033[0m"
  elif [ "$6" == 5 ] && [ "${12}" == 3 ]
  then
    cat WC/RS9113_APPEND_FILE_WB >> non_rf_values3.txt 
    touch -m WC/RS9113_APPEND_FILE_WB	
    echo -e "\033[35m Using File _WB\\033[0m"
  elif [ "$6" == 5 ] && [ "${12}" == 4 ]
  then
    cat WC/RS9113_APPEND_FILE_WB >> non_rf_values3.txt 
    touch -m WC/RS9113_APPEND_FILE_WB	
    echo -e "\033[35m Using File _WB\\033[0m"
  elif [ "$6" == 5 ] && [ "${12}" == 0 ]
  then
    cat WC/RS9113_APPEND_FILE_WB >> non_rf_values3.txt 
    touch -m WC/RS9113_APPEND_FILE_WB	
    echo -e "\033[35m Using File _WB\\033[0m"
  elif [ "$6" == 5 ] && [ "${12}" == 1 ]
  then
    cat WC/RS9113_APPEND_FILE_WB_DRG >> non_rf_values3.txt 
    touch -m WC/RS9113_APPEND_FILE_WB_DRG	
    echo -e "\033[35m Using File _WB_DRG\\033[0m"
  elif [ "$6" == 5 ] && [ "${12}" == 2 ]
  then
    cat WC/RS9113_APPEND_FILE_WB_FIPS >> non_rf_values3.txt 
    touch -m WC/RS9113_APPEND_FILE_WB_FIPS	
    echo -e "\033[35m Using File _WB_FIPS\\033[0m"
  elif [ "$6" == 6 ]
  then
    cat WC/RS9113_APPEND_FILE_BZ >> non_rf_values3.txt 
    touch -m WC/RS9113_APPEND_FILE_BZ	
    echo -e "\033[35m Using File _BZ\033[0m"
  elif [ "$6" == 7 ]
  then
    cat WC/RS9113_APPEND_FILE_WZ >> non_rf_values3.txt 
    touch -m WC/RS9113_APPEND_FILE_WZ	
    echo -e "\033[35m Using File _WZ\033[0m"
  else
    echo -e "Invalid argument for RS9113_append"
    exit 1
  fi
  
  cp non_rf_values3.txt RS9113_RS8111_calib_values.txt
  if [ "${12}" == 1 ]
  then
	  echo -e "\033[35m APPENDING FIPS\033[0m"
	  status=` ./flash append_fips | grep -i -m 1 "WARNING" | cut -d ' ' -f 1`
	  if [ "$status" ==  "WARNING" ]
	  then 
		  echo -e "\033[31m !!!!! DANGER2 !!!!\033[0m"
		  echo -e "\033[31m UNABLE TO APPEND FIPS KEYS\033[0m"
		  echo -e "\033[31m Upgrade Failed\033[0m"
		  exit 1
	  else
		  echo -e "\033[35m INCLUDING FIPS KEY\033[0m"
	  fi
  elif [ "${12}" == 2 ]
  then
	  echo -e "\033[35m NO FIPS KEY\033[0m"
  fi


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

  S_v1=$(date +%s)
  if [ "${10}" == 1 ]
  then
    status=`./onebox_util rpine0 verify_flash 0 | grep -i -m 1 "Failed" | cut -d ' ' -f 2`
    E_v=$(date +%s)
    dif_time=$((E_v - S_v1))
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
    echo -e "\033[35m Not verifying Status of Flash, Assumtion: Flash is Clean\033[0m"
    sh upgrade.sh 0

    E_u=$(date +%s)
    diff_time=$((E_u - S_u))
    echo -e "\033[35m Time elapsed for Upgrade script is $diff_time  Seconds \033[0m"
  fi
  MODE=`grep "DRIVER_MODE=" insert.sh | cut -d '=' -f 2| cut -d ' ' -f 2`

  if [ "$MODE" ==  "" ]
  then 
    echo -e "\033[31m !!!!! DANGER2 !!!!\033[0m"
    echo -e "\033[31m Insert.sh file not found\033[0m"
    exit 1
  fi

  echo -e " \n\033[35m Verifying FLASH ....\033[0m"
  S_v=$(date +%s)

  status=`./onebox_util rpine0 verify_flash 1 | grep -i -m 1 "Failed" | cut -d ' ' -f 2`
  E_v=$(date +%s)
  dif_time=$((E_v - S_v))
  echo -e "\033[35m Time elapsed for verify flash is $dif_time  Seconds \033[0m"
  if [ "$status" ==  "Failed" ]
  then 
    echo -e "\033[31m !!!!! DANGER !!!!\033[0m"
    echo -e "\033[31m Flash Verification Failed\033[0m"
    echo -e "\033[31m Check FLASH_CHECK flag in case of already calibrated module\033[0m"
    sed 's/='$MODE'/=3/' insert.sh > insert_temp.sh
    cp insert_temp.sh insert.sh
    rm insert_temp.sh -rf
    exit 2
  else
    echo -e "\033[35mUpgrade Successful...\033[0m"
    cd  flash/
    #creating Log file
    #echo "card_no_w:$5"
    ./flash 5 $5
    cd ../
  fi

  E_wc=$(date +%s)
  diff_time2=$((E_wc - S_wc))
  echo -e "\033[35m Time elapsed for run_calib_wc.sh is $diff_time2  Seconds \033[0m"

  S_r=$(date +%s)
  echo -e " \n\033[31m WLAN MAC_ID ....\033[0m"
  ./onebox_util rpine0 eeprom_read 6 45
  echo -e " \n\033[31m BT MAC_ID ....\033[0m"
  ./onebox_util rpine0 eeprom_read 6 56
  echo -e " \n\033[31m Zigbee MAC_ID ....\033[0m"
  ./onebox_util rpine0 eeprom_read 8 67
  echo -e " \n\033[31m MBR ....\033[0m"
  ./onebox_util rpine0 eeprom_read 16 0
  echo -e " \n\033[31m FLASH TYPE and SIZE....\033[0m"
  ./onebox_util rpine0 eeprom_read 4 20
  echo -e " \n\033[31m EEPROM VERSION ....\033[0m"
  ./onebox_util rpine0 eeprom_read 4 75
  echo -e " \n\033[31m DPD POWER MAGIC WORD ....\033[0m"
  ./onebox_util rpine0 eeprom_read 4 888
  echo -e " \n\033[31m BB/RF CALIB MAGIC WORD2....\033[0m"
  ./onebox_util rpine0 eeprom_read 6 424
  echo -e " \n\033[31m Bootloader Code ....\033[0m"
  ./onebox_util rpine0 eeprom_read 6 4096
  echo -e " \n\033[31m WLAN FW Code ....\033[0m"
  ./onebox_util rpine0 eeprom_read 6 69632
  echo -e " \n\033[31m BT FW Code ....\033[0m"
  ./onebox_util rpine0 eeprom_read 6 1048576
  echo -e " \n\033[31m Boot descriptor ....\033[0m"
  ./onebox_util rpine0 eeprom_read 6 65536
  echo -e " \n\033[31m Channel PWR values ....\033[0m"
  ./onebox_util rpine0 eeprom_read 8 1130

  #echo -e " \n\033[31m BB TX_BCOS value2 ....\033[0m "
  #./onebox_util rpine0 bb_read 319
  #echo -e " \n\033[31m BB TX_BSIN value2 ....\033[0m"
  #./onebox_util rpine0 bb_read 31a
  #echo -e " \n\033[31m BB DPD 0x0F03 value2 ....\033[0m"
  #./onebox_util rpine0 bb_read f03
  #echo -e " \n\033[31m BB DPD 0x0F04 value2 ....\033[0m"
  #./onebox_util rpine0 bb_read f04
  #echo -e " \n\033[31m BB DPD 0x0F05 value2 ....\033[0m"
  #./onebox_util rpine0 bb_read f05
  #echo -e " \n\033[31m BB DPD 0x0F06 value2 ....\033[0m"
  #./onebox_util rpine0 bb_read f06
  #echo -e " \n\033[31m BB DPD 0x0F10 value2 ....\033[0m"
  #./onebox_util rpine0 bb_read f10
  #echo -e " \n\033[31m BB DPD 0x0F02 value2 ....\033[0m"
  #./onebox_util rpine0 bb_read f02
  echo -e "\033[31m Successfully Burnt the following...\033[0m"
  echo -e "\033[31m 1.Calibration Values\033[0m"
  echo -e "\033[31m 2.MAC ID\033[0m"
  echo -e "\033[31m 3.Boot Loader\033[0m"
  echo -e "\033[31m 4.WLAN and BT Firmware\033[0m"
  sh /home/rsi/release/remove.sh
  sed 's/='$MODE'/=3/' insert.sh > insert_temp.sh
  cp insert_temp.sh insert.sh
  rm insert_temp.sh -rf
  E_r=$(date +%s)
  diff_time3=$((E_r - S_r))
  echo -e "\033[31m \t\tTime elapsed for eeprom_reads is $diff_time3  Seconds \033[0m"
  exit 0
  cd -
  cd -
fi

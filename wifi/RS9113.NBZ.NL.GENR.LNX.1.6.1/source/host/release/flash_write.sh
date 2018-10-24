cmd=`lsmod | grep onebox`
if [ "$cmd" ]; then
echo "onebox modules are already inserted, doing rmmod";
sh remove_all.sh
sleep 5
fi
dmesg -c
echo "doing insmod for writing to flash";
sh onebox_insert.sh 1

WLAN=1
./onebox_util rpine0 enable_protocol $WLAN

sleep 2
str=`dmesg | grep "CARD READY RECEIVED FROM WLAN FIRMWARE"`
if [ "$str" ]
then
echo "Writing Values to Flash"
./append_pwr_values
else
echo "Driver is not inserted properly. Hence not writing to flash"
fi

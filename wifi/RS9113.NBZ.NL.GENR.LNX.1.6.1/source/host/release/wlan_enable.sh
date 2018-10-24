cmd=`lsmod | grep onebox`
if [ "$cmd" ]; then
echo "onebox modules are already inserted";
else
sh onebox_insert.sh
fi

WLAN=1
./onebox_util rpine0 enable_protocol $WLAN

cmd=`lsmod | grep onebox`
if [ "$cmd" ]; then
echo "onebox modules are already inserted";
else
sh onebox_insert.sh
fi

ZIGB=4
./onebox_util rpine0 enable_protocol $ZIGB

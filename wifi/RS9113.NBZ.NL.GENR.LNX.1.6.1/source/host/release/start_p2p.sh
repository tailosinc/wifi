
cat /dev/null > /var/log/messages
sh wlan_enable.sh
./onebox_util rpine0 create_vap wifi0 p2p
./wpa_supplicant -i wifi0 -D bsd -c p2p.conf -ddddt > log &

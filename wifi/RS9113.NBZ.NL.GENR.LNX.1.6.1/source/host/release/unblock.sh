#rfkill list | grep hci1 | cut -f1 -d: | xargs rfkill unblock
#sleep 0.2
#hciconfig hci1 up
#sleep 0.2
hciconfig hci0 inqparms 2048:4096
sleep 0.1
hciconfig hci0 pageparms 2048:4096
sleep 0.1
hciconfig hci0 piscan

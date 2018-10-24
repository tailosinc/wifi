#!/bin/bash

echo " ===== Starting wifi container ===== "

# Create symlinks, copy files
pushd wifi/RS9113.NBZ.NL.GENR.LNX.1.6.1/source/host/release
cp -r firmware/ /data/
mkdir -p /mnt/data/docker/volumes/${RESIN_APP_ID}_resin-data/_data
ln -s /data/firmware /mnt/data/docker/volumes/${RESIN_APP_ID}_resin-data/_data

# Load the wifi driver
modprobe uinput
modprobe cfg80211
modprobe bluetooth
sh start_sta.sh
popd

sleep infinity

rfkill list | grep hci0 | cut -f1 -d: | xargs rfkill block

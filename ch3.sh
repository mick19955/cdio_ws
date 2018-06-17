SSID=`grep ssid_single_player /data/config.ini | awk -F "=" '{print $2}'`
ifconfig ath0 down
iwconfig ath0 mode master essid $SSID channel 3
iwconfig ath0 commit
ifconfig ath0 192.168.1.1 netmask 255.255.255.0 up

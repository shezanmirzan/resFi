#!/bin/bash


if [ $# -eq 0 ]
  then
    echo "No arguments supplied; usage: $0 <phy>"
    exit 0
fi
if [ $# -eq 1 ]
  then
    echo "No arguments supplied for spectral scan; usage: $0 <phy>"
    exit 0
fi


phy=$1

# cleanup
for i in `seq 0 10`;
do
  sudo iw dev mon${i} del 2>/dev/null
  sudo iw dev wlan${i} del 2>/dev/null
  sudo iw dev wifi${i} del 2>/dev/null
  sudo iw dev ap${i} del 2>/dev/null
done

sudo rfkill unblock all 2>/dev/null

sudo iw phy $2 interface add wlan0 type managed
sudo iw phy $2 interface add mon0 type monitor
sudo ifconfig mon0 up 
sudo ifconfig wlan0 up


#Configuring AP
sleep 1
sudo killall -9 hostapd 2> /dev/null
sleep 1
sudo iw phy ${phy} interface add ap5 type monitor
sleep 1
sudo ifconfig ap5 192.168.6.1 netmask 255.255.255.0
sleep 1
sudo service network-manager stop /dev/null
sleep 1
sudo ./hostapd-20131120/hostapd/hostapd hostapd-20131120/hostapd/hostapd-ch40.conf &
sleep 1
echo chanscan > /sys/kernel/debug/ieee80211/phy0/ath9k/spectral_scan_ctl
sudo iw dev wlan0 scan 2>/dev/null
sleep 5
#Starting ResFi Agent
cd framework/
sudo python resfi_loader.py


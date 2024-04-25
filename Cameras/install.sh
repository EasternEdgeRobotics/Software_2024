#!/bin/bash
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y python3-picamera2 python3-libcamera git make
git clone https://github.com/roamingthings/spyglass
cd spyglass
make install
mkdir -p ~/printer_data/config
cp spyglass.conf ~/printer_data/config/spyglass.conf
echo "all done, make sure to reboot"
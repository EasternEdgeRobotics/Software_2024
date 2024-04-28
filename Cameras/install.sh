#!/bin/bash
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y python3-picamera2 python3-libcamera git make

# Clone and make git repo
git clone https://github.com/roamingthings/spyglass

# Copy the spyglass configuration such that it is seen by the spyglass application
mkdir -p ~/printer_data/config
sudo cp spyglass.conf ~/printer_data/config/spyglass.conf

# Install the spyglass
cd spyglass
make install
cd ..

# Create the service
echo "
[Unit]
Description=Spyglass Camera Streamer
After=network.target

[Service]
Type=simple
Restart=always
User=$USER
ExecStart=python3 $PWD/spyglass/run.py -f 60 -r "1280x720"

[Install]
WantedBy=multi-user.target
" | sudo tee /etc/systemd/system/spyglass.service
sudo systemd daemon-reload
sudo systemctl enable --now spyglass

echo "all done."

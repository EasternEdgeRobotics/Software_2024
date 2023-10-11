#!/bin/bash
sudo raspi-config nonint do_legacy 0
sudo apt update
sudo apt install -y cmake libjpeg-dev gcc g++
sudo cp eer-camera.service /etc/systemd/system/eer-camera.service
cd mjpg-streamer/mjpg-streamer-experimental
make
sudo make install
sudo systemctl enable --now eer-camera
echo "########################"
echo " Installation complete!"
echo "########################"
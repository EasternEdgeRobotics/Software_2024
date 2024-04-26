sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y python3-picamera2 python3-libcamera git make
git clone https://github.com/roamingthings/spyglass
cd spyglass
mkdir -p ~/printer_data/config
make install
cd ..
sudo cp spyglass.conf ~/printer_data/config/spyglass.conf
sudo cp spyglass.service /etc/systemd/system/spyglass.service
sudo systemd daemon-reload
sudo systemctl enable --now spyglass
echo "all done, make sure to reboot"

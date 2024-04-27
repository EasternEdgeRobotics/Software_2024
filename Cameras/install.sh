sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y python3-picamera2 python3-libcamera git make
git clone https://github.com/roamingthings/spyglass
sudo cp spyglass.conf ~/printer_data/config/spyglass.conf
cd spyglass
mkdir -p ~/printer_data/config
echo "
[Unit]
Description=Spyglass Camera Streamer
After=network.target

[Service]
Type=simple
Restart=always
User=$USER
ExecStart=python3 $PWD/run.py -f 60 -r "1280x720"

[Install]
WantedBy=multi-user.target
" | sudo tee /etc/systemd/system/spyglass.service
sudo systemd daemon-reload
sudo systemctl enable --now spyglass
echo "all done, make sure to reboot"

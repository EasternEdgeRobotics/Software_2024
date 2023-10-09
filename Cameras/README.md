# Eastern Edge Cameras
This iteration of cameras uses the mjpg-streamer program to stream mjpg to the browser with minimal (90-110ms) latency.
## Installation
### Run the install script:
Script may be missing dependencies (I haven't tested it on a clean raspberry pi install)
```
bash install-cameras.sh
```
### Camera should be available at:
```
http://<YOUR-PIS-IP>:8080/?action=stream
```
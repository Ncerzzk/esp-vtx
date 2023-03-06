# ESP-VTX
Digital,low latency video transmitter base on ESP32
origin project:https://github.com/jeanlemotan/esp32-cam-fpv

## Features:
It uses FEC encoding (2/3 currently configured) with 1400 byte packets and it achieves quite good performance:
* Up to 54Mbps video rate.
* More than 80 FPS at 320x240 or lower with 20-50 ms latency.
* 30+ FPS at 800x600 or 640x480 with 50-80 ms latency.
* 12 FPS at 1024x768 with >100 ms latency.
* run at any ESP-cam boards(like AI Thinker etc.) with OV2640.

## Theory
The data is received from the camera module as JPEG ar 20MHz I2S clock and passed directly to the wifi module and written to the SD card if the DVR is enabled.
The ESP camera component was modified to send the data as it's received from the DMA instead of frame-by-frame basis. This decreases latency quite significantly (10-20 ms) and reduces the need to allocate full frames in PSRAM.

The wifi data is send using packet injection with configurable rate - from 2 MB CCK to 72MB MCS7 - and power.

The air unit can also record the video straight from the camera to a sd card. The format is a rudimentary MJPEG without any header so when playing back the FPS will be whatever your player will decide.\
There is significant buffering when writing to SD (3MB at the moment) to work around the very regular slowdowns of sd cards.


The receiver is a Raspberry PI 4 with 2 wifi adapters in monitor mode (TL-WN722N). The adapters work as diversity receivers and the data is reconstructed from the FEC packets.

The JPEG decoding is done with turbojpeg to lower latency and - based on the resolution - can take between 1 and 7 milliseconds.\
It's then uploaded to 3 separate textures as YUV and converted to RGB in a shader.

The link is bi-directional so the ground station can send data to the air unit. At the moment it sends camera and wifi configuration data but I plan to have a full bi-directional serial port for telemetry coming soon.\
The back link uses 64byte packets with a 2/6 FEC encoding (so quite solid) at a low wifi rate (I think 1Mb).

This is very WIP at the moment, but it proves that the ESP32 can definitely stream video with very low latency. \
I plan to use this is a long range micro quad.

Here is a video shot at 30 FPS at 800x600 (video converted from the source mjpeg):

https://user-images.githubusercontent.com/10252034/116135308-43c08c00-a6d1-11eb-99b9-9dbb106b6489.mp4

## Building
### esp32 firmware:
- Uses esp-idf-v4.3-beta1, can probably work with newer idf versions. It needs to be properly installed (follow the instructions in the IDF documentation)
- Only the Ai Thinker esp cam board tested
- In the air_firmware, execute this: `idf.py -p /dev/tty.usbserial1 flash monitor`. Replace `tty.usbserial1` with your serial port.
- Make sure you place the board in flashing mode bu connecting IO0 to GND and resetting the board.
- After compiling and resetting, you should get some stats per second, smth like this in the console:\
`WLAN S: 695196, R: 350, E: 0, D: 0, % : 0 || FPS: 64, D: 401268 || D: 0, E: 0`\
`WLAN S: 761616, R: 420, E: 0, D: 0, % : 0 || FPS: 69, D: 443679 || D: 0, E: 0`\
`WLAN S: 763092, R: 420, E: 0, D: 0, % : 0 || FPS: 69, D: 449410 || D: 0, E: 0`\
`WLAN S: 764568, R: 420, E: 0, D: 0, % : 0 || FPS: 69, D: 450996 || D: 0, E: 0`\
`WLAN S: 761616, R: 350, E: 0, D: 0, % : 0 || FPS: 69, D: 449347 || D: 0, E: 0`

### Ground Station:
- Most Pi with hdmi(or other display) should work
- At least one wifi adapter
- Dependencies:
	`sudo apt install libdrm-dev libgbm-dev libgles2-mesa-dev libpcap-dev libturbojpeg0-dev libts-dev libsdl2-dev libfreetype6-dev `
- In the gs folder, execute `make`
- Run `sudo -E DISPLAY=:0 ./gs`

The GS can run both with X11 and without. However, to run it without GS you need to compile SDL2 yourself to add support for kmsdrm:
`git clone https://github.com/libsdl-org/SDL.git`\
`cd SDL`\
`mkdir build`\
`cd build`\
`../configure --disable-video-rpi --enable-video-kmsdrm --enable-video-x11 --disable-video-opengl`\
`make -j5`\
`sudo make install`

To run without X11 using the just compiled SDL2, do this:\
`sudo -E LD_LIBRARY_PATH=/usr/local/lib DISPLAY=:0 ./gs`

Some other things that should improve latency(raspberry):
- Disable the compositor from raspi-config. This should increase FPS
- Change from fake kms to real kms in config.txt: dtoverlay=vc4-fkms-v3d to dtoverlay=vc4-kms-v3d

VSync is disabled and on a PI4 you should get > 200FPS if everything went ok and you have configured the PI correctly.

### Trouble Shout
- how to check wether the wifi card or os could support monitor mode?
```
sudo apt get install tcpdump
sudo tcpdump --monitor-mode -i INTERFACE_NAME
```
if failed , try https://github.com/lennartkoopmann/nzyme/discussions/339#discussioncomment-521229 
if still failed, then your wifi card(may be driver issue) doesn't support monitor mode.

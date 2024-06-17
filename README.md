# RoboCup Soccer Open, Gen 2

## Setup

### Clone

Clone this repository in the home directory.

### Enable SSH and UART

Using `sudo raspi-config`

### Install packages and libraries

```
sudo apt install python3-pip git i2c-tools python3-opencv python3-serial python3-websockets
```

```
pip3 install bluedot
```

### Set cameras and optionally disable builtin Bluetooth

Add following lines to `/boot/firmware/config.txt`, change cameras if needed:

```
# Automatically load overlays for detected cameras
camera_auto_detect=0
dtoverlay=imx708,rotation=0,cam0
dtoverlay=imx477,cam1

# Uncomment to disable builtin bluetooth - custom
#dtoverlay=disable-bt
```

### Add launcher to autostart

Run `Launcher/apply_service.sh`.

### Pairing robots

Pair using [bluedot manual](https://bluedot.readthedocs.io/en/latest/pairpipi.html).

### Camera

Obtain frames using [Picamera2](https://github.com/raspberrypi/picamera2).

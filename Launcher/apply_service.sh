#!/bin/sh

cp robot-launcher.service /lib/systemd/system/robot-launcher.service
sudo chmod 644 /lib/systemd/system/robot-launcher.service

sudo systemctl daemon-reload
sudo systemctl enable robot-launcher.service

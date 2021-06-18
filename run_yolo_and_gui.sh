#!/bin/sh
cd /home/ur5e/gui/
python3 main.py&
sleep 1
cd /home/ur5e/darknet
python darknet_ros.py

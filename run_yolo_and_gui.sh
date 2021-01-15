#!/bin/sh
cd /home/qwe/gui/
python3 main.py&
sleep 1
cd /home/qwe/darknet-master
python darknet_ros.py
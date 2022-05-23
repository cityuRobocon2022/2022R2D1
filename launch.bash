#!/bin/bash

gnome-terminal -- bash -c '
  echo "[INFO] Running joy node...";
  source /opt/ros/galactic/setup.bash;
  ros2 run joy joy_node;
  '

gnome-terminal -- bash -c '
  echo "[INFO] Running carbase node...";
  source ~/carbase_ws/install/setup.bash;
  ros2 run carbase_package carbase_node
'

gnome-terminal -- bash -c '
  echo 'admin' | sudo -S chmod 666 /dev/ttyACM*;
  MicroXRCEAgent serial --dev /dev/ttyACM0 -b 115200 -v6;
  '
gnome-terminal -- bash -c '
  MicroXRCEAgent serial --dev /dev/ttyACM1 -b 115200 -v6;
  '

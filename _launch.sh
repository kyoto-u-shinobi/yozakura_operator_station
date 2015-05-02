#!/bin/bash
tab="--tab-with-profile=yozakura -e "
window="--window-with-profile=yozakura -e "

pkill rosmaster
pkill rosout

roscore &
sleep 5

gnome-terminal \
  $tab ./1_main.bash -t "ROS Main"\
  $tab ./2_theta.bash -t "Theta Server"\
  $tab ./3-1_aiball_overview.bash -t "Overview Camera"\
  $tab ./3-2_aiball_front.bash -t "Front Camera"\
  $tab ./3-3_aiball_back.bash -t "Back Camera"\
  $tab ./3-4_aiball_arm.bash -t "Arm Camera"

gnome-terminal $window ./6_rpi.bash -t "RPi SSH"

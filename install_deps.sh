#!/usr/bin/env bash

case "$(lsb_release -si)" in
"Arch")
	yay -S --asdeps ros-noetic-effort-controllers ros-noetic-moveit ros-noetic-ompl qt4 libspnav
	;;
"Ubuntu")
	sudo apt install -y gazebo7 ros-kinetic-qt-build ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control ros-kinetic-control-toolbox ros-kinetic-realtime-tools ros-kinetic-ros-controllers ros-kinetic-xacro python-wstool ros-kinetic-tf-conversions ros-kinetic-kdl-parser
	;;
esac

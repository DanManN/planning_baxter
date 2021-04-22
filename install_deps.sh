#!/usr/bin/env bash

case "$(lsb_release -si)" in
"Arch")
	yay -S --asdeps ros-noetic-effort-controllers ros-noetic-moveit ros-noetic-ompl qt4 libspnav
	;;
"Ubuntu")
	sudo apt install -y ros-noetic-effort-controllers ros-noetic-moveit ros-noetic-ompl ros-noetic-moveit-visual-tools ros-noetic-rosparam-shortcuts qt4-default libspnav0
	;;
esac

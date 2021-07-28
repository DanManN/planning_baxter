#!/usr/bin/env bash

case "$(lsb_release -si)" in
"Arch")
	yay -S --asdeps ros-noetic-effort-controllers ros-noetic-moveit ros-noetic-ompl ros-noetic-moveit-visual-tools ros-noetic-rosparam-shortcuts qt4 libspnav
	;;
"Ubuntu")
	sudo add-apt-repository -y ppa:rock-core/qt4
	sudo apt install -y ros-noetic-effort-controllers ros-noetic-moveit ros-noetic-ompl ros-noetic-moveit-visual-tools ros-noetic-rosparam-shortcuts qt4-default libspnav0 python3-empy python3-collada
	;;
esac

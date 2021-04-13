all: ./src/baxter ./src/baxter_common ./src/baxter_tools ./src/baxter_examples ./src/baxter_interface ./src/baxter_simulator ./src/sim_ros_interface ./src/timed_roslaunch #./src/moveit_robots

./src/baxter:
	git clone https://github.com/RethinkRobotics/baxter.git $@

./src/baxter_common:
	git clone https://github.com/RethinkRobotics/baxter_common.git $@

./src/baxter_tools:
	git clone https://github.com/DanManN/baxter_tools.git $@

./src/baxter_examples:
	git clone https://github.com/DanManN/baxter_examples.git $@

./src/baxter_interface:
	git clone https://github.com/DanManN/baxter_interface.git $@

./src/baxter_simulator:
	git clone https://github.com/DanManN/baxter_simulator.git $@

./src/sim_ros_interface:
	git clone --branch coppeliasim-v4.1.0 --recursive https://github.com/CoppeliaRobotics/simExtROSInterface.git $@

./src/timed_roslaunch:
	git clone https://github.com/MoriKen254/timed_roslaunch.git $@

# ./src/moveit_robots:
#         https://github.com/ros-planning/moveit_robots.git $@

deps:
	./install_deps.sh

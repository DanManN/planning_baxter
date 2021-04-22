all: ./src/baxter ./src/baxter_common ./src/baxter_tools ./src/baxter_examples ./src/baxter_interface ./src/baxter_simulator ./src/moveit_resources ./src/moveit_grasps # ./src/timed_roslaunch ./src/sim_ros_interface ./src/moveit_robots

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

./src/moveit_resources:
	git clone https://github.com/ros-planning/moveit_resources.git $@

./src/moveit_grasps:
	git clone https://github.com/ros-planning/moveit_grasps.git $@

# ./src/timed_roslaunch:
#         git clone https://github.com/MoriKen254/timed_roslaunch.git $@

# ./src/sim_ros_interface:
#         git clone --branch coppeliasim-v4.1.0 --recursive https://github.com/CoppeliaRobotics/simExtROSInterface.git $@

# ./src/moveit_robots:
#         https://github.com/ros-planning/moveit_robots.git $@

deps:
	./install_deps.sh

clean-src:
	rm -rf ./src/baxter ./src/baxter_common ./src/baxter_tools ./src/baxter_examples ./src/baxter_interface ./src/baxter_simulator ./src/sim_ros_interface ./src/moveit_resources ./src/timed_roslaunch

clean:
	rm -rf build devel

clean-all: clean clean-src

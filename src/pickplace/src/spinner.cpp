#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "misc_spinner");
	ros::AsyncSpinner spinner(0);
	spinner.start();
	ros::waitForShutdown();
}

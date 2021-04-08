#!/usr/bin/env bash
rosrun baxter_tools enable_robot.py -e
sleep 1
rosrun baxter_interface joint_trajectory_action_server.py

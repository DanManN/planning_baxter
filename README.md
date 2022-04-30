# The tutorial for the Aggregation
Perception-Planning-Execution framework for baxter.

## outline:
1. connect to the Internet
2. setup baxter robot
3. setup workspace
4. setup camera
5. run pipeline

## connect to the Internet
connect the ethernet cable


## set up baxter robot
open a terminal
terminal 1
``` console
cd /home/pracsys/retrieval/Kai/Aggregation
roscore
```
Turn on the baxter robot. wait until the head light turns green.\
Open new terminal tab.\
terminal 2:
``` consule
source connect_baxter
enable
quiet
```
Open new terminal tab.\
terminal 3:
``` consule
source real_robot_commands/1_open_moveit
```
Open new terminal tab.\
terminal 4:
``` consule
source real_robot_commands/move_arm
```
## Set up workspace
Set up stages.\
Set up table board.\
Set up bricks so that tag 108 is under the finger tip.\
<pre>
108              107\
|                  |\
105--------------109\
</pre>
Move tip above tag 105.\
terminal 4
``` consule
source real_robot_commands/get_arm_position
```
Copy the output x,y numbers to Aggregation/constants.py.\
Repeat for tag 109 107.

## Set up camera
Insert camera cable at USB 3.0
terminal 4
``` consule
realsense-viewer
```
Set up the camera to make sure four tags are all covered in the 1920*1080 frame

## Run the pipeline
terminal 4
``` consule
source pipeline
```

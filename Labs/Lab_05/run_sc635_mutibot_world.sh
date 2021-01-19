#!/bin/bash  

killall -9 gzserver
killall -9 gzclient
killall -9 rosmaster

cd ./mybot_ws
source ./devel/setup.bash
catkin_make       	
roslaunch mybot_gazebo mybot_world.launch &
sleep 5
rosrun swarm_ctrl swarm_ctrl.py

done

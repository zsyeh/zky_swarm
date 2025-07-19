#!/bin/bash
sleep 2s
gnome-terminal -x bash -c "cd ~/livox_ws&&source devel/setup.bash&&roslaunch livox_ros_driver2 msg_MID360.launch" & sleep 8
gnome-terminal -x bash -c "cd ~/livox_ws&&source devel/setup.bash&&roslaunch fast_lio mapping_mid360.launch" & sleep 5
gnome-terminal -x bash -c "source /opt/ros/noetic/setup.bash&&roslaunch mavros px4.launch" & sleep 15
gnome-terminal -x bash -c "cd ~/zky_swarm_real&&source devel/setup.bash&&rosrun ego_planner vins_to_mavros" & sleep 3
gnome-terminal -x bash -c "cd ~/zky_swarm_real&&source devel/setup.bash&&roslaunch ego_planner swarm.launch" & sleep 5
gnome-terminal -x bash -c "cd ~/zky_swarm_real&&source devel/setup.bash&&roslaunch swarm_ros_bridge gcs_bridge.launch"


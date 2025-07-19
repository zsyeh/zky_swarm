source livox_ws/devel/setup.bash
nohup roslaunch livox_ros_driver2 msg_MID360.launch  & sleep 2
roslaunch fast_lio mapping_mid360.launch

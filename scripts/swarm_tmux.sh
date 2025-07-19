#!/bin/bash

# 定义 tmux 会话名称
SESSION_NAME="ros_swarm"

# 检查会话是否存在，如果存在则杀死它
tmux has-session -t $SESSION_NAME 2>/dev/null
if [ $? == 0 ]; then
    echo "发现已存在的 tmux 会话 '$SESSION_NAME'，正在终止..."
    tmux kill-session -t $SESSION_NAME
fi

echo "正在启动新的 tmux 会话 '$SESSION_NAME'..."

# 启动 tmux 会话，并在第一个窗口 (window 0) 运行 livox_ros_driver2
# -d: 分离模式，在后台创建
# -s: 指定会话名称
# -n: 指定窗口名称
tmux new-session -d -s $SESSION_NAME -n "livox_driver" 'cd ~/livox_ws && source devel/setup.bash && roslaunch livox_ros_driver2 msg_MID360.launch'

# 等待
sleep 8

# 创建新窗口 (window 1) 并运行 fast_lio
tmux new-window -t $SESSION_NAME:1 -n "fast_lio" 'cd ~/livox_ws && source devel/setup.bash && roslaunch fast_lio mapping_mid360.launch'

# 等待
sleep 5

# 创建新窗口 (window 2) 并运行 mavros
tmux new-window -t $SESSION_NAME:2 -n "mavros" 'source /opt/ros/noetic/setup.bash && roslaunch mavros px4.launch'

# 等待
sleep 15

# 创建新窗口 (window 3) 并运行 vins_to_mavros
tmux new-window -t $SESSION_NAME:3 -n "vins_to_mavros" 'cd ~/zky_swarm_real && source devel/setup.bash && rosrun ego_planner vins_to_mavros'

# 等待
sleep 3

# 创建新窗口 (window 4) 并运行 ego_planner
tmux new-window -t $SESSION_NAME:4 -n "ego_planner" 'cd ~/zky_swarm_real && source devel/setup.bash && roslaunch ego_planner swarm.launch'

# 等待
sleep 5

# 创建新窗口 (window 5) 并运行 gcs_bridge
tmux new-window -t $SESSION_NAME:5 -n "gcs_bridge" 'cd ~/zky_swarm_real && source devel/setup.bash && roslaunch swarm_ros_bridge gcs_bridge.launch'

echo "所有 ROS 节点已在 tmux 会话 '$SESSION_NAME' 的不同窗口中启动。"
echo "使用 'tmux attach -t $SESSION_NAME' 命令连接到会话查看各个节点的输出。"

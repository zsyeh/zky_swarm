#!/bin/bash

# 定义 tmux 会话名称
SESSION_NAME="position_mode"

# 检查会话是否存在，如果存在则杀死它，防止重复运行
tmux has-session -t $SESSION_NAME 2>/dev/null
if [ $? == 0 ]; then
    echo "发现已存在的 tmux 会话 '$SESSION_NAME'，正在终止..."
    tmux kill-session -t $SESSION_NAME
fi

echo "正在为 Position 模式启动新的 tmux 会话 '$SESSION_NAME'..."
echo "将依次启动 Livox Driver, Fast-LIO, MAVROS, 和 Pose Transformer..."

# 1. 启动 tmux 会话，并在第一个窗口 (window 0) 运行 Livox 驱动
# -d: 分离模式，在后台创建
# -s: 指定会话名称
# -n: 指定窗口名称
tmux new-session -d -s $SESSION_NAME -n "livox_driver" 'cd ~/livox_ws && source devel/setup.bash && roslaunch livox_ros_driver2 msg_MID360.launch'

# 等待驱动启动
sleep 8

# 2. 创建新窗口 (window 1) 并运行 Fast-LIO
tmux new-window -t $SESSION_NAME:1 -n "fast_lio" 'cd ~/livox_ws && source devel/setup.bash && roslaunch fast_lio mapping_mid360.launch'

# 等待 Fast-LIO 初始化
sleep 5

# 3. 创建新窗口 (window 2) 并运行 MAVROS
tmux new-window -t $SESSION_NAME:2 -n "mavros" 'source /opt/ros/noetic/setup.bash && roslaunch mavros px4.launch'

# 等待 MAVROS 和飞控建立连接
sleep 15

# 4. 创建新窗口 (window 3) 并运行 vins_to_mavros (位姿转换节点)
# 注意：这个节点所在的包是 zky_swarm_real
tmux new-window -t $SESSION_NAME:3 -n "pose_transformer" 'cd ~/zky_swarm_real && source devel/setup.bash && rosrun ego_planner vins_to_mavros'

echo "--------------------------------------------------------"
echo "Position 模式所需的核心节点已全部在后台启动。"
echo "Fast-LIO 正在计算位姿，并通过 MAVROS 发送给飞控。"
echo "可以使用 'tmux attach -t $SESSION_NAME' 命令连接到会话查看各节点状态。"
echo "--------------------------------------------------------"

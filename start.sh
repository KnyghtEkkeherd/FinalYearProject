#!/bin/bash
SESSION="ros2_session"

tmux new-session -d -s "$SESSION" -n "launch_robot" "ros2 launch robot_description launch_robot.launch.py"
tmux new-window -t "$SESSION:1" -n "rplidar" "ros2 launch robot_description rplidar.launch.py"
tmux new-window -t "$SESSION:2" -n "slam" "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false"
tmux new-window -t "$SESSION:3" -n "nav2" "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true"

tmux new-window -t "$SESSION:4" -n

tmux attach-session -t "$SESSION"

#!/bin/bash
# Name of the tmux session
SESSION="ros2_session"

# Create a new tmux session in the background with the first window
tmux new-session -d -s "$SESSION" -n "launch_robot" "ros2 launch robot_description launch_robot.launch.py"

# Create additional windows for the other commands
tmux new-window -t "$SESSION:1" -n "rplidar" "ros2 launch robot_description rplidar.launch.py"
tmux new-window -t "$SESSION:2" -n "slam" "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false"
tmux new-window -t "$SESSION:3" -n "nav2" "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true"

# (Optional) Create an extra, empty window if you need one:
# tmux new-window -t "$SESSION:4" -n "empty"

# Attach to the tmux session
tmux attach-session -t "$SESSION"

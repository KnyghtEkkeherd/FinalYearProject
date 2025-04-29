#!/bin/bash

tmux kill-server
sudo cp src/robot_description/config/my_controllers.yaml /tmp/

SESSION="ros2_session"

tmux new-session -d -s "$SESSION" -n "launch_robot" "ros2 launch robot_description launch_robot.launch.py"
tmux new-window -t "$SESSION:1" -n "rplidar" "ros2 launch robot_description rplidar.launch.py"
tmux new-window -t "$SESSION:2" -n "slam" "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false"
tmux new-window -t "$SESSION:3" -n "nav2" "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true params_file:=src/robot_description/config/nav2_params.yaml"
#tmux new-window -t "$SESSION:4" -n "gpio" "ros2 run gpio_handler gpio_handler"
#tmux new-window -t "$SESSION:5" -n "cam" "ros2 run camera_ros camera_node"
#tmux new-window -t "$SESSION:6" -n "med" "ros2 run medicine_dispenser medicine_dispenser"

tmux new-window -t "$SESSION:7" -n

tmux attach-session -t "$SESSION"

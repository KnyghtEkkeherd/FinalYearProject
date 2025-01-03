conda activate ros_env
colcon build --symlink-install
source install/setup.zsh

ros2 run localisation_planning icp &
ros2 run localisation_planning laser_scan_handler &
ros2 run localisation_planning laser_scan_debug_publisher &
ros2 run localisation_planning ekf_slam &

wait

source install/setup.zsh &

ros2 run odometry icp &
ros2 run odometry laser_scan_debug_publisher &
wait

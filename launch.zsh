source install/setup.zsh &

ros2 run odometry icp &
ros2 run odometry laser_scan_handler &
ros2 run odometry laser_scan_debug_publisher &
ros2 run odometry ekf_slam &

wait

source install/setup.zsh &

ros2 run odometryicp &
ros2 run odometrylaser_scan_handler &
ros2 run odometrylaser_scan_debug_publisher &
ros2 run odometryekf_slam &

wait

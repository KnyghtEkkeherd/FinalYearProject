**FollowMe Robot System**

*Launch the robot visualisation*
```
ros2 launch robot_description viz.launch.py
```

*Launch the robot*
```
ros2 launch robot_description launch_robot.launch.py
```

*Drive the Robot with teleop keyboard*
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

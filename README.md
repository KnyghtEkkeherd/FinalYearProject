**FollowMe Robot System**

*Build the workspace*
```
colcon build --symlink-install
```
*Source the workspace*
```
source install/setup.bash
```

*Launch the robot visualisation OR launch the robot*
```
ros2 launch robot_description viz.launch.py
```
```
ros2 launch robot_description launch_robot.launch.py
```

*Drive the robot with teleop keyboard*
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

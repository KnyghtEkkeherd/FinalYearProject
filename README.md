**FollowMe Robot System**

*Build the workspace*
```
colcon build --symlink-install
```
*Source the workspace*
```
source install/setup.bash
```

*Launch the robot visualisation*
```
ros2 launch robot_description viz.launch.py
```
**Launch the simulation**
```
ros2 launch robot_description sim.launch.py
```
```
rviz2
```
```
ros2 launch robot_descriptionlocalization_launch.py map:=./map_save.yaml use_sim_time:=true
```
(If no map has been created run SLAM)
```
ros2 launch robot_description online_async_launch.py slam_params_file:=./src/robot_description/config/mapper_params_online_async.yaml use_sim_time:=true
```
**Launch the robot**
*Raspberry PI*
```
ros2 launch robot_description launch_robot.launch.py
```
```
ros2 launch robot_description rplidar.launch.py
```
*Launch the navigation toolbox with AMCL*
```
ros2 launch robot_description localization_launch.py map:=./map_save.yaml use_sim_time:=false
```
```
ros2 launch robot_description navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true
```
(Launch slam toolbox if not running Nav2 with AMCL)
```
ros2 launch robot_description online_async_launch.py slam_params_file:=./src/robot_description/config/mapper_params_online_async.yaml use_sim_time:=false
```
*Dev machine*
*Drive the robot with teleop keyboard*
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```
```
rviz2
```
```

```

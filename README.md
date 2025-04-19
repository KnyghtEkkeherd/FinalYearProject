# Follow me Robot System

## Build the workspace

[Dev Machine | Raspberry Pi]

```
colcon build --symlink-install
```

Source the workspace

```
source install/setup.bash
```

---

## Launch the robot

### [Raspberry Pi]

Launch the robot description and lidar handler nodes

```
ros2 launch robot_description launch_robot.launch.py
```

```
ros2 launch robot_description rplidar.launch.py
```

Launch the SLAM toolbox (no AMCL)

```
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

Launch the navigation

```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true params_file:=src/robot_description/config/nav2_params.yaml
```

Launch the localization toolbox with AMCL (if not running SLAM toolbox)

```
ros2 launch robot_description localization_launch.py map:=./map_save.yaml use_sim_time:=false
```

### [Dev Machine]

Visualize the robot and the local map

```
rviz2
```

Run the camera node

```
ros2 run camera_ros camera_node
```

Display the image topic (select /camera/image_raw/compressed):

```
rqt
```

---

**Launch the simulation in Gazebo**

```
ros2 launch robot_description sim.launch.py
```

```
rviz2
```

```
ros2 launch robot_description localization_launch.py map:=./map_save.yaml use_sim_time:=true
```

```
ros2 launch robot_description navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
```

(If no map has been created run SLAM)

```
ros2 launch robot_description online_async_launch.py slam_params_file:=./src/robot_description/config/mapper_params_online_async.yaml use_sim_time:=true
```

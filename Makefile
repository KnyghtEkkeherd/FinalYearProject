BUILD_DIR := build
INSTALL_DIR := install

.PHONY: init build clean help

build:
	echo "Building workspace..."
	colcon build --symlink-install

launch:
	echo "Launching robot description..."
	ros2 launch robot_description launch_robot.launch.py &
	echo "Launching lidar handler nodes..."
	ros2 launch robot_description rplidar.launch.py &
	echo "Launching SLAM toolbox without AMCL..."
	ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false &
	echo "Launching nav..."
	ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true &

nodes:
	echo "Launching nodes..."
	ros2 run camera_ros camera_node &
	ros2 run uwb uwb &

gazebo:
	echo "Launching simulation in Gazebo..."
	ros2 launch robot_description sim.launch.py &
	ros2 launch robot_description localization_launch.py map:=./map_save.yaml use_sim_time:=true &
	ros2 launch robot_description navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true &

slam:
	echo "Running SLAM..."
	ros2 launch robot_description online_async_launch.py slam_params_file:=./src/robot_description/config/mapper_params_online_async.yaml use_sim_time:=true &

clean:
	echo "Cleaning build artifacts and workspace..."
	rm -rf $(BUILD_DIR) $(INSTALL_DIR)

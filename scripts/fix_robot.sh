#!/bin/bash

#Stopping all background processes to prevent interfering
killall -9 gz ruby python3
pkill -9 -f gazebo
pkill -9 -f ros
pkill -9 -f slam_toolbox
pkill -9 -f robot_state_publisher
pkill -9 -f parameter_bridge

#Waiting for cleanup
sleep 3

echo "Starting gazebo simulation"
ros2 launch ~/ros2_ws/src/launch/simulation.launch.py &
sleep 15	#Waiting to load Gazebo properly

#Establishing clock bridge first
ros2 run ros_gz_bridge parameter_bridge /world/robocon_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock --ros-args -r /world/robocon_world/clock:=/clock &
sleep 2

#Bridge for tracking robot parts
ros2 run ros_gz_bridge parameter_bridge /world/robocon_world/model/turtlebot3_waffle/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model --ros-args -r /world/robocon_world/model/turtlebot3_waffle/joint_state:=/joint_states &
sleep 1

#Bridge for tracking co-ordinate system
ros2 run ros_gz_bridge parameter_bridge /tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V --ros-args -p use_sim_time:=true &

#Bridge for LiDAR
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan --ros-args -p use_sim_time:=true &
sleep 1

#Bridge necessary for running teleop_twist_keyboard
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist --ros-args -p use_sim_time:=true &

#Was having issues with 2 Robot State Publishers running at once:
#1)Real Time (Wall Clock)
#2)Sim Time (Gazebo Clock)
#So killing all robot state publishers and relaunching once sim time
pkill -f robot_state_publisher
sleep 2

ros2 run robot_state_publisher robot_state_publisher --ros-args -p use_sim_time:=true -p robot_description:="$(xacro ~/ros2_ws/src/config/clean_waffle.urdf)" &
sleep 2

#Was getting continuous "Timed out waiting for transform" errors
#So manually wrote these 3 bridges
#Odom to gazebo odom
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom turtlebot3_waffle/odom --ros-args -p use_sim_time:=true &
#Gazebo base to real base
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 turtlebot3_waffle/base_footprint base_footprint --ros-args -p use_sim_time:=true &
#Real base to laser
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint turtlebot3_waffle/base_footprint/hls_lfcd_lds --ros-args -p use_sim_time:=true &

#Used for SLAM, currently commented since SLAM has already been done
#ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

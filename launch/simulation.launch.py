import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    #WORLD & ROBOT PATHS
    world_file = os.path.expanduser('~/ros2_ws/src/world/robocon.world')
    urdf_file = os.path.expanduser('~/ros2_ws/src/clean_waffle.urdf')
    pkg_turtlebot3 = get_package_share_directory('turtlebot3_description')
    pkg_share_path = os.path.dirname(pkg_turtlebot3)

    #Setting environment variables
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=pkg_share_path
    )

    #Telling SLAM where the lidar is relative to the wheels
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': True}], 
    )

    #Gazebo simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    #Spawning robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_waffle',
            '-file', urdf_file,
            '-x', '0.6165', '-y', '1.0', '-z', '0.3', '-P', '0.1'   #Not spawning at origin as origin is inside some obstacle, which was causing spawning issues
        ],
        output='screen'
    )

    #Bridges
    #Bridges were not getting established, so I have commented all these
    #and establishing all bridges manually in terminal window (inside fix_robot.sh file)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
#        arguments=[
#            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',      # Drive Command
#            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',   # Lidar Scan
#            '/model/turtlebot3_waffle/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',    # Odometry
#            '/model/turtlebot3_waffle/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',           # TF for SLAM
#            '/world/robocon_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'              # Clock time sync for SLAM
#        ],
#        remappings=[
#            ('/model/turtlebot3_waffle/odometry', '/odom'),
#            ('/model/turtlebot3_waffle/tf', '/tf'),
#	    ('/world/robocon_world/clock', '/clock')
#        ],
        output='screen'
    )

    return LaunchDescription([
        set_resource_path,
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        bridge
    ])

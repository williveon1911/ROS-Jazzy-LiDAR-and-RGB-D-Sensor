import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description(): # Fixed the space typo here
    package_name = 'my_bot'

    # Path to your custom world file
    world_file_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'my_world.sdf')

    # 1. Include the Robot State Publisher (rsp)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py')]), 
            launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Include the Gazebo Sim launch (Updated package and file name)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments={'gz_args': f'-r {world_file_path}'}.items()    
    )

    slam_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'mapper_params_online_async.yaml'
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items()
    )

    # 3. The New Spawner (Updated executable and arguments)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', 
                   '-name', 'my_bot',
                   '-z', '0.1'], # Slightly lifted to avoid floor clipping
        output='screen'
    )
    # 4. The Bridge (Translates ROS cmd_vel to Gazebo cmd_vel)
    bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        # Lidar (GZ to ROS)
        '/model/my_bot/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        # Odometry (GZ to ROS)
        '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        # cmd_vel (ROS to GZ)
        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        # Clock (Essential for Rviz to synchronize)
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        # TF (Transforms - crucial for Rviz movement)
        '/model/my_bot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        # Joint States (For wheel movement)
        '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
        # Camera topics...
        '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
    
    remappings=[
        # Remap gz-sim namespaced topics to what ROS nodes expect
        ('/model/my_bot/scan', '/scan'),
        ('/model/my_bot/odometry', '/odom'),
        ('/model/my_bot/tf', '/tf'),
    ],
    
    parameters=[{
        'qos_overrides./model/my_bot/scan.publisher.depth': 10,
    }],
    
    output='screen'
)

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        bridge,
        slam_toolbox_launch
    ])
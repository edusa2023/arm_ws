import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('arm3dof')
    
    # 1. Gazebo'yu Başlat
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2. Robotu Yarat (Spawn) 
    spawn = Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=['-name', 'arm3dof', '-file', os.path.join(pkg, 'models', 'model.sdf')], 
        output='screen'
    )

    # 3. Bridge (Köprü)
    bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge', 
        parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml')}], 
        output='screen'
    )

    # 4. Kontrolcü
    control = Node(
        package='arm3dof', 
        executable='move_demo.py', 
        name='arm_controller', 
        output='screen'
    )

    return LaunchDescription([gazebo, spawn, bridge, control])
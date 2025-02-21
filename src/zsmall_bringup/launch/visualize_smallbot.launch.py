import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    pkg_description = get_package_share_directory('zsmall_description')
    pkg_localization = get_package_share_directory('zsmall_localization')
    pkg_mapping = get_package_share_directory('zsmall_mapping')
    pkg_controller = get_package_share_directory('zsmall_controller')
    
    use_slam = LaunchConfiguration("use_slam")
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )
    
    use_map = LaunchConfiguration("use_map")
    use_map_arg = DeclareLaunchArgument(
        "use_map",
        default_value="false"
    )
    
    map_name = LaunchConfiguration("map_name")
    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="floor2"
    )
    
    world = LaunchConfiguration("world") 
    world_arg = DeclareLaunchArgument(
        name="world", 
        default_value="empty.world"
    )
    

    gz_bridge_params_path = os.path.join(
        pkg_description,
        'config',
        'gz_bridge.yaml'
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description,
            'launch',
            'world.launch.py'),
        ),
            launch_arguments={
            'world': world,
        }.items(),
    )

    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output="screen",
        parameters=[
            {'use_sim_time': False}
        ]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_localization, 'config', 'ekf.yaml'),
            {'use_sim_time': False}
             ]
    )
    
    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                pkg_mapping,
                "rviz",
                "slam.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(use_slam)
    )
    
    nav_given_map = IncludeLaunchDescription(
        os.path.join(
            pkg_mapping,
            "launch",
            "navigation_given_map.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False",
            "map_name": map_name,
        }.items(),
        condition=IfCondition(use_map)
    )

    
    return LaunchDescription([
        use_slam_arg,
        use_map_arg,
        map_name_arg,
        world_arg,
        world_launch,
        gz_bridge_node,
        ekf_node,
        world_arg,
    #    safety_stop,
        rviz_slam,
        nav_given_map,
    ])
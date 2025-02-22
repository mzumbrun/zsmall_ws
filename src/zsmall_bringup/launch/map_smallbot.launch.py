import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    use_slam = LaunchConfiguration("use_slam")
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )
    
    map_name = LaunchConfiguration("map_name")
    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="floor2"
    )

    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="rplidar_composition",
            parameters=[os.path.join(
                get_package_share_directory("zsmall_bringup"),
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen"
    ) 
       
    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zsmall_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={
            "map_name": map_name,
        }.items(),
        condition=UnlessCondition(use_slam)
    )

    jazzy_slam_toolbox_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("slam_toolbox"),
            "launch",
            "online_sync_launch.py"
        ),
        launch_arguments={
            "slam_params_file": os.path.join(get_package_share_directory('zsmall_mapping'), "config", "slam_toolbox.yaml"),
            "use_sim_time": "False",
        }.items(),
        condition=IfCondition(use_slam)
    )
    
    return LaunchDescription([
        use_slam_arg,
        map_name_arg,
        laser_driver,
       # localization,
        # jazzy_slam_toolbox_launch
    ])
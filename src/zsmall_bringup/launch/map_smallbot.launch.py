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
    
    use_map = LaunchConfiguration("use_map")

    use_map_arg = DeclareLaunchArgument(
        "use_map",
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
            "map_name": use_map,
        }.items(),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zsmall_mapping"),
            "launch",
            "slam.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False",
        }.items(),
        condition=IfCondition(use_slam)
    )
    
    return LaunchDescription([
        use_slam_arg,
        use_map_arg,
        laser_driver,
     #   controller,
     #   joystick,
     #   safety_stop,
# localization,
        slam
    ])
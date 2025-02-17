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

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zsmall_description"),
            "launch",
            "smallbot_gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zsmall_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zsmall_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    safety_stop = Node(
        package="zsmall_utils",
        executable="safety_stop.py",
        output="screen",
        parameters=[{"use_sim_time": True}]
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
        condition=IfCondition(use_slam)
    )

    rviz_localization = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("zsmall_localization"),
                "rviz",
                "global_localization.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_slam)
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("zsmall_mapping"),
                "rviz",
                "slam.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slam)
    )
    
    moveit = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("zsmall_moveit"),
                "launch",
                "moveit.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items()
        )
    
    remote_interface = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("zsmall_remote"),
                "launch",
                "remote_interface.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items()
        )
    
    return LaunchDescription([
        use_slam_arg,
        use_map_arg,
        gazebo,
        controller,
        joystick,
        safety_stop,
        localization,
        slam,
        rviz_localization,
        rviz_slam,
        #moveit,
        #remote_interface
    ])
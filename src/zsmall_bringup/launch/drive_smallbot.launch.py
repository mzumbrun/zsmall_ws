import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zsmall_firmware"),
            "launch",
            "hardware_interface_smallbot.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False",
            "use_ros2_control": "True"
        }.items(),
    )

   
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zsmall_controller"),
            "launch",
            "controller_smallbot.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "True",
            "use_sim_time": "False",
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("zsmall_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )
        
    imu_driver_node = Node(
        package="zsmall_firmware",
        executable="mpu6050_driver.py"
    )

    safety_stop = Node(
        package="zsmall_utils",
        executable="safety_stop.py",
        output="screen",
    )
   
    moveit = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("zsmall_moveit"),
                "launch",
                "moveit.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    remote_interface = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("zsmall_remote"),
                "launch",
                "remote_interface.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    return LaunchDescription([
        hardware_interface,
        controller,
     #   joystick,
        imu_driver_node,
      #  safety_stop,
     #   moveit,
     #   remote_interface,
    ])
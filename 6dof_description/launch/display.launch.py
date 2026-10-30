import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('6dof_description')
    xacro_file = os.path.join(pkg_share, 'urdf', '6dof.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'display.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str
    )

    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start joint_state_publisher_gui (joint sliders)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])

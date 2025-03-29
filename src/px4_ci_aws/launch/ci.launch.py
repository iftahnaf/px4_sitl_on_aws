from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch.logging
import logging
import os

launch.logging.launch_config.level = logging.INFO

parameters = os.path.join(
        get_package_share_directory('px4-external-module'),
        'config',
        'params.yaml'
    )


def generate_launch_description():
    px4_launch_command = (
        "cd /workspaces/px4_sitl_on_aws/PX4-Autopilot && sleep 2 &&"
        + " PX4_SYS_AUTOSTART=4001"
        + " PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4"
    )

    px4_proc = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            f"{px4_launch_command}",
        ],
        output="screen",
        emulate_tty=True,
        name="px4",
    )

    node_dds_agent = ExecuteProcess(
        cmd=["bash", "-c", "MicroXRCEAgent udp4 -p 8888"],
        output="screen",
        emulate_tty=True,
    )

    node_arm_and_offboard = Node(
            package='px4_ci_aws',
            executable='px4_arm_and_offboard.py',
            name='arm_and_offboard',
        )
    
    node_offboard = Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='offboard_control',
            name='control',
            parameters= [{'radius': 10.0},{'altitude': 5.0},{'omega': 0.5}]
        )
    
    elements_to_launch = [
        node_arm_and_offboard,
        node_offboard,
        node_dds_agent,
        px4_proc,
    ]

    ld = LaunchDescription(elements_to_launch)
    return ld
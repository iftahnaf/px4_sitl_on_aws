from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess
)
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
        " PX4_SYS_AUTOSTART=4001 "
        + "PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4"
    )

    px4_proc = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            f" cd /workspaces/px4_sitl_on_aws/PX4-Autopilot && sleep 6 && {px4_launch_command}",
        ],
        output="screen",
        emulate_tty=True,
        name="px4",
    )

    gz_sim_command = [
        "python3",
        "/workspaces/px4_sitl_on_aws/PX4-Autopilot/Tools/simulation/gz/simulation-gazebo",
        "--world",
        "default",
        "--model_store",
        "/workspaces/px4_sitl_on_aws/PX4-Autopilot/Tools/simulation/gz/",
    ]

    gz_sim = ExecuteProcess(
        cmd=gz_sim_command,
        name="gz_sim",
        output="screen",
        emulate_tty=True,
    )

    node_dds_agent = ExecuteProcess(
        cmd=["bash", "-c", "MicroXRCEAgent udp4 -p 8888"],
        output="screen",
        emulate_tty=True,
    )

    elements_to_launch = [
        gz_sim,
        node_dds_agent,
        px4_proc,
    ]

    ld = LaunchDescription(elements_to_launch)
    return ld

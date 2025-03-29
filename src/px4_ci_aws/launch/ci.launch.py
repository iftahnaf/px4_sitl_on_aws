from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction,
    TimerAction,
)
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
import launch.logging
import logging
import time
import os
import signal
from launch import LaunchContext

from rosbags.highlevel import AnyReader
from pathlib import Path
from collections import defaultdict

launch.logging.launch_config.level = logging.INFO

logging.basicConfig(level=logging.INFO)

def update_nav_dll_act(filepath: str, new_value: int = 0):
    """
    Update the NAV_DLL_ACT parameter value in a PX4 shell configuration script.

    Args:
        filepath (str): Path to the shell script file.
        new_value (int): The new value to set (default: 0).
    """
    with open(filepath, 'r') as f:
        lines = f.readlines()

    updated_lines = []
    for line in lines:
        if line.strip().startswith("param set-default NAV_DLL_ACT"):
            updated_line = f"param set-default NAV_DLL_ACT {new_value}\n"
            updated_lines.append(updated_line)
        else:
            updated_lines.append(line)

    with open(filepath, 'w') as f:
        f.writelines(updated_lines)

    logging.info(f"Updated NAV_DLL_ACT to {new_value} in {filepath}")

def post_process(context: LaunchContext, arg1: LaunchConfiguration, bag_name: str, recorder: ExecuteProcess):
        time.sleep(1.0)
        pid = recorder.process_details['pid']
        os.kill(pid, signal.SIGTERM)

        logging.info(f"killed recorded on process {pid}")
        time.sleep(1.0)

        message_counts = defaultdict(int)

        with AnyReader([Path(bag_name)]) as reader:
            reader.open()
            for conn, timestamp, raw in reader.messages():
                message_counts[conn.topic] += 1

        logging.info(f"Topics in bag: {bag_name}\n")
        for topic, count in sorted(message_counts.items(), key=lambda x: x[0]):
            logging.info(f"{topic}: {count} messages")

        logging.info(f"Bag {bag_name} has been analyzed")

update_nav_dll_act("/workspaces/px4_sitl_on_aws/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500", 0)

def generate_launch_description():

    px4_launch_command = (
        "cd /workspaces/px4_sitl_on_aws/PX4-Autopilot && sleep 2 &&"
        + " HEADLESS=1 PX4_SYS_AUTOSTART=4001"
        + " PX4_SIM_MODEL=gz_x500 make px4_sitl gz_x500"
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
            executable='px4_state_machine.py',
            name='px4_state_machine',
        )
    
    node_offboard = Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='offboard_control',
            name='control',
            parameters= [{'radius': 10.0},{'altitude': 30.0},{'omega': 0.5}]
        )
    
    bag_name = f'/bags/{time.strftime("%Y-%m-%d_%H:%M:%S", time.gmtime())}'
    recorder = ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "-a",
                "-o",
                f"{bag_name}",

            ],
        )
    
    sys_shut_down = RegisterEventHandler(
        OnProcessExit(
            target_action=node_arm_and_offboard,
            on_exit=[
                LogInfo(msg=("The Scenario has ended!")),
                EmitEvent(event=Shutdown(reason="Finished")),
            ],
        )
    )

    sys_shut_down_timer = TimerAction(
        period=float(120.0),
        actions=[
            LogInfo(msg="Timeout reached, shutting down the scenario."),
            EmitEvent(event=Shutdown(reason="Timeout reached")),
        ],
    )

    analysis_configuration = LaunchConfiguration('analysis')
    analyze = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(msg='Analyzing the bag'),
                OpaqueFunction(function=post_process, args=[analysis_configuration, bag_name, recorder])
            ]
        )
    )
    
    elements_to_launch = [
        px4_proc,
        node_arm_and_offboard,
        node_offboard,
        node_dds_agent,
        recorder,
        sys_shut_down,
        sys_shut_down_timer,
        analyze
    ]

    ld = LaunchDescription(elements_to_launch)
    return ld
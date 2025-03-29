from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction
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

launch.logging.launch_config.level = logging.INFO

def post_process(context: LaunchContext, arg1: LaunchConfiguration, bag_name: str, recorder: ExecuteProcess):
        time.sleep(1.0)
        pid = recorder.process_details['pid']
        os.kill(pid, signal.SIGTERM)

        print(f"killed recorded on process {pid}")
        time.sleep(1.0)

        print("post processing...")

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
            executable='px4_state_machine.py',
            name='px4_state_machine',
        )
    
    node_offboard = Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='offboard_control',
            name='control',
            parameters= [{'radius': 10.0},{'altitude': 5.0},{'omega': 0.5}]
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
        node_arm_and_offboard,
        node_offboard,
        node_dds_agent,
        recorder,
        sys_shut_down,
        analyze,
        px4_proc,
    ]

    ld = LaunchDescription(elements_to_launch)
    return ld
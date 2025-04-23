#!/usr/bin/env python3

# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025, Iftach Naftaly <iftahnaf@gmail.com>

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from px4_msgs.msg import VehicleStatus, VehicleCommand, OffboardControlMode
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from enum import Enum
import sys


class XRCE_TOPICS(Enum):
    VEHICLE_COMMAND = "in/vehicle_command"
    OFFBOARD_CONTROL_MODE = "in/offboard_control_mode"

class OffboardState(Enum):
    INIT = 0
    ARMING = 1
    ARMED = 2
    OFFBOARD = 3
    RTL = 4
    DONE = 5


class ArmOffboardNode(Node):
    def __init__(self):
        super().__init__("arm_and_offboard")

        self.status = None
        self.prefix = "/fmu"
        self.offboard_state = OffboardState.INIT
        self.rtl_triggered = False

        self.offboard_time_s = self.declare_parameter('offboard_time_s', 30.0).value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, f"{self.prefix}/{XRCE_TOPICS.VEHICLE_COMMAND.value}", qos_profile
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, f"{self.prefix}/{XRCE_TOPICS.OFFBOARD_CONTROL_MODE.value}", qos_profile
        )

        self.create_subscription(
            VehicleStatus,
            f"{self.prefix}/out/vehicle_status_v1",
            self.status_callback,
            qos_profile,
        )

        # Timer to check status
        self.timer = self.create_timer(0.5, self.state_machine)

    def status_callback(self, msg: VehicleStatus):
        self.status = msg

    def state_machine(self):
        if self.offboard_state == OffboardState.INIT:
            self.get_logger().info("Waiting for vehicle_status...", throttle_duration_sec=5.0)
            if self.status is not None:
                self.offboard_state = OffboardState.ARMING

        if self.offboard_state == OffboardState.ARMING:
            self.get_logger().info("Sending ARM command...", throttle_duration_sec=5.0)
            self.arm()
            if self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.offboard_state = OffboardState.ARMED

        elif self.offboard_state == OffboardState.ARMED:
            self.get_logger().info("Sending OFFBOARD command...", throttle_duration_sec=5.0)
            self.set_offboard()
            if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.offboard_state = OffboardState.OFFBOARD
                self.offboard_init_time_s = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] / 1e9

        elif self.offboard_state == OffboardState.OFFBOARD:
            self.get_logger().info("Vehicle is armed and in OFFBOARD mode.", throttle_duration_sec=5.0)
            current_time_s = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] / 1e9
            delta_time_s = current_time_s - self.offboard_init_time_s
            if delta_time_s > self.offboard_time_s:
                self.offboard_state = OffboardState.RTL
                self.get_logger().info("RTL command will be sent.", throttle_duration_sec=5.0)
        
        elif self.offboard_state == OffboardState.RTL:
            if not self.rtl_triggered:
                self.set_rtl()
                self.get_logger().info("Sending RTL command...", once=True)
                self.rtl_triggered = True
            if self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.offboard_state = OffboardState.DONE
        
        elif self.offboard_state == OffboardState.DONE:
            self.get_logger().info("Mission completed.", once=True)
            self.timer.cancel()
            sys.exit(0)


    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0  # arm
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 101
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def set_offboard(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0
        cmd.param2 = 6.0  # PX4 offboard mode ID
        cmd.timestamp = int(Clock().now().nanoseconds / 1000)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 101
        cmd.source_component = 1
        cmd.from_external = True
        self.vehicle_command_pub.publish(cmd)
        self.set_offboard_control_mode()

    def set_offboard_control_mode(self):
        ctrl = OffboardControlMode()
        ctrl.timestamp = int(Clock().now().nanoseconds / 1000)
        ctrl.attitude = False
        ctrl.body_rate = False
        ctrl.velocity = False
        ctrl.position = True
        ctrl.acceleration = False
        self.offboard_mode_pub.publish(ctrl)

    def set_rtl(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 101
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info("RTL command sent.")


def main(args=None):
    rclpy.init(args=args)
    node = ArmOffboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

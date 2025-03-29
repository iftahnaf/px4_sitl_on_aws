#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from px4_msgs.msg import VehicleStatus, VehicleCommand, OffboardControlMode
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from enum import Enum


class XRCE_TOPICS(Enum):
    VEHICLE_COMMAND = "in/vehicle_command"
    OFFBOARD_CONTROL_MODE = "in/offboard_control_mode"


class ArmOffboardNode(Node):
    def __init__(self):
        super().__init__("arm_and_offboard")

        self.status = None
        self.prefix = "/fmu"

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
        self.timer = self.create_timer(0.5, self.control_loop)

    def status_callback(self, msg: VehicleStatus):
        self.status = msg

    def control_loop(self):
        if self.status is None:
            self.get_logger().info("Waiting for vehicle_status...", throttle_duration_sec=5.0)
            return

        if self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info("Sending ARM command...", throttle_duration_sec=5.0)
            self.arm()

        elif self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().info("Sending OFFBOARD command...", throttle_duration_sec=5.0)
            self.set_offboard()

        else:
            self.get_logger().info("Vehicle is armed and in OFFBOARD mode.", once=True)

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


def main(args=None):
    rclpy.init(args=args)
    node = ArmOffboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

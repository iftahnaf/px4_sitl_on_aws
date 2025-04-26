# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025, Iftach Naftaly <iftahnaf@gmail.com>

import os
from pathlib import Path

from utils import register_custom_msgs, get_px4_msgs
from datetime import datetime
from rosbags.rosbag2 import Reader
from rosbags.typesys.store import Typestore
import matplotlib.pyplot as plt
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class SingleReport:
    def __init__(self, bag_path: str, px4_msgs_path: str = None):
        logger.info("Initializing SingleReport...")
        logger.info(f"Bag path: {bag_path}")
        logger.info(f"PX4 messages path: {px4_msgs_path}")
        self.bag_path = bag_path
        px4_msgs = get_px4_msgs(px4_msgs_path)
        self.typestore: Typestore = register_custom_msgs(px4_msgs)

        results_dir = "/workspaces/px4_sitl_on_aws/results"
        os.makedirs(results_dir, exist_ok=True)  # Create results dir if not exists

        self.report_path = os.path.join(
            results_dir, f"{os.path.basename(bag_path)}_report.md"
        )

        self.messages_per_topic = {}

        self.local_position = {
            "timestamp": [],
            "x": [],
            "y": [],
            "z": [],
            "vx": [],
            "vy": [],
            "vz": [],
        }
        self.vehicle_status = {
            "timestamp": [],
            "nav_state": [],
            "armed": [],
        }

        self.required_topics = [
            "/fmu/out/vehicle_local_position",
            "/fmu/out/vehicle_status_v1",
        ]

        self.read_bag()
        self.generate_report()
        logger.info(f"Report generated at: {self.report_path}")

    def generate_report(self):
        self.generate_plots()
        with open(self.report_path, "w") as report_file:
            report_file.write("# 📢 Report 📢\n")
            report_file.write(f"## 🎒 Bag file: {self.bag_path}\n")
            report_file.write(f"## 📆 Report generated on: {datetime.now()}\n")
            report_file.write("\n")
            report_file.write("## ✏️ Messages per topic:\n")
            report_file.write("| Topic | Count |\n")
            report_file.write("|-------|-------|\n")
            for topic, count in self.messages_per_topic.items():
                report_file.write(f"| {topic} | {count} |\n")
            report_file.write("\n")
            report_file.write("## 📈 Local Position vs time plots:\n")
            report_file.write("### Position\n")
            report_file.write("![Position](position_local_position.png)\n")
            report_file.write("### Velocity\n")
            report_file.write("![Velocity](velocity_local_position.png)\n")
            report_file.write("\n")
            report_file.write("## 📉 Trajectory plot:\n")
            report_file.write("![Trajectory](trajectory.png)\n")
            report_file.write("\n")
            report_file.write("## 📊 Vehicle Status vs time plots:\n")
            report_file.write("### Navigation State\n")
            report_file.write("![Navigation State](vehicle_status.png)\n")

    def generate_plots(self):
        self.generate_plot_local_position_vs_time("position")
        self.generate_plot_local_position_vs_time("velocity")
        self.generate_plot_trajectory()
        self.generate_plot_vehicle_status_vs_time()

    def generate_plot_local_position_vs_time(self, plot_type: str = "position"):
        plt.figure(figsize=(10, 6))
        plot_id = "" if plot_type == "position" else "v"
        plot_description = "Position" if plot_type == "position" else "Velocity"
        plot_units = "m" if plot_type == "position" else "m/s"
        plt.plot(
            self.local_position[f"{plot_id}x"],
            label=f"X {plot_description}",
        )
        plt.plot(
            self.local_position[f"{plot_id}y"],
            label=f"Y {plot_description}",
        )
        plt.plot(
            self.local_position[f"{plot_id}z"],
            label=f"Z {plot_description}",
        )
        plt.xlabel("Timestamp")
        plt.ylabel(f"{plot_description} [{plot_units}]")
        plt.title(f"{plot_description} Local Position over Time")
        plt.legend()
        plt.grid()
        plt.savefig(
            os.path.join(
                os.path.dirname(self.report_path),
                f"{plot_description}_local_position.png",
            )
        )
        plt.close()

    def generate_plot_trajectory(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.local_position["x"], self.local_position["y"], label="Trajectory")
        plt.xlabel("X Position [m]")
        plt.ylabel("Y Position [m]")
        plt.title("Trajectory in XY Plane")
        plt.legend()
        plt.grid()
        plt.savefig(os.path.join(os.path.dirname(self.report_path), "trajectory.png"))
        plt.close()

    def generate_plot_vehicle_status_vs_time(self):
        plt.figure(figsize=(10, 6))
        plt.plot(
            self.vehicle_status["nav_state"], label="Navigation State"
        )
        plt.plot(self.vehicle_status["armed"], label="Armed State")
        plt.xlabel("Timestamp")
        plt.ylabel("State")
        plt.title("Vehicle Status over Time")
        plt.legend()
        plt.grid()
        plt.savefig(
            os.path.join(os.path.dirname(self.report_path), "vehicle_status.png")
        )
        plt.close()

    def read_bag(self):
        with Reader(Path(self.bag_path)) as reader:
            for connection in reader.connections:
                self.messages_per_topic[connection.topic] = 0
            for connection, timestamp, rawdata in reader.messages():
                self.messages_per_topic[connection.topic] += 1
            connections = [
                x for x in reader.connections if x.topic in self.required_topics
            ]
            for connection, timestamp, rawdata in reader.messages(
                connections=connections
            ):
                msg = self.typestore.deserialize_cdr(rawdata, connection.msgtype)
                if connection.topic in self.required_topics[0]:
                    self.local_position["timestamp"].append(timestamp)
                    self.local_position["x"].append(msg.x)
                    self.local_position["y"].append(msg.y)
                    self.local_position["z"].append(msg.z)
                    self.local_position["vx"].append(msg.vx)
                    self.local_position["vy"].append(msg.vy)
                    self.local_position["vz"].append(msg.vz)
                elif connection.topic in self.required_topics[1]:
                    self.vehicle_status["timestamp"].append(timestamp)
                    self.vehicle_status["nav_state"].append(msg.nav_state)
                    self.vehicle_status["armed"].append(msg.arming_state)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Generate a report from a ROS2 bag file."
    )
    parser.add_argument("bag_path", type=str, help="Path to the ROS2 bag file.")
    parser.add_argument(
        "--px4_msgs_path",
        type=str,
        help="Path to the directory containing custom PX4 messages.",
        default="/workspaces/px4_sitl_on_aws/install/share/px4_msgs/msg",
    )
    args = parser.parse_args()

    report = SingleReport(args.bag_path, args.px4_msgs_path)

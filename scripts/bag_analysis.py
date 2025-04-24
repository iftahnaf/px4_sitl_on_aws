# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025, Iftach Naftaly <iftahnaf@gmail.com>


import os
import sys

def main():
    if len(sys.argv) != 2:
        print("Usage: python bag_analysis.py <bag_directory>")
        sys.exit(1)

    bag_path = sys.argv[1]
    print(f"Analyzing bag at: {bag_path}")

    results_dir = "results"
    os.makedirs(results_dir, exist_ok=True)

    report_path = os.path.join(results_dir, f"{os.path.basename(bag_path)}_report.md")
    with open(report_path, "w") as f:
        f.write("# Bag Analysis Report\n\n")
        f.write(f"Hello, world!\n\n")
        f.write(f"Bag path: `{bag_path}`\n")

    print(f"Report written to {report_path}")

if __name__ == "__main__":
    main()

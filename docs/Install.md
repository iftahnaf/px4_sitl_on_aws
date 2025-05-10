# Installation & Development Setup

This repository is fully configured for development using a prebuilt Docker image and DevContainer.

Everything you need — dependencies, PX4 Autopilot, ROS 2 workspace — is already provided via the development container.

---

## Quick Start

### 1. Clone the Repository

Clone this repository to your local machine.

Navigate into the project directory.

---

### 2. Open in DevContainer (VSCode)

Make sure you have installed:

- Visual Studio Code
- Remote - Containers Extension (DevContainers)
- Docker installed and running on your machine

Then:

- Open the repository folder in VSCode.
- VSCode will prompt you to "Reopen in Container" — click it.

---

## Behind the Scenes

- The development environment is based on a prebuilt Docker image hosted on GHCR (GitHub Container Registry).
- The image includes:
  - PX4 Autopilot source & dependencies.
  - ROS 2 workspace ready to build.
  - All required tools and environment variables.
- A `postCreateCommand` script will automatically run inside the container to finalize the setup (dependencies, workspace build, etc.).

---

## Notes

- No need to install PX4 or ROS 2 manually.
- No need to build the Docker image locally.
- Just clone → open in DevContainer → start developing.

---

## Development Usage

After opening the DevContainer:

- Build the workspace using `colcon build`.
- Source the workspace with `source install/setup.bash`.
- Launch your simulation or ROS 2 nodes as needed.

---

This setup ensures a consistent and reproducible development environment across all developers and CI pipelines.

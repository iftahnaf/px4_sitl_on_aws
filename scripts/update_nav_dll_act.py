import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y%m%d_%H%M%S'
)


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

if __name__ == "__main__":
    filepath = "/workspaces/px4_sitl_on_aws/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500"
    new_value = 0
    update_nav_dll_act(filepath, new_value)
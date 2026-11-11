"""robotics-lab: ROS 2 and CAN bus tools, plus re-implementations of robotics papers.

Part of the Modelshifts showcase. Status: v1.

The ROS 2 tools read and write rosbag2 (mcap) files. They do not import rclpy.
The CAN tools use a virtual bus by default. The paper code is simulation only.
"""

__version__ = "0.1.0"


class RoboticsLabError(Exception):
    """A problem the user can fix. The CLI prints it and exits 2."""

# cmurkd_real.py
import frankapy as franka
import numpy as np
from shared_api import RobotInterface
import time


class FrankaRobot(RobotInterface):
    """
    Implementation of RobotInterface using a real Franka Panda arm.
    Requires FrankaPy to be installed and a connection to a robot.
    """

    def __init__(self, robot_ip: str = '172.16.0.2'):
        self.robot_ip = robot_ip
        self.arm = None

    def connect(self):
        """Initializes and connects to the Franka robot."""
        print(f"Connecting to Franka robot at {self.robot_ip}...")
        self.arm = franka.FrankArm(self.robot_ip)
        print("Franka robot connected.")

    def disconnect(self):
        """Disconnects from the Franka robot."""
        # FrankaPy handles disconnection gracefully, often not an explicit method call.
        print("Franka robot disconnected.")

    def get_joint_angles(self) -> np.ndarray:
        """Returns the current joint angles from the real robot."""
        if not self.arm:
            raise RuntimeError("Not connected to the robot. Call connect() first.")
        return np.array(self.arm.get_joints())

    def execute_trajectory(self, trajectory: list[np.ndarray]):
        """Executes a trajectory of joint positions on the real robot."""
        if not self.arm:
            raise RuntimeError("Not connected to the robot. Call connect() first.")

        print(f"Executing trajectory of {len(trajectory)} points on real robot...")
        for target_qpos in trajectory:
            # FrankaPy's go_to_joints is a high-level, blocking command.
            self.arm.goto_joints(target_qpos, duration=1.0)  # Move to each point in 1 second

        print("Trajectory execution complete.")

    def home_robot(self):
        """Moves the real robot to its home position."""
        if not self.arm:
            raise RuntimeError("Not connected to the robot. Call connect() first.")

        self.arm.home()
        print("Robot homed.")

# Total lines of code: ~45 lines
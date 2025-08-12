from abc import ABC, abstractmethod
import numpy as np

class RobotInterface(ABC):
    """
    Abstract base class for the Franka Panda robot control interface.
    All implementations (sim or real) must inherit from this class.
    """
    def __init__(self, *args, **kwargs):
        pass

    @abstractmethod
    def connect(self):
        """Connects to the robot or initializes the simulator."""
        pass

    @abstractmethod
    def disconnect(self):
        """Disconnects from the robot or shuts down the simulator."""
        pass

    @abstractmethod
    def get_joint_angles(self) -> np.ndarray:
        """Returns the current joint angles of the robot."""
        pass

    @abstractmethod
    def execute_trajectory(self, trajectory: list[np.ndarray]):
        """Executes a trajectory of joint positions."""
        pass

    @abstractmethod
    def home_robot(self):
        """Moves the robot to a predefined home position."""
        pass


#go_to_cartesian_pose()
#set_joint_torques()
#get_end_effector_pose()
#open_gripper() and close_gripper()
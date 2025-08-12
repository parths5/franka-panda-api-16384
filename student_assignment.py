import numpy as np
from cmurkd_sim import MuJoCoSim as Robot # To run on the simulator
# from cmurkd_real import FrankaRobot as Robot # To run on the real robot

def compute_robot_trajectory() -> list[np.ndarray]:
    """
    like an assignment question - computes a simple trajectory for the robot. Without solving IK for now.
    """
    # example:
    home_pose = np.array([0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4])
    intermediate_pose = np.array([-0.5, -0.2, 0.5, -1.5, 0.5, 1.2, 0.5])
    new_test_pose = np.array([0.2, -1.0, -0.2, -0.0, 0.2, 0.0, 0.2])
    trajectory = [home_pose, intermediate_pose, new_test_pose]
    return trajectory


if __name__ == '__main__':
    # Students define their model path
    robot_path = '/home/parth/Desktop/CMU/16384/16384_ws/franka_emika_panda/panda_nohand_torque.xml'
    # Instantiate the robot
    robot = Robot(model_path=robot_path)

    try:
        robot.connect()
        # students implement their own trajectory function
        student_trajectory = compute_robot_trajectory()
        # Run the trajectory using our provided API
        robot.execute_trajectory(student_trajectory)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        robot.disconnect()
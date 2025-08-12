import mujoco
import numpy as np
import threading
import time
from shared_api import RobotInterface
import glfw


class MuJoCoSim(RobotInterface):
    def __init__(self, model_path: str = 'franka_panda.xml'):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        self.viewer = None
        self.sim_thread = None
        self.running = False

        self.Kp = 500
        self.Kd = 50
        self.target_joint_angles = self.data.qpos.copy()

    def _simulation_loop(self):
        self.running = True

        try:
            from mujoco.glfw import glfw
            from mujoco import viewer

            if not glfw.init():
                return

            self.window = glfw.create_window(1920, 1080, "MuJoCoSim", None, None)
            glfw.make_context_current(self.window)
            self.viewer = viewer.launch_passive(self.model, self.data)

            with self.viewer:
                while self.running and not glfw.window_should_close(self.window):
                    qpos_error = self.target_joint_angles - self.data.qpos
                    qvel_error = -self.data.qvel

                    self.data.ctrl[:] = self.Kp * qpos_error + self.Kd * qvel_error

                    mujoco.mj_step(self.model, self.data)
                    self.viewer.sync()
                    time.sleep(self.model.opt.timestep)

            glfw.terminate()

        except Exception as e:
            print(f"An error occurred in the simulation loop: {e}")
            self.running = False

        glfw.terminate()

    def connect(self):
        print("MuJoCo simulator connected.")
        if not self.sim_thread or not self.sim_thread.is_alive():
            self.sim_thread = threading.Thread(target=self._simulation_loop, daemon=True)
            self.sim_thread.start()

    def disconnect(self):
        print("MuJoCo simulator disconnected.")
        self.running = False
        if self.sim_thread:
            self.sim_thread.join()

    def get_joint_angles(self) -> np.ndarray:
        return self.data.qpos.copy()

    def execute_trajectory(self, trajectory: list[np.ndarray]):
        """Executes a trajectory of joint positions by setting new targets and waiting."""
        print(f"Executing trajectory of {len(trajectory)} points in MuJoCo...")
        for i, target_qpos in enumerate(trajectory):
            self.target_joint_angles = target_qpos
            print(f"  > Moving to pose {i+1}/{len(trajectory)}")
        time.sleep(2.0)  # Wait for the robot to reach the pose
        print("Trajectory execution complete.")

    def home_robot(self):
        home_joints = np.array([0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4])
        self.execute_trajectory([home_joints])
        print("Robot homed.")
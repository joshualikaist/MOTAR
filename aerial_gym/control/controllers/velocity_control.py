import torch
from aerial_gym.utils.math import *


from aerial_gym.control.controllers.base_lee_controller import *
from aerial_gym.utils.logging import CustomLogger

logger = CustomLogger("velocity_controller")


class LeeVelocityController(BaseLeeController):
    def __init__(self, config, num_envs, device):
        super().__init__(config, num_envs, device)

    def init_tensors(self, global_tensor_dict=None):
        super().init_tensors(global_tensor_dict)

    def update(self, command_actions):
        """
        Lee attitude controller
        :param robot_state: tensor of shape (num_envs, 13) with state of the robot
        :param command_actions: tensor of shape (num_envs, 4) with desired thrust, roll, pitch and yaw_rate command in vehicle frame
        :return: m*g normalized thrust and interial normalized torques
        """
        self.reset_commands()
        self.accel[:] = self.compute_acceleration(
            setpoint_position=self.robot_position,
            setpoint_velocity=command_actions[:, 0:3],
        )
        forces = (self.accel[:] - self.gravity) * self.mass
        # thrust command is transformed by the body orientation's z component
        b3 = quat_to_rotation_matrix(self.robot_orientation)[:, :, 2]
        if getattr(self.cfg, "tilt_thrust_compensation", False):
            # Altitude-priority thrust (PX4-style tilt compensation). The standard Lee projection
            # T = f·b3 delivers a vertical force of (f·b3)*b3_z, which equals the commanded f_z ONLY
            # when the desired force direction and the CURRENT body axis agree. During attitude-lag
            # transients (e.g. a weave reversal: body still tilted one way, f already pointing the
            # other) the achieved vertical force sags below f_z deterministically -- both vectors are
            # known at this line, so no prediction is involved. Choosing T = f_z / b3_z makes the
            # achieved vertical force EXACTLY f_z regardless of the current tilt, decoupling altitude
            # from lateral/yaw transients. Cost: during the mismatch some thrust leaks laterally
            # along the stale body axis (slightly slower reversals) -- acceptable, floor strikes are
            # terminal. clamp(min=0.5) caps compensation at a 60 deg tilt to avoid thrust blowup.
            self.wrench_command[:, 2] = forces[:, 2] / b3[:, 2].clamp(min=0.5)
        else:
            self.wrench_command[:, 2] = torch.sum(forces * b3, dim=1)

        # after calculating forces, we calculate the desired euler angles
        self.desired_quat[:] = calculate_desired_orientation_for_position_velocity_control(
            forces, self.robot_euler_angles[:, 2], self.buffer_tensor
        )

        self.euler_angle_rates[:, :2] = 0.0
        self.euler_angle_rates[:, 2] = command_actions[:, 3]
        self.desired_body_angvel[:] = euler_rates_to_body_rates(
            self.robot_euler_angles, self.euler_angle_rates, self.buffer_tensor
        )

        self.wrench_command[:, 3:6] = self.compute_body_torque(
            self.desired_quat, self.desired_body_angvel
        )

        return self.wrench_command

import numpy as np
import time

import config
from .lcm2mujuco_bridge import Lcm2MujocoBridge
from state_estimator import HopperStateEstimator
from utils import *

class HopperBridge(Lcm2MujocoBridge):
    def __init__(self, mj_model, mj_data):
        super().__init__(mj_model, mj_data)
        # State estimator
        self.state_estimator = HopperStateEstimator(config.dt_sim)
        self.thr_counter = 0
        
        # For state estimation visualization only
        self.pos_est = np.array([0, 0, 0.3])
        self.R_body = np.eye(3)

    def parse_robot_specific_low_state(self):

        # Use world frame angular rate and acceleration for state estimation
        quat = Quaternion(*self.low_state.quaternion)
        R_body = quat_to_rot(quat) # body to world

        omega_body = self.low_state.omega
        omega_world = R_body @ omega_body
        self.low_state.omega[:] = omega_world.tolist() # overwrite omega

        accel_body = self.mj_data.sensordata[self.dim_motor_sensor + 7:self.dim_motor_sensor + 10]
        accel_world = R_body @ accel_body + np.array([0, 0, -9.8])
        self.low_state.acceleration[:] = accel_world.tolist()

        omega_body = self.mj_data.sensordata[self.dim_motor_sensor + 4:self.dim_motor_sensor + 7]

        # Estimated foot contact based on knee joint pos and vel tracking error
        curr_qj_knee_err = self.low_cmd.qj_pos[1] - self.mj_data.sensordata[1]
        curr_dq_knee_err = self.low_cmd.qj_vel[1] - self.mj_data.sensordata[1 + self.num_motor]
        
        # Use a window to filter outliers
        is_estimated_contact = False
        if curr_qj_knee_err > 0.1 or curr_dq_knee_err > 5:
            self.thr_counter += 1
            if self.thr_counter > 4.5:
                is_estimated_contact = True
        else:
            self.thr_counter = 0

        # print(f"GT: {self.low_state.foot_force:.5f}\t EST {curr_q_knee_pose_err:.5f}")

        if is_estimated_contact:
            self.low_state.foot_force = 1
        else:
            self.low_state.foot_force = 0

        # Estimated position and velocity based on IMU and encoder readings
        se_state = self.state_estimator.predict(self.low_state.acceleration)
        foot_pos_body_frame = self.state_estimator.foot_pos_body_frame(self.low_state.qj_pos)
        foot_vel_body_frame = self.state_estimator.foot_vel_body_frame(self.low_state.qj_pos, self.low_state.qj_vel)
        vel_measured = -R_body @ (np.cross(omega_body, foot_pos_body_frame) + foot_vel_body_frame)
        height_measured = -(R_body @ foot_pos_body_frame)[-1]
        if self.low_cmd.contact:
            se_state = self.state_estimator.correct(np.append(height_measured, vel_measured))

        pos_est = se_state[:3]
        vel_est = se_state[3:]
        self.low_state.position[:] = pos_est.tolist()
        self.low_state.velocity[:] = vel_est.tolist()
        self.pos_est = self.low_state.position.copy()
        self.R_body = R_body

        # Handle reset request
        if self.low_cmd.reset_se:
            self.state_estimator.reset(np.array([0, 0, height_measured, *vel_measured]))

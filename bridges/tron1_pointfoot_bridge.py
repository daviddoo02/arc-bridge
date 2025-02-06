import numpy as np
import mujoco
import time

import config
from .lcm2mujuco_bridge import Lcm2MujocoBridge
from utils import *

class Tron1PointfootBridge(Lcm2MujocoBridge):
    def __init__(self, mj_model, mj_data):
        super().__init__(mj_model, mj_data)

        self.torso_name = "base_Link" # body
        self.left_foot_link_name = "foot_L_Link" # body
        self.right_foot_link_name = "foot_R_Link" # body
        self.left_foot_name = "foot_L_collision" # geom
        self.right_foot_name = "foot_R_collision" # geom

        # Override motor offsets
        self.joint_offsets = np.array([0, 0.53, -0.55,  # right leg
                                       0, 0.53, -0.55]) # left leg

    def parse_robot_specific_low_state(self):
        
        # Send inertia matrix and bias force
        temp_inertia_mat = np.zeros((self.mj_model.nv, self.mj_model.nv))
        mujoco.mj_fullM(self.mj_model, temp_inertia_mat, self.mj_data.qM)
        self.low_state.inertia_mat = temp_inertia_mat.tolist()
        self.low_state.bias_force = self.mj_data.qfrc_bias

        # Parse J and dJdq
        # np.set_printoptions(precision=4, suppress=True, linewidth=1000)
        dq = np.concatenate((self.low_state.velocity, self.low_state.omega, self.low_state.qj_vel), axis=0)
        
        # Torso jacobians
        # J_tor
        torso_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, self.torso_name)
        torso_pos = self.mj_data.xpos[torso_id]
        J_tor = np.zeros((6, self.mj_model.nv))
        mujoco.mj_jac(self.mj_model, self.mj_data, J_tor[:3, :], J_tor[3:, :], torso_pos, torso_id)
        self.low_state.J_tor = J_tor.tolist()
        # assert(np.linalg.norm((J_tor @ dq)[3:6] - omega_world) < 1e-6)

        # dJdq_tor
        dJ_tor = np.zeros((6, self.mj_model.nv))
        mujoco.mj_jacDot(self.mj_model, self.mj_data, dJ_tor[:3, :], dJ_tor[3:, :], torso_pos, torso_id)
        dJdq_tor = dJ_tor @ dq
        self.low_state.dJdq_tor = dJdq_tor.tolist()

        # Left foot jacobians
        left_foot_link_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, self.left_foot_link_name)

        # J_lin_foot_L
        left_foot_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_GEOM, self.left_foot_name)
        left_foot_pos = self.mj_data.geom_xpos[left_foot_id]
        J_lin_foot_L = np.zeros((3, self.mj_model.nv))
        mujoco.mj_jac(self.mj_model, self.mj_data, J_lin_foot_L, None, left_foot_pos, left_foot_link_id)

        # dJdq_foot_L
        dJ_foot_L = np.zeros((3, self.mj_model.nv))
        mujoco.mj_jacDot(self.mj_model, self.mj_data, dJ_foot_L, None, left_foot_pos, left_foot_link_id)
        dJdq_foot_L = dJ_foot_L @ dq

        # Right foot jacobians
        right_foot_link_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, self.right_foot_link_name)

        # J_lin_foot_R
        right_foot_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_GEOM, self.right_foot_name)
        right_foot_pos = self.mj_data.geom_xpos[right_foot_id]
        J_lin_foot_R = np.zeros((3, self.mj_model.nv))
        mujoco.mj_jac(self.mj_model, self.mj_data, J_lin_foot_R, None, right_foot_pos, right_foot_link_id)

        # dJdq_foot_R
        dJ_foot_R = np.zeros((3, self.mj_model.nv))
        mujoco.mj_jacDot(self.mj_model, self.mj_data, dJ_foot_R, None, right_foot_pos, right_foot_link_id)
        dJdq_foot_R = dJ_foot_R @ dq

        J_gc = np.concatenate((J_lin_foot_R, J_lin_foot_L), axis=0)
        self.low_state.J_gc = J_gc.tolist()

        dJdq_gc = np.concatenate((dJdq_foot_R, dJdq_foot_L), axis=0)
        self.low_state.dJdq_gc = dJdq_gc.tolist()

        p_gc = np.concatenate((right_foot_pos, left_foot_pos), axis=0)
        self.low_state.p_gc = p_gc.tolist()

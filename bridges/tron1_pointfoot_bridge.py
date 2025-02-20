import mujoco
import numpy as np
import pinocchio as pin

from state_estimators import FloatingBaseLinearStateEstimator
from .lcm2mujuco_bridge import Lcm2MujocoBridge
from utils import *

class Tron1PointfootBridge(Lcm2MujocoBridge):
    def __init__(self, mj_model, mj_data, config):
        super().__init__(mj_model, mj_data, config)

        self.torso_name = "base_Link" # body
        self.left_foot_link_name = "foot_L_Link" # body
        self.right_foot_link_name = "foot_R_Link" # body
        self.left_foot_name = "foot_L_collision" # geom
        self.right_foot_name = "foot_R_collision" # geom

        # Override motor offsets
        self.joint_offsets = np.array([0, 0.53, -0.55,  # right leg
                                       0, 0.53, -0.55]) # left leg

        # Pinocchio model
        self.pin_model = pin.buildModelFromMJCF(self.config.robot_xml_path)
        self.pin_data = self.pin_model.createData()

        # State estimator
        height_init = 0.85
        # Process noise (px, py, pz, vx, vy, vz)
        KF_Q = np.diag([0.002, 0.002, 0.002, 0.02, 0.02, 0.02])
        # Measurement noise (pz, vx, vy, vz)
        KF_R = np.diag([0.001, 0.1, 0.1, 0.1])
        self.KF = FloatingBaseLinearStateEstimator(self.config.dt_sim, KF_Q, KF_R, height_init)
        self.low_state.position = [0, 0, height_init]
        self.low_state.quaternion = [1, 0, 0, 0] # wxyz
        self.low_cmd.contact = [True, True]

        # Visualization
        self.vis_se = True # override default flag
        self.vis_pos_est = np.array([0, 0, height_init])
        self.vis_R_body = np.eye(3)
        self.vis_box_size = [0.1, 0.1, 0.08]

    def update_state_estimation(self):
        # Retrive states from IMU readings
        omega_body = self.low_state.omega
        acc_body = self.low_state.acceleration

        # Retrive states from Pinocchio data
        R_body_to_world = self.pin_data.oMf[self.pin_model.getFrameId("base_Link")].rotation
        pos_world = self.pin_data.oMf[self.pin_model.getFrameId("base_Link")].translation
        vel_body = pin.getFrameVelocity(self.pin_model, self.pin_data, self.pin_model.getFrameId("base_Link"), pin.LOCAL).linear
        vel_world = R_body_to_world @ vel_body
        vel_measured = vel_world

        # Predict based on accelerations
        acc_world = R_body_to_world @ acc_body
        se_state = self.KF.predict(acc_world + np.array([0, 0, -9.81]))

        # Correct based on foot contact
        # if self.low_cmd.contact[0] == 1: # right foot contact
        if self.low_state.foot_force[0] > 0: # right foot contact
            foot_pos_world = self.pin_data.oMf[self.pin_model.getFrameId("foot_R_Link")].translation
            height_measured = (pos_world - foot_pos_world)[-1]
            se_state = self.KF.correct(np.append(height_measured, vel_measured))

        # if self.low_cmd.contact[1] == 1: # left foot contact
        if self.low_state.foot_force[1] > 0: # left foot contact
            foot_pos_world = self.pin_data.oMf[self.pin_model.getFrameId("foot_L_Link")].translation
            height_measured = (pos_world - foot_pos_world)[-1]
            se_state = self.KF.correct(np.append(height_measured, vel_measured))

        # print(f"GT: {self.low_state.position[2]}\t EST {se_state[2]}")
        self.vis_pos_est = se_state[:3]
        self.vis_R_body = R_body_to_world
        # self.low_state.position[2] = se_state[2]
        # self.low_state.velocity[:] = se_state[3:]

        if self.low_cmd.reset_se:
            self.KF.reset(np.array([0, 0, height_measured, *vel_measured]))

    def parse_robot_specific_low_state(self, backend="pinocchio"):
        # Parse common robot states to low_state first
        # Required fields: position, quaternion, velocity, omega, qj_pos, qj_vel
        if backend == "pinocchio":
            self.update_kinematics_and_dynamics_pinocchio()
            self.update_state_estimation()
        else:
            self.update_kinematics_and_dynamics_mujoco()

    def update_kinematics_and_dynamics_mujoco(self):
        # Send inertia matrix and bias force
        temp_inertia_mat = np.zeros((self.mj_model.nv, self.mj_model.nv))
        mujoco.mj_fullM(self.mj_model, temp_inertia_mat, self.mj_data.qM)
        self.low_state.inertia_mat = temp_inertia_mat.tolist()
        self.low_state.bias_force = self.mj_data.qfrc_bias

        # Parse J and dJdq
        # np.set_printoptions(precision=4, suppress=True, linewidth=1000)
        #* dq = [v_world, omega_body, qj_vel]
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

        # Assemble gc jacobians
        J_gc = np.concatenate((J_lin_foot_R, J_lin_foot_L), axis=0)
        self.low_state.J_gc = J_gc.tolist()

        dJdq_gc = np.concatenate((dJdq_foot_R, dJdq_foot_L), axis=0)
        self.low_state.dJdq_gc = dJdq_gc.tolist()

        p_gc = np.concatenate((right_foot_pos, left_foot_pos), axis=0)
        self.low_state.p_gc = p_gc.tolist()

    def update_kinematics_and_dynamics_pinocchio(self):
        # Pinocchio computations
        # np.set_printoptions(precision=4, suppress=True, linewidth=1000)

        #* dq = [v_world, omega_body, qj_vel] in Mujoco conventions
        dq = np.concatenate((self.low_state.velocity, self.low_state.omega, self.low_state.qj_vel), axis=0)

        #* q = [pos, quat(xyzw), qj_pos] in Pinocchio conventions
        pin_q = np.zeros(self.pin_model.nq)
        pin_q[:3] = self.low_state.position.copy()
        pin_q[3:7] = quat_wxyz_to_xyzw(self.low_state.quaternion) # Pinocchio uses xyzw
        pin_q[7:] = np.array(self.low_state.qj_pos) - self.joint_offsets
        # print(f"q: {pin_q}")

        #* v = [v_body, omega_body, qj_vel] in Pinocchio conventions
        R_body_to_world = pin.Quaternion(pin_q[3:7]).toRotationMatrix()
        pin_v = dq.copy()
        pin_v[:3] = R_body_to_world.T @ dq[:3] # velocity in body frame
        # print(f"dq: {pin_v}")

        # Update forward kinematics, dynamics and frame placements in Pinocchio
        pin.computeAllTerms(self.pin_model, self.pin_data, pin_q, pin_v)
        pin.updateFramePlacements(self.pin_model, self.pin_data)

        # H and C
        T_H = np.eye(self.pin_model.nv)
        T_H[:3, :3] = R_body_to_world
        H_prime = T_H @ self.pin_data.M @ T_H.T
        v_temp = np.zeros((self.pin_model.nv))
        v_temp[:3] = np.cross(R_body_to_world @ pin_v[3:6], dq[:3])
        C_prime = T_H @ self.pin_data.nle - H_prime @ v_temp
        # print(f"H:\n{self.pin_data.M}")
        # print(f"H:\n{temp_inertia_mat}")
        # print(f"H_diff:\n{H_prime - temp_inertia_mat}")
        # print(f"C_diff:\n{C_prime - self.mj_data.qfrc_bias}")
        # assert(np.allclose(self.low_state.inertia_mat, H_prime, atol=1e-5))
        # assert(np.allclose(self.low_state.bias_force, C_prime, atol=1e-5))
        self.low_state.inertia_mat = H_prime.tolist()
        self.low_state.bias_force = C_prime.tolist()

        # J_tor
        J_geom_tor = pin.getFrameJacobian(self.pin_model, 
                                          self.pin_data, 
                                          self.pin_model.getFrameId("base_Link"), 
                                          pin.WORLD) # J_G maps body velocity to world twist
        J_G_to_A = np.eye(3, 6)
        J_G_to_A[:3, 3:] = - pin.skew(pin_q[:3])
        J_lin_tor = J_G_to_A @ J_geom_tor @ T_H.T
        J_tor = np.concatenate((J_lin_tor, J_geom_tor[3:, :]), axis=0)
        # print(f"J_geom_tor:\n{J_geom_tor}")
        # print(f"J_tor:\n{J_tor}")
        # assert(np.allclose(self.low_state.J_tor, J_tor, atol=1e-5))
        self.low_state.J_tor = J_tor.tolist()

        # dJdq_tor
        dJ_geom_tor = pin.frameJacobianTimeVariation(self.pin_model, 
                                                self.pin_data,
                                                pin_q,
                                                pin_v,
                                                self.pin_model.getFrameId("base_Link"), 
                                                pin.WORLD)
        dT_H = np.zeros((self.pin_model.nv, self.pin_model.nv))
        dT_H[:3, :3] = - pin.skew(pin_v[3:6]) @ R_body_to_world.T
        dJ_lin_tor = J_G_to_A @ (dJ_geom_tor @ T_H.T + J_geom_tor @ dT_H) #- pin.skew(dq[:3]) @ J_geom_tor[3:, :] @ T_H.T
        dJ_tor = np.concatenate((dJ_lin_tor, dJ_geom_tor[3:, :]), axis=0)
        dJdq_tor = dJ_tor @ dq
        # assert(np.allclose(self.low_state.dJdq_tor, dJdq_tor, atol=1e-5))
        self.low_state.dJdq_tor = dJdq_tor.tolist()

        # J_lin_foot_L
        J_geom_foot_L = pin.getFrameJacobian(self.pin_model, 
                                             self.pin_data, 
                                             self.pin_model.getFrameId("foot_L_Link"), 
                                             pin.WORLD)
        left_foot_pos = self.pin_data.oMf[self.pin_model.getFrameId("foot_L_Link")].translation
        J_lin_foot_L_G_to_A = np.eye(3, 6)
        J_lin_foot_L_G_to_A[:, 3:] = -pin.skew(left_foot_pos)
        J_lin_foot_L = J_lin_foot_L_G_to_A @ J_geom_foot_L @ T_H.T

        # dJdq_foot_L
        dJ_geom_foot_L = pin.frameJacobianTimeVariation(self.pin_model, 
                                                        self.pin_data,
                                                        pin_q,
                                                        pin_v,
                                                        self.pin_model.getFrameId("foot_L_Link"), 
                                                        pin.WORLD)
        dJ_lin_foot_L = J_lin_foot_L_G_to_A @ (dJ_geom_foot_L @ T_H.T + J_geom_foot_L @ dT_H)
        dJdq_foot_L = dJ_lin_foot_L @ dq

        # J_lin_foot_R
        J_geom_foot_R = pin.getFrameJacobian(self.pin_model, 
                                             self.pin_data, 
                                             self.pin_model.getFrameId("foot_R_Link"), 
                                             pin.WORLD)
        right_foot_pos = self.pin_data.oMf[self.pin_model.getFrameId("foot_R_Link")].translation
        J_lin_foot_R_G_to_A = np.eye(3, 6)
        J_lin_foot_R_G_to_A[:, 3:] = -pin.skew(right_foot_pos)
        J_lin_foot_R = J_lin_foot_R_G_to_A @ J_geom_foot_R @ T_H.T

        # dJdq_foot_R
        dJ_geom_foot_R = pin.frameJacobianTimeVariation(self.pin_model, 
                                                        self.pin_data,
                                                        pin_q,
                                                        pin_v,
                                                        self.pin_model.getFrameId("foot_R_Link"), 
                                                        pin.WORLD)
        dJ_lin_foot_R = J_lin_foot_R_G_to_A @ (dJ_geom_foot_R @ T_H.T + J_geom_foot_R @ dT_H)
        dJdq_foot_R = dJ_lin_foot_R @ dq

        # Assemble gc jacobians
        # J_gc
        J_gc = np.concatenate((J_lin_foot_R, J_lin_foot_L), axis=0)
        # assert(np.allclose(self.low_state.J_gc, J_gc, atol=1e-5))
        self.low_state.J_gc = J_gc.tolist()

        # dJdq_gc
        dJdq_gc = np.concatenate((dJdq_foot_R, dJdq_foot_L), axis=0)
        # assert(np.allclose(self.low_state.dJdq_gc, dJdq_gc, atol=1e-5))
        self.low_state.dJdq_gc = dJdq_gc.tolist()

        # p_gc
        p_gc = np.concatenate((right_foot_pos, left_foot_pos), axis=0)
        # assert(np.allclose(self.low_state.p_gc, p_gc, atol=1e-5))
        self.low_state.p_gc = p_gc.tolist()

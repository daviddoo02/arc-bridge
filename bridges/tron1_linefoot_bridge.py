import mujoco
import numpy as np
import pinocchio as pin

from state_estimators import FloatingBaseLinearStateEstimator, MovingWindowFilter
from .lcm2mujuco_bridge import Lcm2MujocoBridge
from lcm_types.robot_lcm import tron1_linefoot_state_t, tron1_linefoot_control_t
from utils import *

class Tron1LinefootBridge(Lcm2MujocoBridge):
    def __init__(self, mj_model, mj_data, config):
        super().__init__(mj_model, mj_data, config)

        # Override motor offsets (rad)
        self.joint_offsets = np.array([0, 0.53 - 0.06, -0.55 - 0.54, 0,  
                                       0, 0.53 - 0.06, -0.55 - 0.54, 0])

        self.torso_name = "base_Link" # body
        self.left_foot_link_name = "ankle_L_Link" # body
        self.right_foot_link_name = "ankle_R_Link" # body
        self.left_heel_name = "heel_L_site" # site
        self.left_toe_name = "toe_L_site" # site
        self.right_heel_name = "heel_R_site" # site
        self.right_toe_name = "toe_R_site" # site
        self.gc_names = [self.right_heel_name, 
                         self.right_toe_name, 
                         self.left_heel_name, 
                         self.left_toe_name] # name of links for ground contact
        self.num_legs = 2

        # Pinocchio model
        self.pin_model = pin.buildModelFromMJCF(self.config.robot_xml_path)
        self.pin_data = self.pin_model.createData()

    def parse_robot_specific_low_state(self, backend="pinocchio"):
        # Parse common robot states to low_state first
        # Required fields: position, quaternion, velocity, omega, qj_pos, qj_vel

        # Update low_state.position[2] and low_state.velocity
        # TODO implement state estimation
        # self.update_state_estimation()

        if backend == "pinocchio":
            # Overwrite position to (0, 0, pz)
            self.low_state.position[0] = 0
            self.low_state.position[1] = 0

            #* q = [pos, quat(xyzw), qj_pos] in Pinocchio conventions
            pin_q = np.zeros(self.pin_model.nq)
            pin_q[:3] = self.low_state.position.copy()
            pin_q[3:7] = quat_wxyz_to_xyzw(self.low_state.quaternion) # Pinocchio uses xyzw
            pin_q[7:] = np.array(self.low_state.qj_pos) - self.joint_offsets
            # print(f"q: {pin_q}")

            #* v = [v_body, omega_body, qj_vel] in Pinocchio conventions
            R_body_to_world = pin.Quaternion(pin_q[3:7]).toRotationMatrix()
            pin_v = np.concatenate((R_body_to_world.T @ self.low_state.velocity, self.low_state.omega, self.low_state.qj_vel), axis=0)
            # print(f"dq: {pin_v}")

            # Update forward kinematics, dynamics and frame placements in Pinocchio
            pin.computeAllTerms(self.pin_model, self.pin_data, pin_q, pin_v)
            pin.updateFramePlacements(self.pin_model, self.pin_data)

            # Retrive states from Pinocchio data
            self.update_kinematics_and_dynamics_pinocchio(pin_q, pin_v)
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
        left_ankle_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, self.left_foot_link_name)

        # J_lin_heel_L
        left_heel_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_SITE, self.left_heel_name)
        left_heel_pos = self.mj_data.site_xpos[left_heel_id]
        J_lin_heel_L = np.zeros((3, self.mj_model.nv))
        mujoco.mj_jac(self.mj_model, self.mj_data, J_lin_heel_L, None, left_heel_pos, left_ankle_id)

        # dJdq_heel_L
        dJ_heel_L = np.zeros((3, self.mj_model.nv))
        mujoco.mj_jacDot(self.mj_model, self.mj_data, dJ_heel_L, None, left_heel_pos, left_ankle_id)
        dJdq_heel_L = dJ_heel_L @ dq

        # J_lin_toe_L
        left_toe_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_SITE, self.left_toe_name)
        left_toe_pos = self.mj_data.site_xpos[left_toe_id]
        J_lin_toe_L = np.zeros((3, self.mj_model.nv))
        mujoco.mj_jac(self.mj_model, self.mj_data, J_lin_toe_L, None, left_toe_pos, left_ankle_id)

        # dJdq_toe_L
        dJ_toe_L = np.zeros((3, self.mj_model.nv))
        mujoco.mj_jacDot(self.mj_model, self.mj_data, dJ_toe_L, None, left_heel_pos, left_ankle_id)
        dJdq_toe_L = dJ_toe_L @ dq

        # Right foot jacobians
        right_ankle_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, self.right_foot_link_name)

        # J_lin_heel_R
        right_heel_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_SITE, self.right_heel_name)
        right_heel_pos = self.mj_data.site_xpos[right_heel_id]
        J_lin_heel_R = np.zeros((3, self.mj_model.nv))
        mujoco.mj_jac(self.mj_model, self.mj_data, J_lin_heel_R, None, right_heel_pos, right_ankle_id)

        # dJdq_heel_R
        dJ_heel_R = np.zeros((3, self.mj_model.nv))
        mujoco.mj_jacDot(self.mj_model, self.mj_data, dJ_heel_R, None, right_heel_pos, right_ankle_id)
        dJdq_heel_R = dJ_heel_R @ dq

        # J_lin_toe_R
        right_toe_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_SITE, self.right_toe_name)
        right_toe_pos = self.mj_data.site_xpos[right_toe_id]
        J_lin_toe_R = np.zeros((3, self.mj_model.nv))
        mujoco.mj_jac(self.mj_model, self.mj_data, J_lin_toe_R, None, right_toe_pos, right_ankle_id)

        # dJdq_toe_R
        dJ_toe_R = np.zeros((3, self.mj_model.nv))
        mujoco.mj_jacDot(self.mj_model, self.mj_data, dJ_toe_R, None, right_heel_pos, right_ankle_id)
        dJdq_toe_R = dJ_toe_R @ dq

        # Assemble gc jacobians
        J_gc = np.concatenate((J_lin_heel_R, J_lin_toe_R, J_lin_heel_L, J_lin_toe_L), axis=0)
        self.low_state.J_gc = J_gc.tolist()

        dJdq_gc = np.concatenate((dJdq_heel_R, dJdq_toe_R, dJdq_heel_L, dJdq_toe_L), axis=0)
        self.low_state.dJdq_gc = dJdq_gc.tolist()

        p_gc = np.concatenate((right_heel_pos, right_toe_pos, left_heel_pos, left_toe_pos), axis=0)
        self.low_state.p_gc = p_gc.tolist()

    def update_kinematics_and_dynamics_pinocchio(self, pin_q, pin_v):
        # Pinocchio computations
        R_body_to_world = pin.Quaternion(pin_q[3:7]).toRotationMatrix()
        #* dq = [v_world, omega_body, qj_vel] in Mujoco conventions
        dq = np.concatenate((self.low_state.velocity, self.low_state.omega, self.low_state.qj_vel), axis=0)

        # Mass matrix and bias force
        transform_mat = np.eye(self.pin_model.nv)
        transform_mat[:3, :3] = R_body_to_world
        H_prime = transform_mat @ self.pin_data.M @ transform_mat.T
        v_temp = np.zeros((self.pin_model.nv))
        v_temp[:3] = np.cross(R_body_to_world @ pin_v[3:6], dq[:3])
        C_prime = transform_mat @ self.pin_data.nle - H_prime @ v_temp

        self.low_state.inertia_mat = H_prime.tolist()
        self.low_state.bias_force = C_prime.tolist()

        # Torso Jacobians and their derivatives
        # J_tor
        J_geom_tor = pin.getFrameJacobian(self.pin_model, 
                                          self.pin_data, 
                                          self.pin_model.getFrameId(self.torso_name), 
                                          pin.WORLD) # J_G maps body velocity to world twist
        J_G_to_A = np.eye(3, 6)
        J_G_to_A[:3, 3:] = - pin.skew(pin_q[:3])
        J_lin_tor = J_G_to_A @ J_geom_tor @ transform_mat.T
        J_tor = np.concatenate((J_lin_tor, J_geom_tor[3:, :]), axis=0)

        self.low_state.J_tor = J_tor.tolist()

        # dJdq_tor
        dJ_geom_tor = pin.frameJacobianTimeVariation(self.pin_model, 
                                                self.pin_data,
                                                pin_q,
                                                pin_v,
                                                self.pin_model.getFrameId(self.torso_name), 
                                                pin.WORLD)
        transform_mat_derivative = np.zeros((self.pin_model.nv, self.pin_model.nv))
        transform_mat_derivative[:3, :3] = - pin.skew(pin_v[3:6]) @ R_body_to_world.T
        dJ_lin_tor = J_G_to_A @ (dJ_geom_tor @ transform_mat.T + J_geom_tor @ transform_mat_derivative) #- pin.skew(dq[:3]) @ J_geom_tor[3:, :] @ transform_mat.T
        dJ_tor = np.concatenate((dJ_lin_tor, dJ_geom_tor[3:, :]), axis=0)
        dJdq_tor = dJ_tor @ dq

        self.low_state.dJdq_tor = dJdq_tor.tolist()

        # Contact Jacobians and their derivatives
        J_gc = np.zeros((12, self.pin_model.nv))
        dJdq_gc = np.zeros((12, ))
        p_gc = np.zeros((12, ))
        for idx, name in enumerate(self.gc_names):
            J_geom = pin.getFrameJacobian(self.pin_model,
                                          self.pin_data, 
                                          self.pin_model.getFrameId(name), 
                                          pin.WORLD)
            link_pos = self.pin_data.oMf[self.pin_model.getFrameId(name)].translation
            J_G_to_A = np.eye(3, 6)
            J_G_to_A[:, 3:] = -pin.skew(link_pos)
            J_lin = J_G_to_A @ J_geom @ transform_mat.T

            dJ_geom = pin.frameJacobianTimeVariation(self.pin_model,
                                                    self.pin_data,
                                                    pin_q,
                                                    pin_v,
                                                    self.pin_model.getFrameId(name), 
                                                    pin.WORLD)
            dJ_lin = J_G_to_A @ (dJ_geom @ transform_mat.T + J_geom @ transform_mat_derivative)
            dJdq = dJ_lin @ dq

            J_gc[idx*3:(idx+1)*3, :] = J_lin
            dJdq_gc[idx*3:(idx+1)*3] = dJdq
            p_gc[idx*3:(idx+1)*3] = link_pos

        self.low_state.J_gc = J_gc.tolist()
        self.low_state.dJdq_gc = dJdq_gc.tolist()
        self.low_state.p_gc = p_gc.tolist()

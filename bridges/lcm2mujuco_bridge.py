from abc import abstractmethod
import time
import lcm
import mujoco
import numpy as np

from threading import Thread

import config
from state_estimator import HopperStateEstimator
from lcm_types.robot_lcm import *
from utils import *

MOTOR_SENSOR_NUM = 3 # pos, vel, torque

class Lcm2MujocoBridge:

    def __init__(self, mj_model, mj_data):
        self.mj_model = mj_model
        self.mj_data = mj_data

        self.topic_state = config.robot_state_topic
        self.topic_cmd = config.robot_cmd_topic

        self.num_motor = self.mj_model.nu
        self.num_body_state = self.mj_model.nq - self.num_motor
        self.num_joint_state = self.num_motor
        self.dim_motor_sensor = MOTOR_SENSOR_NUM * self.num_motor
        self.have_imu = False
        self.have_frame_sensor = False
        self.have_foot_sensor = False
        self.dt = self.mj_model.opt.timestep
        self.start_time = time.time_ns()

        # Check sensors
        for i in range(self.dim_motor_sensor, self.mj_model.nsensor):
            name = mujoco.mj_id2name(self.mj_model,
                                     mujoco._enums.mjtObj.mjOBJ_SENSOR, i)
            if name == "imu_quat":
                self.have_imu = True
            if name == "frame_pos":
                self.have_frame_sensor = True
            if name == "foot_force":
                # TODO make this more general
                self.have_foot_sensor = True

        self.print_scene_info()

        # LCM messages
        self.lc = lcm.LCM()
        self.low_state = eval(self.topic_state+"_t")()
        self.low_cmd = eval(self.topic_cmd+"_t")()
        self.lcm_handle_thread = None

        self.low_cmd_received = False
        self.is_running = None

        # Motor offsets
        self.motor_offset = np.zeros(self.num_motor)

    def start_lcm_thread(self):
        self.is_running = True
        self.lcm_handle_thread = Thread(target=self.lcmHandleThread)
        self.lcm_handle_thread.start()

    def stop_lcm_thread(self):
        self.is_running = False
        self.lcm_handle_thread.join()
        print("LCM thread exited")

    def register_low_state_subscriber(self, topic=None):
        if topic:
            assert(topic == "HOPPER_STATE")
            self.low_state_suber = self.lc.subscribe("HOPPER_STATE", self.lowStateHandler)
        else:
            self.low_state_suber = self.lc.subscribe(self.topic_state, self.lowStateHandler)
        self.low_state_suber.set_queue_capacity(1)

    def register_low_cmd_subscriber(self):
        self.low_cmd_suber = self.lc.subscribe(self.topic_cmd, self.lowCmdHandler)
        self.low_cmd_suber.set_queue_capacity(1)

    def lcmHandleThread(self):
        while self.is_running:
            self.lc.handle_timeout(0)

    def lowCmdHandler(self, channel, data):
        if self.mj_data != None:
            self.low_cmd = eval(self.topic_cmd+"_t").decode(data)
            self.low_cmd_received = True

    def lowStateHandler(self, channel, data):
        if self.mj_data == None:
            return
        msg = eval(self.topic_state+"_t").decode(data)
        #! The following is used for hopper only
        self.mj_data.qpos[0] = msg.position[0]
        self.mj_data.qpos[1] = msg.position[2]-0.47 # offsetted z joint height in xml
        self.mj_data.qpos[2] = msg.rpy[1]
        self.mj_data.qpos[3:5] = msg.qj_pos
        self.mj_data.qvel[:] = 0
        # self.mj_data.act[:] = False
        # self.mj_data.qacc_warmstart[:] = 0
        # self.mj_data.ctrl[:] = 0

    def update_motor_cmd(self):
        for i in range(self.num_motor):
            ctrlrange = self.mj_model.actuator_ctrlrange[i]
            motor_tau = self.low_cmd.qj_tau[i] +\
                        self.low_cmd.kp[i] * (self.low_cmd.qj_pos[i] - self.mj_data.sensordata[i] - self.motor_offset[i]) +\
                        self.low_cmd.kd[i] * (self.low_cmd.qj_vel[i] - self.mj_data.sensordata[i + self.num_motor])
            self.mj_data.ctrl[i] = np.clip(motor_tau, ctrlrange[0], ctrlrange[1])

    def parse_common_low_state(self):
        if self.mj_data == None:
            return -1
        
        for i in range(self.num_motor):
            self.low_state.qj_pos[i] = self.mj_data.sensordata[i]
            self.low_state.qj_vel[i] = self.mj_data.sensordata[i + self.num_motor]
            self.low_state.qj_tau[i] = self.mj_data.sensordata[i + 2 * self.num_motor]

        if self.have_frame_sensor:
            # Ground truth position and velocity readings in the world frame
            self.low_state.position[0] = self.mj_data.sensordata[self.dim_motor_sensor + 10]
            self.low_state.position[1] = self.mj_data.sensordata[self.dim_motor_sensor + 11]
            self.low_state.position[2] = self.mj_data.sensordata[self.dim_motor_sensor + 12]

            self.low_state.velocity[0] = self.mj_data.sensordata[self.dim_motor_sensor + 13]
            self.low_state.velocity[1] = self.mj_data.sensordata[self.dim_motor_sensor + 14]
            self.low_state.velocity[2] = self.mj_data.sensordata[self.dim_motor_sensor + 15]

        if self.have_foot_sensor:
            # Ground truth contact sensing
            # TODO make this more general
            self.low_state.foot_force = self.mj_data.sensordata[self.dim_motor_sensor + 16]

        if self.have_imu:
            self.low_state.quaternion[0] = self.mj_data.sensordata[self.dim_motor_sensor + 0]
            self.low_state.quaternion[1] = self.mj_data.sensordata[self.dim_motor_sensor + 1]
            self.low_state.quaternion[2] = self.mj_data.sensordata[self.dim_motor_sensor + 2]
            self.low_state.quaternion[3] = self.mj_data.sensordata[self.dim_motor_sensor + 3]
            
            quat = Quaternion(*self.low_state.quaternion)
            rpy = quat_to_rpy(quat)
            self.low_state.rpy[0] = rpy[0]
            self.low_state.rpy[1] = rpy[1]
            self.low_state.rpy[2] = rpy[2]

            # Body frame angular rate and linear acceleration
            self.low_state.omega[:] = self.mj_data.sensordata[self.dim_motor_sensor + 4:self.dim_motor_sensor + 7]
            self.low_state.acceleration[:] = self.mj_data.sensordata[self.dim_motor_sensor + 7:self.dim_motor_sensor + 10]

            # self.mj_data.qvel[3:6] # this is Eular angle rate != omega_body
            # self.low_state.omega[:] = omega_body.tolist()
        return 0

    def publish_low_state(self):
        if self.parse_common_low_state() < 0:
            return
        
        self.parse_robot_specific_low_state()

        # Encode and publish robot states
        self.low_state.timestamp = time.time_ns()
        self.lc.publish(self.topic_state, self.low_state.encode())

    @abstractmethod
    def parse_robot_specific_low_state(self):
        pass

    def print_scene_info(self):
        print(" ")

        print("<<------------- Link ------------->> ")
        for i in range(self.mj_model.nbody):
            name = mujoco.mj_id2name(self.mj_model,
                                     mujoco._enums.mjtObj.mjOBJ_BODY, i)
            if name:
                print(f"link_index: {i}, name: {name}, mass: {self.mj_model.body_mass[i]:.4f}, inertia: {self.mj_model.body_inertia[i]}")
        print(" ")

        print("<<------------- Joint ------------->> ")
        for i in range(self.mj_model.njnt):
            name = mujoco.mj_id2name(self.mj_model,
                                     mujoco._enums.mjtObj.mjOBJ_JOINT, i)
            if name:
                print("joint_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Actuator ------------->>")
        for i in range(self.mj_model.nu):
            name = mujoco.mj_id2name(self.mj_model,
                                     mujoco._enums.mjtObj.mjOBJ_ACTUATOR, i)
            if name:
                print("actuator_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Sensor ------------->>")
        index = 0
        for i in range(self.mj_model.nsensor):
            name = mujoco.mj_id2name(self.mj_model,
                                     mujoco._enums.mjtObj.mjOBJ_SENSOR, i)
            if name:
                print("sensor_index:", index, ", name:", name, ", dim:",
                      self.mj_model.sensor_dim[i])
            index = index + self.mj_model.sensor_dim[i]
        print(" ")

import os
import signal
import sys
from threading import Thread
import time
from functools import partial
import limxsdk.robot.Rate as Rate
import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes
import lcm
import numpy as np

# Add the parent directory to the path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from lcm_types.robot_lcm import tron1_pointfoot_state_t, tron1_pointfoot_control_t
from utils import *


JOINT_OFFSETS_HIGH = np.array([0, 0.53 - 0.06, -0.55 - 0.54,  # right leg
                                0, 0.53 - 0.06, -0.55 - 0.54]) # left leg


class RobotReceiver:
    def __init__(self):
        self.imu = None
        self.robot_state = None
        self.low_cmd = None
        self.is_running = True

    # Callback function for receiving imu
    def imuDataCallback(self, imu: datatypes.ImuData):
        self.imu = imu

    # Callback function for receiving robot state
    def robotStateCallback(self, robot_state: datatypes.RobotState):
        self.robot_state = robot_state

    # Callback function for receiving sensor joy data
    def sensorJoyCallback(self, sensor_joy: datatypes.SensorJoy):
        print("\n------\nsensor_joy:" + \
              "\n  stamp: " + str(sensor_joy.stamp) + \
              "\n  axes: " + str(sensor_joy.axes) + \
              "\n  buttons: " + str(sensor_joy.buttons))

    # Callback function for receiving diagnostic value
    def diagnosticValueCallback(self, diagnostic_value: datatypes.DiagnosticValue):
        print("\n------\ndiagnostic_value:" + \
              "\n  stamp: " + str(diagnostic_value.stamp) + \
              "\n  name: " + diagnostic_value.name + \
              "\n  level: " + str(diagnostic_value.level) + \
              "\n  code: " + str(diagnostic_value.code) + \
              "\n  message: " + diagnostic_value.message)

    def lowCmdHandler(self, channel, data):
        self.low_cmd = tron1_pointfoot_control_t.decode(data)

    def lcmHandleThread(self):
        while self.is_running:
            lc.handle_timeout(0)

def sdk_state_to_lcm_state(robot_state: datatypes.RobotState, low_state: tron1_pointfoot_state_t):
    low_state.qj_pos[:3] = robot_state.q[3:]
    low_state.qj_pos[3:] = robot_state.q[:3]
    low_state.qj_pos[1] *=-1
    low_state.qj_pos[5] *=-1
    low_state.qj_vel[:3] = robot_state.dq[3:]
    low_state.qj_vel[3:] = robot_state.dq[:3]
    low_state.qj_vel[1] *=-1
    low_state.qj_vel[5] *=-1
    low_state.qj_tau[:3] = robot_state.tau[3:]
    low_state.qj_tau[3:] = robot_state.tau[:3]
    low_state.qj_tau[1] *=-1
    low_state.qj_tau[5] *=-1

def lcm_cmd_to_sdk_cmd(low_cmd: tron1_pointfoot_control_t, robot_cmd: datatypes.RobotCmd):
    robot_cmd.mode = [0.0 for _ in range(motor_number)]
    qj_pos_lower = (np.array((low_cmd.qj_pos)) - JOINT_OFFSETS_HIGH).tolist()
    robot_cmd.q[:3] = qj_pos_lower[3:]
    robot_cmd.q[3:] = qj_pos_lower[:3]
    robot_cmd.q[2] *=-1
    robot_cmd.q[4] *=-1
    robot_cmd.dq[:3] = low_cmd.qj_vel[3:]
    robot_cmd.dq[3:] = low_cmd.qj_vel[:3]
    robot_cmd.dq[2] *=-1
    robot_cmd.dq[4] *=-1
    robot_cmd.tau[:3] = low_cmd.qj_tau[3:]
    robot_cmd.tau[3:] = low_cmd.qj_tau[:3]
    robot_cmd.tau[2] *=-1
    robot_cmd.tau[4] *=-1
    robot_cmd.Kp[:3] = low_cmd.kp[3:]
    robot_cmd.Kp[3:] = low_cmd.kp[:3]
    robot_cmd.Kd[:3] = low_cmd.kd[3:]
    robot_cmd.Kd[3:] = low_cmd.kd[:3]

if __name__ == '__main__':
    os.system("export ROBOT_TYPE=PF_TRON1A")

    # Create a Robot instance of type PointFoot
    robot = Robot(RobotType.PointFoot)

    robot_ip = "10.192.1.2"

    # Initialize the robot with robot_ip
    if not robot.init(robot_ip):
        sys.exit()

    # Get motor number information
    motor_number = robot.getMotorNumber()
    print(f"motor number: {motor_number}")

    # Create an instance of RobotReceiver to handle callbacks
    receiver = RobotReceiver()

    # Create partial functions for callbacks
    imuDataCallback = partial(receiver.imuDataCallback)
    robotStateCallback = partial(receiver.robotStateCallback)
    # sensorJoyCallback = partial(receiver.sensorJoyCallback)
    # diagnosticValueCallback = partial(receiver.diagnosticValueCallback)

    # Subscribe to robot state, sensor joy, and diagnostic value topics
    robot.subscribeImuData(imuDataCallback)
    robot.subscribeRobotState(robotStateCallback)
    # robot.subscribeSensorJoy(sensorJoyCallback)
    # robot.subscribeDiagnosticValue(diagnosticValueCallback)
    
    # Main loop to continuously publish robot commands
    rate = Rate(1000) # 1000 Hz


    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
    low_cmd_suber = lc.subscribe("TRON1_CONTROL", receiver.lowCmdHandler)
    low_cmd_suber.set_queue_capacity(1)

    lc_thread = Thread(target=receiver.lcmHandleThread, daemon=True)
    lc_thread.start()
    
    topic_state = "tron1_pointfoot_state" # state to mujoco bridge
    topic_control = "TRON1_CONTROL" # control from MATLAB
    polarity = [1, -1,  1,
                1,  1, -1]

    cmd_msg = datatypes.RobotCmd()
    # Set default values for control mode, joint positions, velocities, torques, Kp, and Kd
        # motor mode：0: torque； 1：speed； 2：position; default：0
    cmd_msg.mode = [2.0 for _ in range(motor_number)]
    cmd_msg.q = [0.0 for _ in range(motor_number)]
    cmd_msg.dq = [0.0 for _ in range(motor_number)]
    cmd_msg.tau = [0.0 for _ in range(motor_number)]
    cmd_msg.Kp = [40.0 for _ in range(motor_number)]
    cmd_msg.Kd = [2.0 for _ in range(motor_number)]

    # Wait for the robot to initialize
    while receiver.robot_state is None:
        time.sleep(0.1)
        pass

    q_init = np.array(receiver.robot_state.q)
    q_target = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    num_init_steps = 4000

    print("Enter initial pose...")
    for step in range(num_init_steps):
        alpha = step / num_init_steps
        for i in range(motor_number):
            cmd_msg.q[i] = q_init[i] * (1 - alpha) + q_target[i] * alpha

        cmd_msg.stamp = time.time_ns()  # Set the timestamp
        robot.publishRobotCmd(cmd_msg)  # Publish the robot command
        rate.sleep()

    # ====================================== Debug Start
    # cmd_msg.mode[:] = [0] * 6
    # cmd_msg.Kp[:] = [0] * 6
    # cmd_msg.Kd[:] = [0] * 6
    # print("Send zero torque for one leg")
    # for i in range(100):
    #     cmd_msg.stamp = time.time_ns()
    #     robot.publishRobotCmd(cmd_msg)
    #     rate.sleep()
    # ====================================== Debug End

    print("Start to publish robot command...")

    # Capture Ctrl+C signal
    signal.signal(signal.SIGINT, signal.default_int_handler)
    try:
        while True:
            # Parse the received low_cmd
            low_cmd = receiver.low_cmd
            if low_cmd is not None:
                max_tau = 50
                cmd_msg = datatypes.RobotCmd()
                lcm_cmd_to_sdk_cmd(low_cmd, cmd_msg)
                cmd_msg.stamp = time.time_ns()
                tau_wbc = np.array(cmd_msg.tau).clip(-max_tau, max_tau)
                tau_wbc[[0, 1, 3, 4]] = 0
                cmd_msg.tau = tau_wbc.tolist()
                robot.publishRobotCmd(cmd_msg)  # Publish the robot command
                low_cmd = None

            # Parse the received sensor readings
            low_state = tron1_pointfoot_state_t()
            if receiver.robot_state is not None:
                sdk_state_to_lcm_state(receiver.robot_state, low_state)

            if receiver.imu is not None:
                low_state.acceleration = receiver.imu.acc
                low_state.omega = receiver.imu.gyro
                low_state.quaternion = receiver.imu.quat
                quat = Quaternion(*low_state.quaternion)
                rpy = quat_to_rpy(quat)
                low_state.rpy[0] = rpy[0]
                low_state.rpy[1] = rpy[1]
                low_state.rpy[2] = rpy[2]

            lc.publish(topic_state, low_state.encode())
            rate.sleep()  # Control loop frequency
    except:
        cmd_msg = datatypes.RobotCmd()
        cmd_msg.stamp = time.time_ns()  # Set the timestamp
        # Set default values for control mode, joint positions, velocities, torques, Kp, and Kd
        # motor mode：0: torque； 1：speed； 2：position; default：0
        cmd_msg.mode = [0.0 for _ in range(motor_number)]
        cmd_msg.q = [0.0 for _ in range(motor_number)]
        cmd_msg.dq = [0.0 for _ in range(motor_number)]
        cmd_msg.tau = [0.0 for _ in range(motor_number)]
        cmd_msg.Kp = [0.0 for _ in range(motor_number)]
        cmd_msg.Kd = [0.0 for _ in range(motor_number)]
        print("Enter zero torque mode!!!!!!!!")
        for i in range(100):
            robot.publishRobotCmd(cmd_msg)  # Publish the robot command
            rate.sleep()

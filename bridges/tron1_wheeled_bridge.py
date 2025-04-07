import mujoco
import numpy as np
# import pinocchio as pin

from state_estimators import FloatingBaseLinearStateEstimator, MovingWindowFilter
from .lcm2mujuco_bridge import Lcm2MujocoBridge
from lcm_types.robot_lcm import tron1_wheeled_state_t, tron1_wheeled_control_t
from utils import *

class Tron1WheeledBridge(Lcm2MujocoBridge):
    def __init__(self, mj_model, mj_data, config):
        super().__init__(mj_model, mj_data, config)
        # Override motor offsets (rad)
        self.joint_offsets = np.array([0, 0.53 - 0.06, -0.55 - 0.54, 0,  
                                       0, 0.53 - 0.06, -0.55 - 0.54, 0])

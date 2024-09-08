import mujoco
import mujoco.viewer

import time
from threading import Thread
import threading
import argparse

from lcm2mujuco_bridge import Lcm2MujocoBridge
import config

parser = argparse.ArgumentParser()
parser.add_argument("--replay", action="store_true", help="replay state trajectory in Mujoco")
args = parser.parse_args()

# Initialize Mujoco
mj_model = mujoco.MjModel.from_xml_path(config.robot_xml_path)
mj_data = mujoco.MjData(mj_model)
mj_model.opt.timestep = config.dt_sim
viewer = mujoco.viewer.launch_passive(mj_model, mj_data)
bridge = Lcm2MujocoBridge(mj_model, mj_data, 
                          config.robot_state_topic, config.robot_cmd_topic,
                          args.replay)
if args.replay:
    # Disable gravity
    mj_model.opt.gravity[:] = 0
    # Disable contact for all geoms
    for i in range(mj_model.ngeom):
        mj_model.geom_contype[i] = 0  # Disable contact type
        mj_model.geom_conaffinity[i] = 0  # Disable contact affinity

# Separate simulation and visualization threads
locker = threading.Lock()

def SimulationThread():
    global mj_data, mj_model
    while viewer.is_running():
        step_start = time.perf_counter()
        locker.acquire()
        mujoco.mj_step(mj_model, mj_data)
        locker.release()
        if not args.replay:
            bridge.publishLowState()
        time_until_next_step = mj_model.opt.timestep - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

def PhysicsViewerThread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.dt_viewer)


if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()

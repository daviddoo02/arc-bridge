import mujoco
import mujoco.viewer

import time
from threading import Thread
import threading
import argparse

from lcm2mujuco_bridge import Lcm2MujocoBridge
import config

parser = argparse.ArgumentParser()
parser.add_argument("--replay", action="store_true", help="replay state trajectory from LCM")
parser.add_argument("--debug", action="store_true", help="debug mode")
parser.add_argument("--track", action="store_true", help="make camera track the robot's motion")
parser.add_argument("--block", action="store_true", help="block the main thread")
args = parser.parse_args()

# Initialize Mujoco
mj_model = mujoco.MjModel.from_xml_path(config.robot_xml_path)
mj_data = mujoco.MjData(mj_model)

# Modify MjOption
mj_model.opt.timestep = config.dt_sim

if args.replay:
    # Disable gravity and all constraints (e.g. contact, friction ...)
    mj_model.opt.disableflags = mujoco.mjtDisableBit.mjDSBL_CONSTRAINT | mujoco.mjtDisableBit.mjDSBL_GRAVITY
elif args.debug:
    mj_model.opt.disableflags = mujoco.mjtDisableBit.mjDSBL_GRAVITY

viewer = mujoco.viewer.launch_passive(mj_model, mj_data)
if args.track:
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = 0

# Enable visualization flags
# viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_INERTIA] = True
viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True

# Initialize bridge
bridge = Lcm2MujocoBridge(mj_model, mj_data)
if args.replay:
    bridge.register_low_state_subscriber()
else:
    bridge.register_low_cmd_subscriber()

# Separate simulation and visualization threads
locker = threading.Lock()

# Reset data keyframe
mujoco.mj_resetDataKeyframe(mj_model, mj_data, 0)
mujoco.mj_step(mj_model, mj_data)

def SimulationThread():
    global mj_data, mj_model
    while viewer.is_running():
        if args.block and not bridge.low_cmd_received:
            bridge.publish_low_tate()
            time.sleep(config.dt_sim)
            continue

        step_start = time.perf_counter()
        locker.acquire()
        mujoco.mj_step(mj_model, mj_data)
        locker.release()
        if not args.replay:
            bridge.publish_low_tate()
            bridge.update_motor_cmd()
            bridge.low_cmd_received = False

        time_until_next_step = mj_model.opt.timestep - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

    print("Simulation thread exited")


def ViewerThread():
    while viewer.is_running():
        locker.acquire()
        if not args.replay and config.robot_type == "hopper":
            # Add geom of estimated position and orientation
            viewer.user_scn.ngeom = 0
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[0],
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[0.2, 0.02, 0.02],
                pos=bridge.pos_est,
                mat=bridge.R_body.flatten(),
                rgba=[1, 0, 0, 0.3]
            )
            viewer.user_scn.ngeom = 1
        viewer.sync()
        locker.release()
        time.sleep(config.dt_viewer)

    print("Viewer thread exited")


if __name__ == "__main__":
    bridge.start_lcm_thread()

    viewer_thread = Thread(target=ViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()

    # Wait for threads to exit after viewer is closed
    viewer_thread.join()
    sim_thread.join()
    bridge.stop_lcm_thread()

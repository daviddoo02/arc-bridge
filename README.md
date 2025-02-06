# Robot Mujoco LCM
Software bridge of a nominal robot controller to Mujoco simulator via LCM communication.

## Dependencies
- [Mujoco](https://github.com/google-deepmind/mujoco)
- [LCM](https://github.com/lcm-proj/lcm)
- Java (for LCM visualization and type generation)

## Quick Start
1. Make sure you have java installed. Type `javac` to verify
2. Initialize LCM as submodule
    ```sh
    git submodule update --init
    ```
3. Build and install LCM from source by following the [official instructions](https://lcm-proj.github.io/lcm/content/build-instructions.html)
4. Copy the compiled [`lcm.jar`](https://lcm-proj.github.io/lcm/content/java-notes.html#finding-lcm-jar) for later use
    ```sh
    cp lcm_types/lcm/build/lcm-java/lcm.jar lcm_types/lcm.jar
    ```
5. Generate LCM types
    ```sh
    ./gen_lcm_types.sh
    ```
> [!WARNING]
> Redo this step every time if any LCM types are changed (may from new commits)
1. Install `mujoco` in your favorite python environment
    ```sh
    pip install mujoco lcm
    ```
2. Select your robot type in `config.py`
3. Run the Mujoco Viewer to simulate your robot
    ```sh
    cd ..
    python robot_mujoco.py
    ```
> [!WARNING]
> Close the viewer to exit the program. Ctrl-C might not be captured properly due to multithreading.
1. Spy the communication rate and plot data
    ```sh
    cd lcm_types
    ./run_lcm_spy.sh
    ```

## Supported Robot Models
- [x] PEA Hopper
- [x] Linefoot Biped
- [x] Pointfoot Biped
- [ ] LimX Tron 1


## Known Issues

### GLFW Error
```sh
GLFWError: (65542) b'GLX: No GLXFBConfigs returned'
GLFWError: (65545) b'GLX: Failed to find a suitable GLXFBConfig'
ERROR: could not create window
```
Solutions
```
# Set NVIDIA GPU as primary renderer (for systems with NVIDIA GPUs)
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

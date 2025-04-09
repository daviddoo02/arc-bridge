# Agile Robot Control (ARC) Bridge
Software bridge of a nominal robot controller to Mujoco simulator via LCM communication protocol.

## Dependencies
- [LCM](https://github.com/lcm-proj/lcm)
- [Mujoco](https://github.com/google-deepmind/mujoco)
- Java (LCM type generation and visualization)

## Quick Start
1. Make sure you have java installed. Type `javac` to verify.
2. Clone this repo using **SSH** and initialize LCM as a submodule.
    ```sh
    git clone git@github.com:ARCaD-Lab-UM/agile-robot-control-bridge.git
    git submodule update --init
    ```
3. Build and **install** LCM from source (under `<lcm_types/lcm>` folder) by following the [official instructions](https://lcm-proj.github.io/lcm/content/build-instructions.html)
4. Copy the compiled `lcm.jar` for later use.
    ```sh
    cp lcm_types/lcm/build/lcm-java/lcm.jar lcm_types/lcm.jar
    ```
    Check [this doc](https://lcm-proj.github.io/lcm/content/java-notes.ml#finding-lcm-jar) to find `lcm.jar` if you install LCM from other sources.
5. Generate LCM types.
    ```sh
    cd lcm_types
    ./gen_lcm_types.sh
    ```
    :warning: Redo this step every time if any LCM types are changed (may from new commits).
6. Create the conda environment with dependencies.
    ```sh
    conda env create -f environment.yml
    ```
7. Launch the entry script and follow the prompt to select robot type.
    ```sh
    conda activate arcpy
    python robot_mujoco.py
    ```
    Use `--help` to find other launching options.
   
    :warning: Use `mjpython` instead of `python` for macOS user.
8. Spy the communication rate and plot data.
    ```sh
    cd lcm_types
    ./run_lcm_spy.sh
    ```

## Supported Robot & Controller
:point_down: Click to find the corresponding controller
- [x] [PEA Hopper](https://github.com/ARCaD-Lab-UM/hopper-kd-mpc/blob/main/HopperMain.m)
- [x] [LimX Tron1 Pointfoot](https://github.com/ARCaD-Lab-UM/tron1-model-based-controller/blob/main/point_foot/MAIN_PF_LCM.m)
- [x] [LimX Tron1 Linefoot](https://github.com/ARCaD-Lab-UM/tron1-model-based-controller/blob/main/line_foot/MAIN_LF_LCM.m)
- [x] [Linefoot Biped](https://github.com/ARCaD-Lab-UM/TrainingWheel/blob/main/ex_Cassie/MAIN_cassie_LCM.m)
- [x] [Pointfoot Biped](https://github.com/ARCaD-Lab-UM/TrainingWheel/blob/main/ex_tron1/MAIN_tron1_LCM.m)
<!-- - [x] 2 Link Arm -->

## Supported OS
- Linux, macOS

## Known Issues
<details>
    <summary>  
        <b> Permission denied when initializing LCM as a submodule </b>
    </summary>

Use **SSH** option to clone this repo.
```sh
git clone --recursive git@github.com:ARCaD-Lab-UM/agile-robot-control-bridge.git
```
</details>

<details>
    <summary>  
        <b> GLFW Error </b>
    </summary>

```sh
GLFWError: (65542) b'GLX: No GLXFBConfigs returned'
GLFWError: (65545) b'GLX: Failed to find a suitable GLXFBConfig'
ERROR: could not create window
```
Set NVIDIA GPU as primary renderer (for systems with NVIDIA GPUs)
```
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```
</details>

<details>
    <summary>  
        <b> limits.h: No such file or directory </b>
    </summary>

When compiling LCM, disable unit tests.
```sh
cmake .. -DLCM_ENABLE_EXAMPLES=OFF -DLCM_ENABLE_TESTS=OFF
```
</details>

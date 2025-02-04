# Robot Mujoco LCM
Software bridge of a nominal robot controller to Mujoco simulator via LCM communication.

## Dependencies
- [LCM](https://github.com/lcm-proj/lcm)
- [Mujoco](https://github.com/google-deepmind/mujoco)
- Java (for LCM visualization and type generation)
> [!NOTE]
> Tested on Ubuntu 22.04 with Python 3.10

## Quick Start
1. Make sure you have java installed. Type `javac` to verify.
2. Initialize LCM as submodule
    ```sh
    git submodule update --init
    ```
3. Build LCM from source to get [`lcm.jar`](https://lcm-proj.github.io/lcm/content/java-notes.html#finding-lcm-jar) for later use
    ```sh
    cd lcm
    mkdir build
    cd build
    cmake ..
    make
    cp lcm-java/lcm.jar ../../lcm_types/lcm.jar
    cd ../..
    ```
4. Generate LCM types
    ```sh
    cd lcm_types
    ./gen_lcm_types.sh
    ```
5. Install `mujoco` in your favorite python environment
    ```sh
    conda activate [name]
    pip install -r requirements.txt
    ```
6. Select your robot type in `config.py`
7. Run the Mujoco Viewer to simulate your robot
    ```sh
    cd ..
    python robot_mujoco.py
    ```
> [!WARNING]
> Close the viewer to exit the program. Ctrl-C might not be captured properly due to multithreading.
8. Spy the communication rate and plot data
    ```sh
    cd lcm_types
    ./run_lcm_spy.sh
    ```

## Supported Robot Models
- [x] PEA Hopper
- [x] Linefoot Biped
- [ ] Pointfoot Biped

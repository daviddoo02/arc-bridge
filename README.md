# Robot Mujoco LCM
Software bridge of a nominal robot controller to Mujoco simulator via LCM communication

## Dependencies
- [LCM](https://github.com/lcm-proj/lcm)
- [Mujoco](https://github.com/google-deepmind/mujoco)

## Quick Start
1. Install dependencies
    ```sh
    pip install -r requirements.txt
    ```
2. Specify your robot xml file path and LCM topic names in `config.py`
3. Check if `<lcm_types>` folder contains the type templates you need
4. Generate LCM types
    ```sh
    cd lcm_types
    ./gen_lcm_types.sh
    ```
5. Run the Mujoco Viewer to simulate your robot!
    ```sh
    python robot_mujoco.py
    ```
6. Spy the communication rate and plot topics
    ```sh
    cd lcm_types
    ./run_lcm_spy.sh
    ```

## Supported Robot Models
- [x] PEA Hopper
- [ ] ??? Biped

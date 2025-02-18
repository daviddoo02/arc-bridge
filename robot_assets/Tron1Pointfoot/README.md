# Minimum Modifications to Make Official Tron1 Model Compatible


## Joint Conventions

### Joint Polarity
Joint polarities should all be reset to nominal directions regardless of actual motor placements. The actual polarities can be applied when dealing with real robot, but in Mujoco layer, resetting polarities ensure the correctness of mass matrix and Jacobian calculations.

### Joint Offset
The robot mesh comes with a non-nominal zero pose, so we manually offset each joint back to nominal zero pose when reading joint states and computing joint PD targets.

### Joint Order
Joint orders in Mujoco depends on the order of declaration of each joint. Quantities related to joint orders like `qpos`, `qvel` are all defined in the same order.

## Body Conventions

### Body Placements
To have the body coordinate aligned with Pinocchio conventions, position of the floating base link should default to `[0, 0, 0]`.

Each body link should be centered at the point of interest (e.g. joint or foot) such that correct frame Jacobians can be extracted.

### Body Inertial
Each body link should have its inertial (mass, and inertia) explicitly specified, since the default inertial is different between Mujoco and Pinocchio models.

## Sensors

### Joint Sensors
Joint sensors (`jointpos`, `jointvel` and `jointactuatorfrc`) are added to get joint position, velocity and torque with each defined in joint orders.

### Frame Sensors
Frame sensors (`framepos`, `framevel`) are added to fetch the ground truth global position and velocity of the robot.


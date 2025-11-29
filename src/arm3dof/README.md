Markdown

# 3-DOF Robot Arm Control Simulation (ITU ERO Assignment)

**Author:** Asude
**Contact:** asdbykr@gmail.com
**Description:** A ROS 2 Jazzy package simulating a 3-DOF robot arm in Gazebo Harmonic, featuring smooth joint position control and stable physics.

---

## Installation & Build (Runbook)

This package utilizes `ament_cmake`. To run this on a fresh machine, please follow these steps:

1. **Create Workspace:**
   Open a terminal and run:
   ```bash
   mkdir -p ~/arm_ws/src
   cd ~/arm_ws/src
   # Clone or copy the 'arm3dof' folder here

    Build:
    Bash

    cd ~/arm_ws
    colcon build
    source install/setup.bash

 ### How to Run

Launch the simulation world, the ROS-Gazebo bridge, and the custom controller node with a single command:
Bash

ros2 launch arm3dof arm_demo.launch.py

Note: If running on a Virtual Machine (VM) and the Gazebo screen is black, please run export LIBGL_ALWAYS_SOFTWARE=1 before launching.

 #### Design Notes

1. Modeling & Physics Strategy

    Structure: The arm is modeled using SDF. It consists of a static Base, a vertical Link 1 (Yaw axis Z), and two horizontal links (Shoulder/Elbow axes Y).

    Stability: To strictly satisfy the assignment's "no oscillations" requirement, I rigidly attached the base to the world using a fixed joint. This prevents the base from sliding due to reaction forces.

    Inertia & Mass: Standard box inertias were calculated. I significantly increased the link masses (Base: 50kg, Link1: 5kg) to help the physics engine resolve collisions more stably and reduce jitter.

2. Control Implementation

    Controller Type: As requested, I used the JointPositionController (system plugin) instead of the BLDC plugin.

    Smooth Motion (Slew Rate): To avoid the "teleporting" effect and reduce jerk, I implemented a Software Ramp (Slew Rate Limiter) within the Python node (move_demo.py). This limits the maximum velocity change per tick, creating a smooth, gliding motion between poses A -> B -> C.

    Target Poses (Radians):

        A: [0.0, -0.5, 0.5]

        B: [1.57, 0.0, -0.5]

        C: [-1.57, 0.5, -0.5]

3. Tuning (PID & Limits)

    Safety Limits: Joint limits are restricted to ±1.57 rad (±90°) to comply with the "reasonable ranges" constraint.

    Anti-Jitter Tuning: Initial tests showed vibration/oscillation at hold positions. I eliminated this by:

        Zeroing I-Gain: Integral gain was causing steady-state hunting, so I set it to 0.0.

        High Damping: I increased joint damping (values between 5.0 - 10.0) and friction. This acts as a mechanical damper, making the movement feel "heavy" and industrial.

        Adjusted P-Gain: I lowered the Proportional gain to prevent aggressive overshooting.

#####  Deliverables

    Topic Graph: Please refer to topic_graph.png included in this folder.

    Demo Video: Please refer to demo_video.mp4 included in this folder.

    Safety Limits: Joint limits are restricted to ±1.57 rad (±90°) to comply with the "reasonable ranges" constraint.

    Anti-Jitter Tuning: Initial tests showed vibration/oscillation at hold positions. I eliminated this by:

        Zeroing I-Gain: Integral gain was causing steady-state hunting, so I set it to 0.0.

        High Damping: I increased joint damping (values between 5.0 - 10.0) and friction. This acts as a mechanical damper, making the movement feel "heavy" and industrial.

        Adjusted P-Gain: I lowered the Proportional gain to prevent aggressive overshooting.

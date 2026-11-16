# Command Reference

## One-command run with Python

From the Projects directory:

```bash
/usr/bin/python3 robo2/scripts/run_simulation.py
```

This builds the package, starts a fresh Gazebo launch, waits for the controller manager, runs one pick-and-place sequence, writes `robo2/results/latest_run.json`, then shuts down Gazebo. Press `Ctrl+C` to interrupt. To reuse the current build:

```bash
/usr/bin/python3 robo2/scripts/run_simulation.py --no-build
```

Use `--help` to choose a different results path. Do not start this while another `robo2` Gazebo launch is active.

Run project commands from the directory that contains `robo2/` (the repository parent). Use a separate sourced terminal for commands marked Terminal 2.

## 1. Build the ROS 2 package

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths robo2 \
  --build-base .ros2_build \
  --install-base .ros2_install \
  --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source .ros2_install/setup.bash
```

The explicit system Python avoids accidentally selecting a different Python version from PATH.

## 2. Start Gazebo Harmonic

**Terminal 1:**

```bash
source /opt/ros/jazzy/setup.bash
source .ros2_install/setup.bash
ros2 launch robo2 gazebo.launch.py
```

This starts the `pick_place` world, robot, ROS–Gazebo bridges, and controllers. Check that `arm_controller`, `gripper_controller`, and `joint_state_broadcaster` become active.

## 3. Run pick and place

**Terminal 2:**

```bash
source /opt/ros/jazzy/setup.bash
source .ros2_install/setup.bash
ros2 run robo2 pick_place.py \
  --results robo2/results/my_run.json
```

The executor moves through pregrasp, grasp, lift, transfer, place, and retreat. It aborts if the object is disturbed during approach or if validation fails.

**Start from a fresh Gazebo scene for each full run.** The expected initial state is a zeroed arm and Box B at its source. Stop Gazebo with `Ctrl+C`, relaunch it, then run the executor again.

## 4. Inspect controllers

```bash
source /opt/ros/jazzy/setup.bash
source .ros2_install/setup.bash
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers '{}'
```

## 5. Regenerate the trajectory

This overwrites `robo2/config/pick_place_plan.json`:

```bash
/usr/bin/python3 robo2/scripts/gp7_kinematics.py
```

Review the generated plan before running it.

## 6. Tests and model checks

```bash
/usr/bin/python3 -m unittest discover -s robo2/tests -v
source /opt/ros/jazzy/setup.bash
xacro robo2/urdf/robo2_sim.urdf.xacro > /tmp/robo2.urdf
check_urdf /tmp/robo2.urdf
```

## 7. Re-derive and verify DH transforms

```bash
/usr/bin/python3 robo2/scripts/derive_dh.py
```

The output includes the current fixed tool transform and a 10,001-configuration comparison against the active URDF model. The exact-axis DH table is ill-conditioned: very large cancelling values are frame artifacts, not physical link lengths. See `DH.md`.

## 8. Small joint-motion smoke test

With Gazebo running and ROS sourced:

```bash
ros2 run robo2 joint_demo.py
```

This performs a small base-axis swivel only. It is not the pick-and-place task.

## 9. Optional headless server and separate GUI

Launch the server without the integrated GUI (replace the path with the absolute path to this checkout; quote paths containing spaces):

```bash
source /opt/ros/jazzy/setup.bash
source .ros2_install/setup.bash
ros2 launch robo2 gazebo.launch.py \
  gz_args:='-r -s /absolute/path/to/robo2/worlds/pick_place.sdf'
```

Open a second terminal for Gazebo GUI:

```bash
source /opt/ros/jazzy/setup.bash
gz sim -g
```

## 10. Stop the simulation

Press `Ctrl+C` in the terminal running `ros2 launch`. For a normal trial, let the executor finish first; then stop the launch and restart before another trial.

## 11. Git commands

From inside the repository (`cd robo2`):

```bash
git status --short --branch
git log --oneline --decorate -6
git add COMMANDS.md
git commit -m "docs: add simulation command reference"
git push
```

Generated build/install files, PDFs, videos, and most run outputs are excluded by `.gitignore`.

## Important limitations

- Box A is at ground location `(0.5,0.5,0)`, changed from the assignment's original position. Confirm instructor approval.
- The simulated grasp uses an idealized Gazebo fixed joint; it does not validate friction-based gripping.
- Manufacturer joint signs/zeros, arm inertias, and actuator dynamics are not fully calibrated. This package is not for controlling physical hardware.

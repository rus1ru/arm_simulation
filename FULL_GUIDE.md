# Full implementation guide: GP7 pick and place in ROS 2 and Gazebo

## 1. What we built

A Yaskawa GP7 CAD arm performs a pick-and-place task in Gazebo Harmonic, controlled through ROS 2 Jazzy. It approaches a 5 cm cube (Box B), closes a simplified parallel gripper, lifts the cube, transfers it while preserving its orientation, releases it at the center of Box A, and retreats.

This is an **educational simulation**, not a manufacturer-calibrated digital twin or hardware-ready controller. The arm uses CAD visual meshes, simplified collision boxes, ideal position actuation, and an explicitly idealized fixed-joint grasp. The cube is not teleported.

The latest scene uses Box A ground location `(0.5, 0.5, 0)` m. Gazebo stores its **center** at `(0.5, 0.5, 0.1)` m because its height is 0.2 m.

## 2. What we used

### Source material

| Input | Use |
|---|---|
| `Task_sheet.pdf` | Assignment requirements and permitted simplified gripper |
| `G7_manual.pdf` | GP7/GP8 mechanical manual; GP7 specifications, home diagrams and workspace |
| `GP7_GP8_datasheet.pdf` | Reach, payload, joint ranges and maximum speeds |
| `GP7-(Solidworks-2020).zip` | Original CAD archive inventory |
| `robo2/` supplied later | SolidWorks-exported URDF, STL meshes, CSV mass properties and ROS1 metadata |

We did **not** convert the native SolidWorks assembly on this Linux machine. The user supplied the exported `robo2` package. Its URDF header identifies SolidWorks to URDF Exporter. We inspected and adapted that export.

### Software verified on this machine

| Tool | Role |
|---|---|
| Ubuntu 24.04.4 LTS | Host operating system |
| ROS 2 Jazzy | Robot description, actions, nodes and controller interfaces |
| Gazebo Harmonic 8.11.0 | Physics and visualization |
| `ament_cmake`, `colcon` | ROS2 package build and installation |
| URDF / Xacro / SDF | Robot description, simulator wrapper and world |
| `robot_state_publisher` | Publishes link transforms from joint states |
| `ros_gz_sim` | Starts Gazebo and spawns the robot |
| `ros_gz_bridge` | Bridges simulator clock, box pose and grasp topics |
| `gz_ros2_control` | Exposes simulated joints to ROS2 controllers |
| Joint State Broadcaster | Publishes joint states |
| Joint Trajectory Controller | Executes arm and finger position trajectories |
| System Python 3.12 | ROS-compatible Python runtime |
| NumPy 1.26.4 / SciPy 1.11.4 | Transforms, bounded numerical IK and numerical checks |
| `pdftotext`, `pdftoppm` | Document extraction and diagram inspection |
| FFmpeg / X11 capture | Demonstration recording |

MoveIt 2 was **not used**. The planner and IK are custom Python code. No new system packages were installed during this work. SolidCraft was present but was not used for the model conversion.

Hardware: Intel i5-13420H, approximately 16 GB RAM, NVIDIA RTX 4050 laptop GPU with 6 GB VRAM. Only about 4 GB disk space was free during development, so avoid unnecessary large installs and recordings.

## 3. Why the scene changed

The assignment originally specified Box A at `(1,1,0)` with the robot at the origin. Its horizontal distance is `sqrt(2) = 1.414 m`, compared with GP7's published 0.927 m reach. This is outside the nominal reach for a conventional compact tool.

We first moved Box A closer to ground location `(0.5,0.3,0)`. The user then requested `(0.5,0.5,0)`. The current horizontal distance is about 0.707 m. Feasibility was checked with the actual numerical IK and a complete simulation run, not just this radius comparison.

**Obtain instructor approval for changing the prescribed scene.**

### Current coordinates

| Item | Center position (m) | Dimensions (m) |
|---|---|---|
| Robot base | Simulation spawn offset `(0,0,0.002)` | GP7 CAD |
| Box A | `(0.50,0.50,0.10)` | `0.30 × 0.30 × 0.20` |
| Box B initially | `(0.40,-0.25,0.025)` | `0.05 × 0.05 × 0.05` |
| Box B after placement | `(0.50,0.50,0.225)` | Same cube |

The target height is `0.20 + 0.05/2 = 0.225 m`. Box B mass is an assumed 0.05 kg, with uniform-cube inertia. Its initial location was chosen, not randomly regenerated each run.

## 4. Package structure

```text
robo2/
  package.xml                       ROS2 package metadata and dependencies
  CMakeLists.txt                    ament build/install rules
  launch/gazebo.launch.py           Starts world, bridges, robot and controllers
  worlds/pick_place.sdf             Ground, light, Box A and Box B
  meshes/*.STL                     Original CAD visual meshes
  urdf/robo2.urdf                   Preserved original export
  urdf/robo2.csv                    Exported properties
  urdf/robo2_sim_geometry.urdf       Active arm collision simplification plus corrected CAD fingers
  urdf/robo2_sim.urdf.xacro          World anchor, simulator control and grasp plugin
  config/controllers.yaml          Controller configuration
  config/pick_place_plan.json       Generated trajectory
  scripts/gp7_kinematics.py          FK, numerical IK and plan generator
  scripts/pick_place.py              Runtime execution and measured validation
  scripts/joint_demo.py              Small base swivel smoke test
  scripts/derive_dh.py               Exact-axis DH derivation and verification
  scripts/audit_mass_properties.py   Export consistency audit
  tests/test_kinematics.py           FK/IK and plan tests
  results/                          Measurements, recordings and validation output
  KINEMATICS.md                     Detailed URDF-based mathematical model
  DH.md                             DH table and conditioning warning
  REPORT_DRAFT.md                   Starting point for course report
```

`processed_documents/` contains extracted PDF text, geometry checks and calibration audits. Several evidence files describe the earlier Box A location; do not treat every historical result as validation of the current scene.

Backups include `robo2_backup_20261116_010427/`, a later `robo2_calibration_backup_*` directory, and `box_a_move_backup_*`. Original CAD meshes and original exported URDF were preserved.

## 5. Migrating the export from ROS1 to ROS2

The supplied package used Catkin, ROS1 XML launch files, `gazebo_ros` spawning and Gazebo Classic conventions. Those launch files cannot simply run under Jazzy/Harmonic.

We:

1. Backed up metadata, URDF, configuration and launch files.
2. Replaced Catkin with `ament_cmake` in `CMakeLists.txt` and `package.xml`.
3. Removed ROS1 launch files from the active package.
4. Created `launch/gazebo.launch.py`.
5. Added the robot description through Xacro and `robot_state_publisher`.
6. Started the SDF world using `ros_gz_sim`.
7. Spawned `robo2` from the `robot_description` topic.
8. Added ROS–Gazebo bridges.
9. Started controller spawners after the spawn process exits.

A fixed world-to-base joint prevents the arm from falling as a free body. The model spawns 2 mm above the world origin as an explicit numerical offset; FK includes that offset.

## 6. Preparing the CAD model

### Visual and collision geometry

The original arm STL meshes remain visual geometry. Collision geometry uses each arm mesh's local axis-aligned bounding box, which becomes an oriented box as the link rotates. This reduces the contact cost of detailed triangle meshes.

Bounding boxes fill cavities and can introduce false contacts. They are a simplification, not a precise collision model. Self collision and continuous swept-volume safety are not certified.

### Corrected CAD gripper

The first export placed the finger meshes far from their declared mass centers and used identical finger directions. The newer `robo2_fixed/robo2/urdf/robo2.urdf` corrects the two mesh visual/collision origins and reverses one prismatic axis. Its finger travel is 47 mm. The active ROS2 simulation geometry imports these corrected fingers while retaining its controller/world integration. Mesh files are unchanged from the original export.

The Link 6 to TCP transform is read from the corrected simulator URDF. To avoid the CAD wrist palm intersecting the cube during descent, the grasp TCP is 10 mm above the cube center; placement uses the same offset, so the object target is unchanged. Conservative palm clearance is approximately 3.4 mm. Both fingers close by 23 mm. The executor aborts before attachment if the cube moves more than 3 mm or rotates more than 0.02 rad during approach.

The CAD fingers are not friction-calibrated. The holding phase still uses an acknowledged Gazebo detachable fixed joint, an idealized grasp. No arbitrary finger geometry transforms are applied at runtime.

## 7. Forward kinematics

We use the actual exported joint origins and local axis directions. For each of six revolute joints:

`T_i(q_i) = T_origin_i · R(axis_i, q_i)`

URDF origin rotations use roll-pitch-yaw. Each axis is normalized before use. The complete transform is:

`T_world_TCP = T_spawn · T_1(q_1) · ... · T_6(q_6) · T_Link6_TCP`

The final tool transform is read from the corrected simulator URDF; it includes the CAD gripper frame rotation as well as translation. The grasp TCP is 10 mm above cube center. The first transform includes the 0.002 m base height.

Read `KINEMATICS.md` for the exported origin and axis table. Normalizing vector lengths is not the same as changing their directions.

### DH representation

`DH.md` and `scripts/derive_dh.py` construct standard DH transforms:

`A_i = Rz(q_i + offset_i) · Tz(d_i) · Tx(a_i) · Rx(alpha_i)`

with explicit fixed base and tool transforms. The DH and URDF FK were compared at 10,001 configurations. Maximum observed position difference was about `1.87e-8 m`, and orientation difference about `2.11e-15 rad`.

**Important:** exported axes 2 and 3 are nearly, but not exactly, parallel. Their exact common normal produces huge cancelling DH offsets of approximately `±30,811,506 m`. These are coordinate-frame artifacts, **not physical link lengths**. Do not round or present them as robot dimensions. The stable URDF transform chain is used at runtime. A practical idealized DH model would need a separate justified approximation and error budget.

## 8. Inverse kinematics

SciPy `least_squares` solves for six joint angles using a six-component residual:

```text
position residual = FK_position(q) - desired_position
rotation residual = rotation_vector(R_desired · R_FK(q)^T)
solver residual   = [position residual, 0.3 × rotation residual]
```

The factor 0.3 balances units in this numerical solver; it is not a physical robot parameter. Joint bounds are the original exported `[-3.14,3.14]` rad values, not verified manufacturer-coordinate limits.

For the initial pregrasp solution, the solver tries zero and deterministic random seeds. Later points use the previous solution as the seed to maintain continuity. Failed residual checks stop plan generation.

This is **numerical IK**, not a closed-form analytical inverse solution.

## 9. Keeping the object upright

During Cartesian phases, the TCP target rotation is fixed at:

`R_desired = Rx(pi)`

This points the tool's local +Z downward. Once the cube is attached, maintaining the tool orientation preserves the cube orientation through the fixed relative transform.

We check object orientation from simulator measurements during transport. Endpoint orientation alone would not prove the object remained upright between endpoints.

## 10. Trajectory generation

A quintic time-scaling function provides smooth progress:

`s(u) = 10u^3 - 15u^4 + 6u^5`, with `u = t/T`.

Its first and second derivatives are zero at both ends. Each phase lasts 8 s and has 161 samples. Joint velocities are estimated from sampled joint positions; endpoint velocities are set to zero and sent to the controller.

| Phase | Purpose |
|---|---|
| Pregrasp | Joint interpolation from zero to 20 cm above source |
| Grasp | Constant-orientation vertical descent to cube center |
| Lift | Vertical rise to source center + 40 cm |
| Transfer | Move to target center + 20 cm |
| Place | Vertical descent to target center |
| Retreat | Vertical rise after release |

The first pregrasp movement is joint-space interpolation. Later phases are Cartesian position interpolation with fixed orientation, converted to joint samples using IK.

The generator checks joint bounds, sampled velocity ceilings, adjacent-sample jumps, phase continuity and orientation. These checks are not continuous collision certification or motor acceleration/torque validation. Quintic Cartesian motion does not automatically guarantee jerk-limited joint motion.

## 11. Controllers and communication

```text
URDF + joint states -> robot_state_publisher -> link transforms
plan JSON -> pick_place.py -> FollowJointTrajectory actions
                             -> ROS2 trajectory controllers
                             -> gz_ros2_control position interfaces
                             -> Gazebo arm and fingers
Gazebo box pose -> bridge -> executor measurement checks
executor attach/detach -> bridge -> DetachableJoint plugin
```

Controllers:

- `joint_state_broadcaster`
- `arm_controller`
- `gripper_controller`

Action endpoints:

- `/arm_controller/follow_joint_trajectory`
- `/gripper_controller/follow_joint_trajectory`

Important bridged topics:

- `/clock`
- `/model/box_b/pose`
- `/gripper/state`
- `/gripper/attach`
- `/gripper/detach`

A dedicated PosePublisher on Box B provides named model poses. The world SceneBroadcaster pose bridge initially produced empty frame identifiers, so it was not reliable for selecting Box B.

Position interfaces are idealized. The arm effort ceiling was changed from the exporter placeholder 2 Nm to a **200 Nm simulation placeholder**, with damping 0.1. It is not a manufacturer motor torque specification or a tuned torque-control system.

## 12. Grasp and release

Gazebo's DetachableJoint creates a fixed physical constraint between `robo2/Link_6` and `box_b/link`.

Sequence:

1. The installed plugin starts attached. Executor first sends detach and waits for acknowledgement **before any motion**.
2. Open fingers.
3. Execute pregrasp and grasp phases.
4. Read fresh joint state and box pose; require TCP-to-box-center separation no greater than 25 mm.
5. Close fingers.
6. Request attachment and require acknowledgement.
7. Lift, transfer and lower with fixed orientation.
8. Detach, open fingers and retreat.
9. Let the released cube settle, then measure its final pose.

Attachment commands are repeated until acknowledgement to handle transport discovery races. Failure aborts execution.

This is an **idealized adhesive grasp**, not contact-triggered force closure, friction validation or a calibrated finger-force controller. There are no object pose commands in the executor. After release, the object is free to settle through physics.

## 13. Build and run

Open a terminal in the `Projects` directory containing `robo2/`.

### Terminal 1 — build and launch

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths robo2 \
  --build-base .ros2_build \
  --install-base .ros2_install \
  --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source .ros2_install/setup.bash
ros2 launch robo2 gazebo.launch.py
```

System Python is explicit because another Python 3.11 on PATH previously caused `catkin_pkg` import failure during the ament build. This does **not** mean the active package still uses Catkin; ament uses that package-parsing library internally.

### Terminal 2 — inspect and start motion

```bash
source /opt/ros/jazzy/setup.bash
source .ros2_install/setup.bash
ros2 service call /controller_manager/list_controllers \
  controller_manager_msgs/srv/ListControllers '{}'
ros2 run robo2 pick_place.py \
  --results robo2/results/my_run.json
```

Expect all three controllers active. Logs should progress through pregrasp, grasp, lift, transfer, place and retreat.

**Use a fresh simulation for every complete run.** The executor assumes the arm starts at zero and Box B starts on the ground. It does not automatically restore a completed scene. Stop the launch with Ctrl+C, wait for shutdown, and relaunch before repeating.

To stop motion unexpectedly, stop the launch/executor; cancellation is not a safety-rated emergency stop. Always restart from a known scene afterward.

### Headless server and separate GUI

```bash
ros2 launch robo2 gazebo.launch.py \
  gz_args:='-r -s /absolute/path/to/robo2/worlds/pick_place.sdf'
```

Then, in a sourced terminal:

```bash
gz sim -g
```

Use an absolute world path; the project directory contains spaces.

## 14. Regenerating and checking trajectories

From `Projects`:

```bash
/usr/bin/python3 robo2/scripts/gp7_kinematics.py
/usr/bin/python3 -m unittest discover -s robo2/tests -v
/usr/bin/python3 robo2/scripts/derive_dh.py
/usr/bin/python3 robo2/scripts/audit_mass_properties.py --help
```

The generator overwrites `config/pick_place_plan.json`. Back it up if preserving historical plans matters.

`processed_documents/check_arm_geometry.py` is a historical sampled collision checker. **Its Box A bounds are hardcoded for the earlier `(0.5,0.3)` scene. Do not use it unchanged to claim validation of the new `(0.5,0.5)` scene.** Its previous result covered arm links versus ground and Box A only, excluding fingers, object, self collision and swept motion.

## 15. Changing a scene position safely

Changing just the displayed box is not enough. Update consistently:

1. `worlds/pick_place.sdf`: Box A center pose.
2. `scripts/gp7_kinematics.py`: target cube center.
3. `tests/test_kinematics.py`: expected placement target.
4. Regenerate `config/pick_place_plan.json`.
5. Run tests and relevant collision checks for the new scene.
6. Restart Gazebo; changing a file does not move an existing world.
7. Execute a fresh full run and save a new results JSON.

For a ground-resting Box A, its center Z is half its height. The placed Box B center is Box A height plus half Box B height. This guide records the current values; historical reports and videos may contain earlier ones.

## 16. Results and what they prove

The latest successful run is `results/box_a_05_05_run.json`.

Observed final cube center:

```text
x = 0.5000001226 m
y = 0.4999894759 m
z = 0.2249994450 m
```

- Success flag: true.
- Final position error: approximately `1.054e-5 m` (0.01054 mm).
- Maximum sampled carry orientation deviation: approximately `5.801e-5 rad` (0.00332 degrees).

These very small errors reflect idealized simulation constraints and numerical control. **They are not measured GP7 hardware accuracy or proof of realistic gripping.**

Runtime acceptance includes:

- TCP proximity check before grasp.
- Observed lift of at least 5 cm.
- Carry orientation deviation within 0.15 rad.
- Final position within 25 mm and orientation within 0.15 rad.
- Detached state after release.

The measured errors were much smaller than these acceptance tolerances. Pose samples are discrete, not a mathematical guarantee at every instant.

`results/pick_place_clean.mp4` is a recording from an earlier geometry and is not current submission evidence. Record a new video showing the corrected CAD gripper and current `(0.5,0.5)` scene. Obstructed desktop recordings were discarded; a Gazebo-window capture was used.

## 17. Calibration audit: what is and is not verified

### Joint limits and mappings

Manual PDF page 42 (printed 5-2) and the datasheet agree:

| Axis | Manufacturer range (degrees) | Maximum speed (degrees/s) |
|---|---|---|
| S | -170 to +170 | 375 |
| L | -65 to +145 | 315 |
| U | -70 to +190 | 410 |
| R | -190 to +190 | 550 |
| B | -135 to +135 | 550 |
| T | -360 to +360 | 1000 |

These are reference specifications, **not the currently applied export limits**. Mapping requires `q_manufacturer = sign × q_export + offset`. Diagram registration suggests some differential signs, but exact wrist signs and encoder offsets are not established. We did not copy asymmetric L/U limits into unknown coordinates.

### Axis directions

Five active revolute axes had their lengths normalized. Their directions and origins were preserved. All 966 saved trajectory poses then agreed with original normalized FK to a maximum matrix-element difference of `5.55e-16`.

Apparent local axis tilts cannot be removed blindly. Shoulder axes are already parallel within about `8.24e-7 degrees`; forearm tilt agrees with CAD geometry. A possible wrist idealization could shift the TCP by nearly 1 mm across sampled configurations. No unique correction was justified, so angular snapping was not applied.

### Mass and inertia

All nine original masses are positive. Their inertia tensors are positive definite and satisfy triangle inequalities. CSV and URDF properties match. However:

- Total original exported mass is 20.3004458 kg, versus GP7 values 34 kg in datasheet and 37 kg in the manual.
- Inferred densities cluster around 1000 kg/m³, raising material-assignment concerns.
- Nonmanifold arm visual meshes limit reliable volume interpretation.
- Original finger COMs and meshes use inconsistent frames; these fingers were replaced in the active model.

Mathematical consistency is not physical validation. We did not scale masses arbitrarily. Verified material assignments, a complete BOM, per-link measurements or manufacturer inertial data are needed.

See `../processed_documents/CALIBRATION_AUDIT.md` and its linked audits for details.

## 18. Troubleshooting

| Symptom | Check / action |
|---|---|
| `ros2` or `gz` not found | Source `/opt/ros/jazzy/setup.bash` |
| Package `robo2` not found | Build and source `.ros2_install/setup.bash` |
| `No executable found` | Confirm script executable bit and rebuild |
| Python package error during build | Use `-DPython3_EXECUTABLE=/usr/bin/python3` |
| `ros2 control` unavailable | Use the list-controllers service shown above; CLI extension was not installed |
| Attachment acknowledgement timeout | Start fresh scene; check bridge/plugin topics and startup logs |
| IK generation fails | Do not execute stale plan; inspect reach, orientation, bounds and source/target |
| No fresh Box B pose | Check `/model/box_b/pose` bridge and PosePublisher |
| Scene does not reflect edit | Restart Gazebo and confirm source/install paths |
| EGL warnings | Warnings occurred on this GPU; screenshots confirmed rendering. Verify actual GUI rather than assuming failure |
| Blank/failed GUI | Try separate GUI with XCB/NVIDIA settings below; inspect logs |
| Low disk space | Preserve user files; remove only confirmed disposable generated artifacts with approval |

Optional separate-GUI environment that was tried:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
QT_QPA_PLATFORM=xcb gz sim -g
```

These settings did not eliminate every EGL warning. They are a machine-specific diagnostic option, not a universal fix.

## 19. Submission checklist

- [ ] Instructor approval for Box A relocation.
- [ ] Team member index numbers.
- [ ] Coordinate-frame illustration consistent with URDF or explicitly stated DH frames.
- [ ] FK and numerical IK explanation; do not claim a closed-form IK derivation.
- [ ] DH conditioning warning and separation of mathematical offsets from physical dimensions.
- [ ] Planning, velocity-control and executor code excerpts.
- [ ] Honest statement of idealized grasp and uncalibrated dynamics.
- [ ] New demonstration recording for the latest scene.
- [ ] Latest measured results JSON and plots if required.
- [ ] Student review of report, equations and code before submission.

The implementation demonstrates the required motion in simulation. It does not certify robot safety, factory calibration, realistic actuator dynamics or frictional grasp reliability.

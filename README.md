# GP7 pick and place — ROS 2 Jazzy / Gazebo Harmonic

## Run
From the Projects directory:
```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths robo2 --build-base .ros2_build --install-base .ros2_install --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source .ros2_install/setup.bash
ros2 launch robo2 gazebo.launch.py
```
Then, in another terminal sourced the same way:
```bash
ros2 run robo2 pick_place.py --results robo2/results/pick_place_results.json
```
Start a fresh simulation for each run. The executor expects an initially zeroed arm and Box B at its source. Do not rerun against the completed scene.

## Implemented
- ROS 1 Catkin and Gazebo Classic launch files replaced with ament_cmake and ROS 2 launch.
- Original exported arm CAD visual meshes retained. Simplified per-link bounding-box collisions.
- Corrected SolidWorks CAD finger meshes imported with mesh origins, opposite prismatic axes and 47mm travel; same mesh geometry as the new `robo2_fixed` export.
- TCP transform read from corrected CAD finger joint frame; grasp reference is 10mm above cube center, world base height .002m.
- URDF-based FK and bounded numerical IK, constant downward orientation, quintic smooth timing, sampled joint-speed checks.
- Approach/grasp/lift/transfer/place/release/retreat through ROS trajectory actions.
- Gazebo DetachableJoint attachment/release with acknowledged state changes. **Idealized adhesion, not validated friction-based gripping.** No box teleport commands.
- Measured grasp proximity, 5cm minimum lift, carry orientation, and released/settled final pose checks. Results include measured carry pose samples.

## Scene and simulation assumptions
Box A center: (.50,.50,.10)m, dimensions .30 x .30 x .20m. This requested relocation differs from the assignment and needs instructor approval.
Box B initial center: (.40,-.25,.025)m; target (.50,.50,.225)m. Ground-resting center convention. Box B mass .05kg, uniform-cube inertia.
Fixed cube orientation throughout carry. Grasp TCP is 10mm above cube center to clear the wrist palm; simulated conservative clearance ≈3.4mm. Finger closure is 23mm per finger.
Ideal position interfaces; arm effort changed from exporter placeholder 2Nm to **200Nm simulation placeholder**, damping .1. This is not a manufacturer torque specification or calibrated dynamics.
Original arm axes, joint ranges (+/-3.14), speed ceilings 2rad/s and arm inertias remain uncalibrated. Manufacturer joint limits require verified sign/zero-offset mapping before replacement. Exported arm mass differs from published GP7 mass. No hardware use.

## Validation and evidence
```bash
/usr/bin/python3 robo2/scripts/gp7_kinematics.py
/usr/bin/python3 -m unittest discover -s robo2/tests -v
/usr/bin/python3 processed_documents/check_arm_geometry.py robo2
```
The final command checks sampled arm collision boxes against ground and Box A only. It does not certify self collision, continuous swept volume, gripper contacts, or hardware safety.
See results/pick_place_results.json, results/recorded_run.json, results/pick_place_clean.mp4 and KINEMATICS.md.

## Corrected-gripper trial
Latest successful run: `results/fixed_grasp_clearance_run.json`. CAD fingers use their corrected mesh origins/opposed axis and 47 mm travel. Grasp TCP is 10 mm above cube center; 23 mm commanded finger displacement; conservative palm clearance about 3.4 mm. Approach disturbance guard: 3 mm / 0.02 rad. Latest measured position error is about 0.011 mm and max sampled carry orientation error about 1.93e-6 rad in this idealized simulation.

## Calibration audit
See `../processed_documents/CALIBRATION_AUDIT.md`. Axis vector lengths normalized only; angular alignment, manufacturer zero/sign mappings, and physical masses are not certified. No limits or masses were guessed.

## Remaining submission work
DH.md now contains a verified exact-axis standard D-H table and base/tool transforms. Tiny near-parallel export errors cause huge cancelling D-H offsets: these are frame artifacts, not physical link dimensions. Use the stable URDF FK for runtime. Numerical IK is implemented; a closed-form analytical IK derivation is not supplied. A final course report still needs team index numbers, instructor approval of the scene change, coordinate-frame illustrations and human review.
Original configuration/URDF backed up in sibling robo2_backup_20261116_010427; original CAD meshes and original exported robo2.urdf preserved. No system packages installed or shell configuration changed. Build pins system Python 3.12 because unrelated Python 3.11 is first on PATH.

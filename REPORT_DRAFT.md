# EN3563 project implementation report draft

Team index numbers: **to be filled by students**.

## Scope
Full GP7 CAD arm visualization in ROS 2 Jazzy / Gazebo Harmonic. Simplified parallel gripper is permitted by the task sheet. Robot base fixed at origin (2mm numerical spawn offset). Box A moved to (.50,.50,.10)m center per user decision; obtain instructor approval for this departure from (1,1,0). Box B 5cm cube is initially at (.40,-.25,.025)m and placed with center (.50,.50,.225)m.

## Modeling and kinematics
See KINEMATICS.md for exact coordinate frame transforms and numerical IK. CAD link geometry and exported inertias are retained for arm visualization/modeling but not independently calibrated. Six revolute joints plus two prismatic fingers. Arm collision shapes are CAD local bounding boxes; corrected CAD finger meshes are used. DH.md and derive_dh.py contain the updated exact-axis model and corrected tool transform; the table remains ill-conditioned due to near-parallel axes.

## Planning and control
Corrected CAD TCP transform is used; grasp TCP is 10mm above cube center. Constant tool rotation Rx(pi) during Cartesian segments keeps grasped object orientation unchanged. Numerical IK uses bounded nonlinear least squares and previous solution as seed. Initial approach from zero is joint interpolation; subsequent pregrasp/grasp/lift/transfer/place/retreat use quintic Cartesian interpolation with 161 samples per 8s phase. Trajectory velocities are numerical derivatives, endpoints zero. ROS joint trajectory controllers interpolate position/velocity samples. Position actuators are idealized; no physical torque-control claim.

## Grasp model
A Gazebo detachable fixed joint attaches Box B only after TCP proximity and commanded closure checks. Release detaches the physical simulator constraint and opens fingers; gravity settles the box. This models adhesion, not force-closure friction. No pose/teleport commands move the object.

## Validation
Two numerical tests pass (FK/IK roundtrip and saved plan). All 966 arm configurations satisfy original exported ranges and sampled speed ceilings. Separate transformed collision-box checks found no arm intersections with ground or Box A at samples. Excludes swept collision, self collision and safety certification.

Two complete simulator runs passed measured lift, carry orientation and post-release target checks. First run position error about 1.06e-5m, maximum carried orientation deviation about 5.8e-5rad. These very small errors reflect ideal simulation constraints, NOT GP7 hardware accuracy. See results JSON files for actual measurements. Recording files are evidence only when visually inspected; obstructed desktop captures are not submission quality.

## Limitations and review checklist
- Verify manufacturer joint zero/sign conventions before applying published limits; current ±3.14rad bounds and 2rad/s ceilings are exporter values, not GP7 specifications.
- 200Nm arm effort ceiling is a simulation placeholder, not a motor specification; arm mass/inertias need calibration.
- Palm clearance at grasp is about 3.4mm.
- DH.md provides exact-axis verified D-H transforms with a severe near-parallel conditioning warning; use URDF FK in code. Closed-form analytical IK is not supplied. Add team details, coordinate frame figure and instructor approval.
- Students must review code, explanation and demonstration before submission.

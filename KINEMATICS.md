# GP7 exported-model kinematics

## Frames and forward kinematics

The model uses the six revolute joints in `urdf/robo2_sim_geometry.urdf`, from `base_link` through `Link_6`. Coordinates are metres and radians. World axes match the spawned base axes; the base origin is at world `[0, 0, 0.002]`. These are the exported SolidWorks frames, not manufacturer-calibrated joint frames.

For column vectors, the TCP transform is

`T_world_TCP(q) = Trans(0,0,0.002) * product_i[T_origin_i * Rot(axis_i,q_i)] * T_Link6_TCP`, where the final fixed transform is read from `urdf/robo2_sim_geometry.urdf`.

Each `T_origin_i` contains the URDF joint origin translation and fixed-axis RPY rotation `Rz(yaw) Ry(pitch) Rx(roll)`. Each axis is expressed in that joint frame and normalized before using Rodrigues/rotation-vector rotation. The final transform includes the corrected CAD TCP rotation and translation; it is not a pure translation along Link_6 +Z. At zero angle every variable joint rotation is identity.

| Joint | Origin xyz | Origin rpy | Exported axis |
|---|---|---|---|
| L0_L1_Joint | 0, 0, .20326 | 0, 0, 0 | 0, 0, -1 |
| L1_L2_Joint | 0, 0, .12674 | 1.5708, 0, 1.5708 | -.0039166, 0, .99999 |
| L2_L3_Joint | .00081105, .44327, 0 | 0, 0, 1.5708 | 0, .0039166, .99999 |
| L3_L4_Joint | .040004, .00081105, 0 | 0, 0, 0 | -.045196, .99897, -.0039126 |
| L4_L5_Joint | .019932, -.44055, .0017255 | 0, 0, -1.5708 | -.0039733, .0012536, .99999 |
| L5_L6_Joint | 0, 0, 0 | 1.5708, 0, 1.5708 | -.048901, -.0039072, -.9988 |

The gripper joints are not part of this six-axis arm FK. DH.md now provides an exact-axis standard Denavit–Hartenberg construction verified against this FK over 10,001 configurations. Near-parallel exported axes cause very large cancelling DH offsets, so the stable URDF transform product remains the runtime model. These DH offsets are frame artifacts, not physical dimensions.

## Numerical inverse kinematics

`gp7_kinematics.py` uses bounded SciPy `least_squares`. Its residual concatenates TCP position error (metres) and `0.3 * log(R_desired R_current^T)` (rotation vector). The 0.3 factor is a numerical weighting, not a physical accuracy claim. The combined residual norm must be below `1e-5`. Each solution seeds the next Cartesian sample to retain the local solution branch. The first pregrasp solution uses deterministic trial seeds (random generator seed 7) if zero initialization fails.

The feasible constant tool orientation is derived from the corrected CAD tool frame so the cube remains upright. Grasp TCP is `[.4,-.25,.035]`; place TCP is `[.5,.5,.235]`. Object centers are `[.4,-.25,.025]` and `[.5,.5,.225]`. A fixed attachment preserves the grasped object's relative transform; the desired released box orientation is the identity quaternion. This assumes attachment occurs at the intended grasp pose without pose snapping.

## Trajectory

Each of six phases has 161 samples over 8 seconds. Normalized path progress is `s(u)=10u^3-15u^4+6u^5`, giving zero endpoint path velocity and acceleration. The initial zero-to-pregrasp move interpolates **joint positions**, so it is not a straight Cartesian path and does not hold downward orientation throughout. Subsequent phases interpolate straight Cartesian segments with fixed orientation and solve IK at each sample:

| Phase | TCP endpoint (world metres) |
|---|---|
| pregrasp | .4, -.25, .225 |
| grasp | .4, -.25, .025 |
| lift | .4, -.25, .425 |
| transfer | .5, .3, .425 |
| place | .5, .3, .225 |
| retreat | .5, .3, .425 |

Times in JSON are local to each phase. Joint velocities are finite-difference estimates, with endpoint values set to zero; they are not analytic IK derivatives. The executor must preserve them rather than impose zero velocity at every interior sample. The planner verifies adjacent phase endpoints, sampled joint increments below 0.1 rad, finite-difference speed bounds, supplied velocity bounds, and fixed orientation after pregrasp.

## Limits and safety scope

The active simulator URDF encodes Yaskawa GP7 manufacturer position ranges and maximum axis speeds from datasheet DS-866 (Jan. 2021), converted from degrees to radians. These are mapped to the exported axes S/L/U/R/B/T by joint order; the exported joint frame zero/sign conventions have not been calibrated against a physical controller. They are simulation constraints, not hardware-safe settings.

Corrected CAD fingers are positioned using exported mesh frames. The grasp TCP is 10 mm above the cube center to clear the palm; conservative simulated palm clearance is approximately 3.4 mm. An approach guard aborts if the cube moves more than 3 mm or rotates more than 0.02 rad before attachment. The lift/transfer height keeps the nominal box bottom above the 0.20 m platform during transfer. Full arm/environment and self-collision checks, continuous-time controller interpolation bounds, dynamics, grasp reliability, and actual object placement require simulation validation. Passing numerical kinematics tests alone does not establish those properties.

## Reproduction and observed tests

Use the installed system Python; no dependencies were installed:

```text
/usr/bin/python3 robo2/scripts/gp7_kinematics.py
Validated trajectory: .../robo2/config/pick_place_plan.json

/usr/bin/python3 -m unittest discover -s robo2/tests -v
test_fk_ik_roundtrip (test_kinematics.KinematicsTest.test_fk_ik_roundtrip) ... ok
test_saved_plan (test_kinematics.KinematicsTest.test_saved_plan) ... ok

Ran 2 tests in 0.047s
OK
```

The roundtrip test compares an FK pose with bounded IK reconstruction. The saved-plan test checks trajectory validation and source/place endpoint positions within `1e-6 m`. These are numerical model checks, not independent physical calibration.

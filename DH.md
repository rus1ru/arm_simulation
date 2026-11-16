# Standard DH representation of the exported URDF

This is a derived standard Denavit–Hartenberg representation of the **actual exported axis lines**, not manufacturer GP7 parameters. It retains the axis components, origins, angle signs, and URDF joint variables without idealizing the geometry.

## Convention

For column vectors:

`T_world_TCP(q) = B A1(q1) A2(q2) ... A6(q6) E`

`Ai(qi) = Rz(qi + theta_offset_i) Tz(di) Tx(ai) Rx(alpha_i)`.

The variables `qi` are the original URDF joint angles in their original order. There are no sign remappings. Lengths are metres and angles radians. Standard DH uses joint axis i as frame i−1 Z.

| i | theta offset | d | a | alpha |
|---|---|---|---|---|
| 1 | 1.5747092727292327 | -0.12673999999999996 | 0 | 1.5707926536179666 |
| 2 | -3.1415908313866554 | 30811506.108419511 | 0.00081185626500530309 | 1.438650862927221e-08 |
| 3 | -1.52558306769027 | -30811506.108419511 | -0.03999977744705286 | 1.5707963217668826 |
| 4 | -3.1403377556432486 | -0.44200184797594355 | 3.7220431755833813e-07 | 1.5707963431331635 |
| 5 | -3.13788375410159 | 2.7299531912827746e-08 | 1.112274236580277e-16 | 1.5707925944818577 |
| 6 | 0 | 5.5511151231257827e-17 | -1.1102230246251565e-16 | -3.5236570605778894e-19 |

### Base transform B: world to DH frame 0

```text
1 0 0 0
0 -1 0 0
0 0 -1 0.20526
0 0 0 1
```

This includes the simulated base spawn height 0.002 m. For base_link-relative FK, subtract 0.002 from B's Z translation.

### Fixed tool transforms: DH frame 6 to Link 6, then current TCP

```text
0.012229280149849777 0.99893683552523782 0.04444820961343475 -0.0061571235189010354
0.99855477273290927 -0.0098739629275023631 -0.052828692094670011 0.0014275764153203979
-0.052333646532069809 0.045030028724503936 -0.99761389623126562 -0.00017594711512385064
0 0 0 1
```

The DH derivation frame 6 is not the same as exported Link_6. The printed matrix above is $T_{DH6,Link6}$; the active model then applies $T_{Link6,TCP}$:

```text
-0.0084034957711798808 0.99876827377812749 0.048901089487693242 0.0051601582623545111
-0.99995534595841862 -0.0086047716967266319 0.0039069160762208802 0.001412173059851697
0.004322886536005939 -0.048866074103685007 0.99879598490066535 0.10536599580637081
0 0 0 1
```

The corrected TCP is the grasp reference point 10 mm above the box center. `scripts/derive_dh.py` reads its transform from the active simulator URDF and includes it separately. The composed tool transform is `E = T_DH6_Link6 * T_Link6_TCP`.

## Construction and mathematical equivalence

1. Multiply the zero-angle URDF origin transforms. For each joint, transform its normalized local axis to obtain a world-space line `p_i + t z_i`.
2. Choose DH frame 0 on the first line with Z equal to its oriented joint axis and X along world +X.
3. For each consecutive line pair, find their closest points by solving `[z_i, -z_(i+1)] [s,t]^T = p_(i+1)-p_i` in least squares. Set the next frame origin at the closest point on the next line. Set its X to the normalized cross product of the two axes; its Y is Z cross X. This makes X a common-normal direction. Signed `a` is allowed.
4. Choose final frame 6 equal to frame 5 at zero configuration. Its Z remains aligned with the sixth joint axis. The last DH link transform is consequently identity at zero angle, up to floating-point noise; the tool transform restores the exported TCP frame.
5. Decompose consecutive zero-angle frame transforms into `Rz(offset) Tz(d) Tx(a) Rx(alpha)`.

Because frame i−1 Z is the physical oriented joint i axis, inserting `Rz(qi)` represents precisely the URDF joint rotation about the same world line. The constant frame changes telescope, so the two FK products describe the same motion for every q, not only the tested samples. Numerical verification below checks implementation and floating-point error.

## Important conditioning warning

The second and third exported joint axes are **almost parallel**, not exactly parallel: their angle is approximately `1.43865e-8 rad`. Their strict common normal lies far from the arm. Consequently the exact-axis standard DH construction has `d2 ≈ +30,811,506 m` and `d3 ≈ -30,811,506 m`. These enormous cancelling offsets are a frame-construction artifact caused by tiny exported skew, **not physical arm dimensions**. Do not round them to a few decimal places or substitute a typical GP7 DH table. Such changes can destroy equivalence.

This table is mathematically valid but poorly conditioned for practical computation. The original URDF transform product is strongly preferred for planning. An idealized near-parallel DH model could have sensible dimensions, but it would be a different approximate model and would need a separate stated error budget. The derivation intentionally does not do that.

## Reproducible verification

Run from the project directory:

```text
/usr/bin/python3 robo2/scripts/derive_dh.py
```

The script emits full-precision JSON containing the table, B, DH6-to-Link6, current Link6-to-TCP, and verification errors. It compares DH FK against `Kinematics.fk` from the active corrected-gripper simulator URDF at zero and 10,000 independently generated uniform joint configurations spanning the exported bounds `[-3.14,3.14]`, with random seed 20261115.

Observed result (10,001 configurations):

- Maximum TCP position error: `1.8658271764110031e-08 m`.
- Maximum orientation geodesic error: `2.1134030099053641e-15 rad`.
- Both are below the requested `1e-6` threshold.

The script asserts both thresholds and installs no dependencies. The reported values reflect double-precision cancellation from the large offsets; they are not physical calibration errors. Small residual values in the nominally identity sixth row are retained in the machine output rather than silently rounded. No joint limits or manufacturer mappings were changed.

# Submission checklist

## Required by task sheet
- Final reviewed report PDF with all member index numbers, coordinate-frame diagram, D-H table, FK/IK calculations, IK/trajectory/velocity-control code excerpts and results.
- New screen-recorded video of CURRENT corrected CAD gripper at current Box A location. Existing pick_place_clean.mp4 is historical, not current evidence.

## Recommended code archive (or repository link in report)
Include the active robo2 package: package.xml, CMakeLists.txt, launch/, worlds/, urdf/, meshes/, config/, scripts/, tests/, README.md and FULL_GUIDE.md. Preserve mesh filename case.
Key code: scripts/gp7_kinematics.py, scripts/pick_place.py, scripts/derive_dh.py; model: urdf/robo2_sim_geometry.urdf and robo2_sim.urdf.xacro; configuration: controllers.yaml, pick_place_plan.json; launch: gazebo.launch.py; world: pick_place.sdf.
Include results/fixed_grasp_clearance_run.json and calibration limitations. Include report/report.tex and report/report.pdf.
Exclude .ros2_build/, .ros2_install/, log/, __pycache__/, duplicate backups and obsolete recordings from submission archive. Do not delete originals from your working project.

## Before submission
- Fill team names/index numbers.
- Obtain approval for Box A relocation.
- Add frame illustration and current simulation screenshots.
- DH tool transform is now regenerated from the active corrected-gripper URDF; re-run `scripts/derive_dh.py` after any frame edits and explain ill-conditioned exact-axis DH table.
- Review code excerpt line ranges and add trajectory-generation excerpt.
- Do not claim analytical IK, friction-based grasp, manufacturer calibration or hardware accuracy.
- Check course rules on AI assistance and acknowledge tools/support as required.

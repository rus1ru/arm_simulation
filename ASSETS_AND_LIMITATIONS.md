# Assets and limitations

- `meshes/*.STL` are the user-provided GP7 SolidWorks-to-URDF export meshes. They are retained as provided; inspect upstream CAD/source attribution before public redistribution.
- The original exporter metadata declares BSD in `robo2_fixed/robo2/package.xml`, but that is not evidence that every supplied CAD asset is licensed under BSD. No license has been inferred or added here.
- The CAD finger URDF corrections are user-supplied in `robo2_fixed/robo2/urdf/robo2.urdf`; `urdf/robo2_fixed_import.urdf` preserves that import.
- The assignment scene's Box A location was changed from (1,1,0) to (0.5,0.5,0) at the user's request. Obtain instructor approval.
- The Gazebo detachable-joint grasp is idealized; arm signs/zeros/dynamics are not fully calibrated.
- Do not use this model for hardware control.

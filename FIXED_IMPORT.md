# Fixed CAD finger integration

Active simulation uses finger links/joints from robo2_fixed/robo2/urdf/robo2.urdf, preserving corrected visual/collision origins, opposite axes and 0.047m travel. Arm collision simplifications and ROS2 control wrapper retained. Fixed import archived as urdf/robo2_fixed_import.urdf. Original export untouched.

WARNING: existing tool0, numerical planner TCP (+Z .13m) and .011m closing command were designed for the previous simplified fingers. They are NOT recalibrated to these CAD fingers. Do not run pick_place.py until TCP/grasp pose and closure have been updated and validated. Display/controller activation only validated for this import. Finger effort2N/speed2m/s are supplied export values, not verified settings.

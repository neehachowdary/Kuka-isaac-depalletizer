Step 1: Get the robot description files

Cloned KUKA's official robot description repository:

git clone https://github.com/kroshu/kuka_robot_descriptions.git

This repo contains kuka_iontec_support with kr50_r2500.urdf.xacro — but it's written in ROS2-style XML, using $(find package) substitutions that only work inside a full ROS2 environment.

Step 2: Decide against installing ROS2

Rather than install all of ROS2 just to convert one file, we researched and confirmed Isaac Sim + cuRobo work natively without ROS2 — ROS2 is only needed for MoveIt2 or real ROS2 robot drivers, neither of which we needed.

Step 3: Install the xacrodoc Python package

This package can resolve xacro files without needing ROS2's ament_index_python:

C:\isaacsim\python.bat -m pip install xacrodoc
Step 4: Write the conversion script

Created convert.py inside the kuka_robot_descriptions folder:

python
import xacrodoc as xd
xd.packages.update_package_cache({
    "kuka_iontec_support": "kuka_iontec_support",
    "kuka_resources": "kuka_resources",
})
doc = xd.XacroDoc.from_file("kuka_iontec_support/urdf/kr50_r2500.urdf.xacro")
doc.to_urdf_file("kr50_r2500.urdf")
Step 5: Run the conversion
C:\isaacsim\python.bat convert.py

This produced a plain kr50_r2500.urdf file — no more ROS2-specific syntax, just a standard URDF.

Step 6: Import into Isaac Sim
Opened Isaac Sim
File → Import
Selected kr50_r2500.urdf
Base Type: Fixed (critical — this tells Isaac Sim to bolt the robot to the ground and automatically creates the correct Articulation Root, avoiding the manual-articulation-root conflicts we hit early on when we tried Fixed vs. other options)
Imported successfully — robot appeared in the viewport
Step 7: Verify

Pressed Play — confirmed the robot's joints moved correctly, articulation worked, ready for the next phase (CUDA/PyTorch/cuRobo setup).
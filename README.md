# Kuka-isaac-depalletizer
depalletization pipeline: KUKA KR50 R2500 + NVIDIA Isaac Sim + cuRobo motion planning. Robot picks boxes off a stack and places them on a conveyor.

## Importing KUKA robot

* Step 1: Get the robot description files
Cloned KUKA’s official robot description repository:
git clone https://github.com/kroshu/kuka_robot_descriptions.git
This repo contains kuka_iontec_support with kr50_r2500.urdf.xacro — but it’s written in ROS2-style XML, using $(find package) substitutions that only work inside a full ROS2 environment.

* Step 2: Install the xacrodoc Python package
This package can resolve xacro files without needing ROS2’s ament_index_python:
C:\isaacsim\python.bat -m pip install xacrodoc

* Step 3: Write the conversion script
Created convert.py inside the kuka_robot_descriptions folder:
```python
import os
import xacrodoc as xd

here = os.path.dirname(os.path.abspath(__file__))
xd.packages.update_package_cache({
    "kuka_iontec_support": os.path.join(here, "kuka_iontec_support"),
    "kuka_resources": os.path.join(here, "kuka_resources"),
})
doc = xd.XacroDoc.from_file(os.path.join(here, "kuka_iontec_support/urdf/kr50_r2500.urdf.xacro"))
doc.to_urdf_file(os.path.join(here, "kr50_r2500.urdf"))
```

* Step 4: Run the conversion C:\isaacsim\python.bat convert.py
This produced a plain kr50_r2500.urdf file — no more ROS2-specific syntax, just a standard URDF.

* Step 5: Import into Isaac Sim Opened Isaac Sim File → Import Selected kr50_r2500.urdf Base Type: Fixed (critical — this tells Isaac Sim to bolt the robot to the ground and automatically creates the correct Articulation Root, avoiding the manual-articulation-root conflicts we hit early on when we tried Fixed vs. other options) Imported successfully — robot appeared in the viewport Step 7: Verify

Pressed Play — confirmed the robot’s joints moved correctly, articulation worked, ready for the next phase (CUDA/PyTorch/cuRobo setup).

## Steps for creating a scene

* Step 1: Open Isaac Sim normally (C:\isaacsim\isaac-sim.bat), with your KUKA robot already imported into the scene.

* Step 2: Open the Script Editor: Window → Script Editor

* Step 3: Paste in the script  for createscene.py, and run it (Ctrl+Enter, or the Run button in the editor).

* Step 4: After it prints "Scene rebuilt: conveyor + 4 stacked boxes" and you can see the objects in the viewport, save the scene properly: File → Save As → navigate to C:\Users\NehaaChowdary\Documents\ → filename newware_house → Save.
That's the complete process — Script Editor for building the objects programmatically, then a manual File → Save As to lock it into the .usd file that all your other standalone scripts (fix_conveyor.py, plan_palletize.py, etc.) depend on.

## Run order

```
fix_box_positions.py   # corrects box positions in the USD stage, saves
fix_conveyor.py         # corrects conveyor position in the USD stage, saves
plan_palletize.py       # cuRobo motion planning -> writes depalletize_trajectory.npy / _segments.npy
run_depalletize.py      # boots Isaac Sim, executes the planned pick-and-place
```
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.usd
from pxr import UsdGeom, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage

open_stage("C:/Users/NehaaChowdary/Documents/newware_house.usd")

world = World()
world.reset()

stage = omni.usd.get_context().get_stage()

box_half_size = 0.08
correct_positions = {
    0: (1.3, -0.8, box_half_size + 0 * (2 * box_half_size)),
    1: (1.3, -0.8, box_half_size + 1 * (2 * box_half_size)),
    2: (1.3, -0.8, box_half_size + 2 * (2 * box_half_size)),
    3: (1.3, -0.8, box_half_size + 3 * (2 * box_half_size)),
}

for i, pos in correct_positions.items():
    prim = stage.GetPrimAtPath(f"/World/box_{i}")
    xform = UsdGeom.Xformable(prim)
    ops = xform.GetOrderedXformOps()
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(*pos))
    print(f"box_{i} reset to {pos}")

# Save the corrected scene
import omni.usd
omni.usd.get_context().save_stage()
print("Scene saved with corrected box positions")

simulation_app.close()
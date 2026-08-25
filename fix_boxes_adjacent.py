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
# Side-by-side instead of stacked - same Z, varying Y with small gaps
adjacent_positions = {
    0: (1.3, -0.9, box_half_size),
    1: (1.3, -0.7, box_half_size),
    2: (1.3, -0.5, box_half_size),
    3: (1.3, -0.3, box_half_size),
}

for i, pos in adjacent_positions.items():
    prim = stage.GetPrimAtPath(f"/World/box_{i}")
    xform = UsdGeom.Xformable(prim)
    ops = xform.GetOrderedXformOps()
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(*pos))
    print(f"box_{i} repositioned to {pos}")

omni.usd.get_context().save_stage()
print("Boxes rearranged side-by-side (adjacent test)")
simulation_app.close()
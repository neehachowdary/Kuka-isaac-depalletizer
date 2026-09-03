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
prim = stage.GetPrimAtPath("/World/conveyor")
xform = UsdGeom.Xformable(prim)
ops = xform.GetOrderedXformOps()
for op in ops:
    if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
        op.Set(Gf.Vec3d(1.3, 0.7, 0.03))

omni.usd.get_context().save_stage()
print("Conveyor shifted left, scene saved")
simulation_app.close()
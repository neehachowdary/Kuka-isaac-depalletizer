from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.usd
from pxr import UsdGeom
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage

open_stage("C:/Users/NehaaChowdary/Documents/newware_house.usd")
world = World()
world.reset()

stage = omni.usd.get_context().get_stage()
robot_prim = stage.GetPrimAtPath("/kr50_r2500")

bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
bbox = bbox_cache.ComputeWorldBound(robot_prim)
rng = bbox.GetRange()
print("Robot bounding box Min:", rng.GetMin())
print("Robot bounding box Max:", rng.GetMax())

simulation_app.close()
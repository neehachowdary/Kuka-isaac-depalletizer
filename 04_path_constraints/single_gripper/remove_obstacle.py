from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage

open_stage("C:/Users/NehaaChowdary/Documents/newware_house.usd")
world = World()
world.reset()
stage = omni.usd.get_context().get_stage()

prim = stage.GetPrimAtPath("/World/static_obstacle")
if prim.IsValid():
    stage.RemovePrim("/World/static_obstacle")
    print("Obstacle removed")
else:
    print("No obstacle found to remove")

omni.usd.get_context().save_stage()
simulation_app.close()

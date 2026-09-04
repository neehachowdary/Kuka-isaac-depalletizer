from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import open3d as o3d
import omni.usd
from pxr import UsdGeom, Gf, UsdShade, Sdf
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage

open_stage("C:/Users/NehaaChowdary/Documents/newware_house.usd")
world = World()
world.reset()
stage = omni.usd.get_context().get_stage()

old = stage.GetPrimAtPath("/World/point_cloud_obstacle")
if old.IsValid():
    stage.RemovePrim("/World/point_cloud_obstacle")

pcd = o3d.io.read_point_cloud("skt.ply")
raw_points = np.asarray(pcd.points, dtype=np.float32) / 1000.0
valid_mask = ~((raw_points[:, 0] == 0.0) & (raw_points[:, 1] == 0.0) & (raw_points[:, 2] == 0.0))
valid_points = raw_points[valid_mask]

pcd_valid = o3d.geometry.PointCloud()
pcd_valid.points = o3d.utility.Vector3dVector(valid_points.astype(np.float64))
pcd_down = pcd_valid.voxel_down_sample(voxel_size=0.02)
points = np.asarray(pcd_down.points, dtype=np.float32)

points_swapped = points.copy()
points_swapped[:, 0] = points[:, 2]
points_swapped[:, 2] = points[:, 0]
points = points_swapped

centroid = points.mean(axis=0)
points = points - centroid

current_size = points.max(axis=0) - points.min(axis=0)
print(f"Original size: {current_size}")

# Scale down to a realistic wall size - target roughly 0.6m tall, thin, moderate width
scale_factor = 1.2 / current_size[2]  # scale so height becomes ~0.6m
points = points * scale_factor
new_size = points.max(axis=0) - points.min(axis=0)
print(f"Scaled size: {new_size}, scale_factor={scale_factor:.3f}")

# Position between box stack (Y=-0.8) and conveyor (Y=0.7)
target_x, target_y, target_z = 1.3, -0.3, 0.3
points[:, 0] += target_x
points[:, 1] += target_y
points[:, 2] += target_z

print(f"Final bounds - X: {points[:,0].min():.2f} to {points[:,0].max():.2f}, Y: {points[:,1].min():.2f} to {points[:,1].max():.2f}, Z: {points[:,2].min():.2f} to {points[:,2].max():.2f}")

points_prim = UsdGeom.Points.Define(stage, "/World/point_cloud_obstacle")
points_prim.CreatePointsAttr([Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in points])
points_prim.CreateWidthsAttr([0.01] * len(points))

material = UsdShade.Material.Define(stage, "/World/point_cloud_obstacle_mat")
shader = UsdShade.Shader.Define(stage, "/World/point_cloud_obstacle_mat/Shader")
shader.CreateIdAttr("UsdPreviewSurface")
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.3, 0.1))
material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI(points_prim.GetPrim()).Bind(material)

omni.usd.get_context().save_stage()
print(f"Point cloud placed (scaled down): {len(points)} points")
print("Window will stay open - close it manually when done looking")

while simulation_app.is_running():
    world.step(render=True)

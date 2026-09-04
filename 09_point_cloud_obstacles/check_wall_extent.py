import open3d as o3d
import numpy as np

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
scale_factor = 1.2 / current_size[2]
points = points * scale_factor

final_size = points.max(axis=0) - points.min(axis=0)
print(f"Final wall dimensions (X, Y, Z): {final_size}")
print(f"This means the wall spans {final_size[1]:.2f}m in the Y direction")
print(f"Wall Y-range (centered at target_y=-0.3): {-0.3 - final_size[1]/2:.2f} to {-0.3 + final_size[1]/2:.2f}")
print(f"Boxes are at Y=-0.8 - {'INSIDE' if (-0.3 - final_size[1]/2) <= -0.8 <= (-0.3 + final_size[1]/2) else 'OUTSIDE'} the wall's Y-range")

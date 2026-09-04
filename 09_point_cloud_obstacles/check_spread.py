import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("skt.ply")
raw_points = np.asarray(pcd.points, dtype=np.float32) / 1000.0

mask = (
    (raw_points[:, 0] >= -0.1) & (raw_points[:, 0] <= 0.2) &
    (raw_points[:, 1] >= -0.1) & (raw_points[:, 1] <= 0.2) &
    (raw_points[:, 2] >= 0.0) & (raw_points[:, 2] <= 0.5)
)
points = raw_points[mask]

print(f"Cropped points: {points.shape[0]}")
print(f"X: min={points[:,0].min():.6f} max={points[:,0].max():.6f} std={points[:,0].std():.6f}")
print(f"Y: min={points[:,1].min():.6f} max={points[:,1].max():.6f} std={points[:,1].std():.6f}")
print(f"Z: min={points[:,2].min():.6f} max={points[:,2].max():.6f} std={points[:,2].std():.6f}")
print(f"\nFirst 5 actual points:")
for p in points[:5]:
    print(f"  {p}")

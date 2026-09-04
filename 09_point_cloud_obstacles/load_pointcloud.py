import numpy as np
import open3d as o3d

# Update this path to match your actual .ply file location
ply_path = "skt.ply"

pcd = o3d.io.read_point_cloud(ply_path)
points = np.asarray(pcd.points, dtype=np.float32)
print(f"Loaded real point cloud: {points.shape[0]} points from {ply_path}")


# Optional: visualize it
# o3d.visualization.draw_geometries([pcd])
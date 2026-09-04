import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("skt.ply")
points = np.asarray(pcd.points, dtype=np.float32) / 1000.0

mask = (points[:, 2] >= 0.0) & (points[:, 2] <= 0.8)
slice_points = points[mask]
print(f"Points within robot's reachable height (Z=0 to 0.8): {slice_points.shape[0]}")

if slice_points.shape[0] > 0:
    print(f"\nWithin this height slice:")
    print(f"X range: {slice_points[:,0].min():.2f} to {slice_points[:,0].max():.2f}")
    print(f"Y range: {slice_points[:,1].min():.2f} to {slice_points[:,1].max():.2f}")

    print("\nX distribution within this height slice:")
    hist, edges = np.histogram(slice_points[:,0], bins=10)
    for i in range(10):
        print(f"  [{edges[i]:.2f}, {edges[i+1]:.2f}]: {hist[i]} points")

    print("\nY distribution within this height slice:")
    hist, edges = np.histogram(slice_points[:,1], bins=10)
    for i in range(10):
        print(f"  [{edges[i]:.2f}, {edges[i+1]:.2f}]: {hist[i]} points")

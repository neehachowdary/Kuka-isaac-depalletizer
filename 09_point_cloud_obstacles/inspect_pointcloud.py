import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("skt.ply")
points = np.asarray(pcd.points, dtype=np.float32) / 1000.0

print(f"Total points: {points.shape[0]}")
print(f"X range: {points[:,0].min():.2f} to {points[:,0].max():.2f}")
print(f"Y range: {points[:,1].min():.2f} to {points[:,1].max():.2f}")
print(f"Z range: {points[:,2].min():.2f} to {points[:,2].max():.2f}")

print("\nX distribution (10 buckets):")
hist, edges = np.histogram(points[:,0], bins=10)
for i in range(10):
    print(f"  [{edges[i]:.2f}, {edges[i+1]:.2f}]: {hist[i]} points")

print("\nY distribution (10 buckets):")
hist, edges = np.histogram(points[:,1], bins=10)
for i in range(10):
    print(f"  [{edges[i]:.2f}, {edges[i+1]:.2f}]: {hist[i]} points")

print("\nZ distribution (10 buckets):")
hist, edges = np.histogram(points[:,2], bins=10)
for i in range(10):
    print(f"  [{edges[i]:.2f}, {edges[i+1]:.2f}]: {hist[i]} points")

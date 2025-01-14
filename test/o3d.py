import open3d as o3d
import numpy as np 

ply_point_cloud = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud(ply_point_cloud.path)
box = o3d.geometry.TriangleMesh.create_box()
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd,box],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])

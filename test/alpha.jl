import Open3DVisualizer as V
import PyCall
global o3d = PyCall.pyimport("open3d");

V.open_window()

box = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)

V.vis.add_geometry(box)
V.sync()
V.vis.clear_geometries()


box = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
material_record = o3d.visualization.rendering.MaterialRecord()
V.vis.add_geometry(box, kwargs=Dict("material"=>material_record))




import open3d as o3d

vis = 

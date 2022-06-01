import open3d as o3d

o3d.visualization.gui.Application.instance.initialize()
vis = o3d.visualization.O3DVisualizer("test",500,500)
box = o3d.t.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
vis.add_geometry("1",box)
o3d.visualization.gui.Application.instance.run_one_tick()

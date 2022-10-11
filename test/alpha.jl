import Open3DVisualizerExp as V
import PyCall
global o3d = PyCall.pyimport("open3d");


function main()
    #open window, add test box geometry 
    V.open_window()
    box = o3d.geometry.TriangleMesh.create_box()
    material_record = o3d.visualization.rendering.MaterialRecord()
    V.vis.add_geometry(box, kwargs=Dict("material"=>material_record))
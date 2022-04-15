module Open3DVisualizer

import PyCall
import MiniGSG as S
import PoseComposition as P
import PoseComposition: Pose, IDENTITY_ORN, IDENTITY_POSE
import Images as I
import Rotations as R
import GLRenderer as GL


function __init__()
    global o3d = PyCall.pyimport("open3d");
    PyCall.py"""
    import open3d as o3d
    import numpy as np
    """
end

function pose_to_transformation_matrix(pose::Pose)::Matrix
    transform = zeros(4,4)
    transform[1:3,1:3] .= Matrix(R.RotMatrix3(pose.orientation))
    transform[1:3, 4] .= pose.pos
    transform[4,4] = 1.
    transform
end

function open_window2(intrinsics::GL.CameraIntrinsics, pose::Pose)
    PyCall.py"""
    import time
    def make_window(width, height, fx, fy, cx, cy, ext_mat):
        vis = o3d.visualization.Visualizer()
        vis.create_window(width=width, height=height)

        mesh = o3d.geometry.TriangleMesh.create_box()
        vis.add_geometry(mesh)
        for _ in range(10):
         capture_screen_float_buffer   vis.poll_events()
            vis.update_renderer()
            time.sleep(0.05)
        vis.get_render_option()
        vis.get_window_name()
        time.sleep(1.0)

        view = vis.get_view_control()

        camera = o3d.camera.PinholeCameraParameters()
        intrinsics = o3d.camera.PinholeCameraIntrinsic(
            width, height, fx, fy, cx, cy
        )
        ext_mat_np = np.array(ext_mat)
        camera.extrinsic = ext_mat_np 
        view.convert_from_pinhole_camera_parameters(camera, allow_arbitrary=True)
        print(view.convert_to_pinhole_camera_parameters().extrinsic)
    """
    PyCall.py"make_window"(
        intrinsics.width, intrinsics.height,
        intrinsics.fx, intrinsics.fy,
        intrinsics.cx, intrinsics.cy,
        pose_to_transformation_matrix(inv(pose))
    )
end

function open_window()
    global vis
    vis = o3d.visualization.Visualizer()
    vis.create_window()
end

function open_window(intrinsics::GL.CameraIntrinsics, pose::Pose)
    global vis
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=intrinsics.width, height=intrinsics.height)
    PyCall.py"""
    import numpy as np
    def make_camera_params(vis, width, height, fx, fy, cx, cy, ext_mat):
        view = vis.get_view_control()
        camera = o3d.camera.PinholeCameraParameters()
        camera.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width, height, fx, fy, cx, cy
        )
        ext_mat_np = np.array(ext_mat)
        camera.extrinsic = ext_mat_np 
        view.convert_from_pinhole_camera_parameters(camera, allow_arbitrary=True)
        print(view.convert_to_pinhole_camera_parameters().extrinsic)
    """
    ext_mat = pose_to_transformation_matrix(inv(pose))
    @show ext_mat
    PyCall.py"make_camera_params"(
        vis,
        intrinsics.width, intrinsics.height,
        intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy,
        ext_mat
    )
    update()
end

function sync()
    vis.poll_events()
    vis.update_renderer()
end

function run()
    vis.run()
    vis.destroy_window()
end

function destroy()
    vis.destroy_window()
end

function add(geometry; update=true)
    vis.add_geometry(geometry)
    if update 
        sync() 
    end
end

function update(geometry, update=true)
    vis.add_geometry(geometry)
    if update sync() end
end

function remove(geometry)
    vis.remove_geometry(geometry)
    sync()
end

function clear()
    vis.clear_geometries()
    sync()
end

function capture_image()
    img_buf = vis.capture_screen_float_buffer()
end

function make_point_cloud(cloud::Matrix; color=nothing)
    make_point_cloud(nothing, cloud; color=color)
end

function make_point_cloud(pcd, cloud::Matrix; color=nothing)
    if isnothing(color)
        color = I.colorant"red"
    end
    PyCall.py"""
    def make_point_cloud(pcd, cloud, colors):
        if pcd is None:
            pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        return pcd
    """
    colors = zeros(size(cloud)[2], 3)
    colors[:,1] .= color.r
    colors[:,2] .= color.g
    colors[:,3] .= color.b
    pcd = PyCall.py"make_point_cloud"(pcd, collect(transpose(cloud)), colors)
end

function make_bounding_box(line_set, box::S.Box, pose::Pose; color=nothing)
    if isnothing(color)
        color = I.colorant"black"
    end
    PyCall.py"""
    def make_bbox(line_set, points, lines, color):
        if line_set is None:
            line_set = o3d.geometry.LineSet()
        line_set.points =  o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        if color is not None:
            line_set.paint_uniform_color(color)
        return line_set
    """

    points = Matrix{Float64}(undef, 3, 9)
    points[:,1] = [box.sizeX/2, -box.sizeY/2, box.sizeZ/2] # blue 
    points[:,2] = [-box.sizeX/2, -box.sizeY/2, box.sizeZ/2] # orange
    points[:,3] = [-box.sizeX/2, box.sizeY/2, box.sizeZ/2] # green
    points[:,4] = [box.sizeX/2, box.sizeY/2, box.sizeZ/2] # red
    points[:,5] = [box.sizeX/2, -box.sizeY/2, -box.sizeZ/2] # purple
    points[:,6] = [-box.sizeX/2, -box.sizeY/2, -box.sizeZ/2] # brown
    points[:,7] = [-box.sizeX/2, box.sizeY/2, -box.sizeZ/2] # pink
    points[:,8] = [box.sizeX/2, box.sizeY/2, -box.sizeZ/2] # yellow

    points[:,9] = [0.0, 0.0, 0.0]
    points = permutedims(GL.move_points_to_frame_b(points, pose))
    lines = [ 
        1 2;
        2 3;
        3 4;
        4 1;
        5 6;
        6 7;
        7 8;
        8 5;
        1 5;
        2 6;
        3 7;
        4 8 
    ] .- 1
    line_set = PyCall.py"make_bbox"(line_set, points, lines, [color.r, color.g, color.b])
end

function make_mesh(m, filename::String; color=nothing)
    o3d.io.read_triangle_mesh(filename)    
end

function make_mesh(m, mesh::GL.Mesh; color=nothing)
    if isnothing(color)
        color = I.colorant"red"
    end
    PyCall.py"""
    def make_mesh(mesh, vertices, triangles, color):
        if mesh is None:
            mesh = o3d.geometry.TriangleMesh()
        mesh.vertices =  o3d.utility.Vector3dVector(vertices)
        mesh.triangles = o3d.utility.Vector3iVector(triangles)
        if color is not None:
            mesh.paint_uniform_color(color)
        return mesh
    """
    m = PyCall.py"make_mesh"(
        m, permutedims(mesh.vertices),
        permutedims(mesh.indices),
        [color.r, color.g, color.b]
    )
end

end
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
    transform[1:3,1:3] .= Matrix(R.RotMatrix{3}(pose.orientation))
    transform[1:3, 4] .= pose.pos
    transform[4,4] = 1.
    transform
end

function open_window(intrinsics::GL.CameraIntrinsics, pose::Pose)
    global vis
    width, height = intrinsics.width, intrinsics.height
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=intrinsics.width, height=intrinsics.height)

    mesh = o3d.geometry.TriangleMesh.create_box()
    vis.add_geometry(mesh)

    view = vis.get_view_control()

    PyCall.py"""
    def make_camera_params(width, height, fx, fy, cx, cy, ext_mat):
        cam = o3d.camera.PinholeCameraParameters()
        intr = o3d.camera.PinholeCameraIntrinsic(
            width, height, fx, fy, cx, cy
        )
        cam.intrinsic = intr
        cam.extrinsic = ext_mat
        return cam
    """

    camera = PyCall.py"make_camera_params"(
        intrinsics.width, intrinsics.height,
        intrinsics.fx, intrinsics.fy,
        intrinsics.cx, intrinsics.cy,
        pose_to_transformation_matrix(inv(pose))
    )

    view.convert_from_pinhole_camera_parameters(camera, allow_arbitrary=true)
    vis.remove_geometry(mesh)
end

function get_camera_extrinsics()
    global vis
    view = vis.get_view_control()
end

function set_camera_intrinsics_and_pose(intrinsics::GL.CameraIntrinsics, pose::Pose)
    global vis
    view = vis.get_view_control()

    PyCall.py"""
    def make_camera_params(width, height, fx, fy, cx, cy, ext_mat):
        cam = o3d.camera.PinholeCameraParameters()
        intr = o3d.camera.PinholeCameraIntrinsic(
            width, height, fx, fy, cx, cy
        )
        cam.intrinsic = intr
        cam.extrinsic = ext_mat
        return cam
    """

    camera = PyCall.py"make_camera_params"(
        intrinsics.width, intrinsics.height,
        intrinsics.fx, intrinsics.fy,
        intrinsics.cx, intrinsics.cy,
        pose_to_transformation_matrix(inv(pose))
    )

    view.convert_from_pinhole_camera_parameters(camera, allow_arbitrary=true)
    @show view.convert_to_pinhole_camera_parameters().extrinsic

end

function open_window()
    global vis
    vis = o3d.visualization.Visualizer()
    vis.create_window()
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

function make_axes(size::Real=1.0)
    return o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
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

function make_bounding_box(box::S.Box, pose::Pose; color=nothing)
    make_bounding_box(nothing, box, pose; color=color)
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


function make_mesh(filename::String; color=nothing)
    make_mesh(nothing, filename; colors=color)
end

function make_mesh(m, filename::String; color=nothing)
    o3d.io.read_triangle_mesh(filename, true)
end


function make_mesh(mesh::GL.Mesh; color=nothing)
    make_mesh(nothing, mesh; color=color)
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

function move_mesh_to_pose(m, pose::Pose)
    m.transform(pose_to_transformation_matrix(pose))
    m
end

function make_agent(pose::Pose; size = 0.2)
    make_agent(nothing, pose; size=size)
end

function make_agent(m, pose::Pose; size = 0.2)
    radius = size
    h = radius * 4.0
    cone = o3d.geometry.TriangleMesh.create_cone(radius=radius*2.0, height=h)
    pretransform = Pose([0.0, 0.0, h], R.RotX(pi))
    cone = cone.transform(pose_to_transformation_matrix(pretransform))
    cone = cone.transform(pose_to_transformation_matrix(pose))
    cone.paint_uniform_color([204, 255, 51] ./ 255.0)

    sphere = o3d.geometry.TriangleMesh.create_sphere(radius= radius)
    sphere.paint_uniform_color([0, 0, 0] ./ 255.0)
    sphere = sphere.transform(pose_to_transformation_matrix(pose))
    cone + sphere
end

end
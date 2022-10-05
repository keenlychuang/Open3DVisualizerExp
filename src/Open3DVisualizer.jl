module Open3DVisualizer

import PyCall
import MiniGSG as S
import PoseComposition as P
import PoseComposition: Pose, IDENTITY_ORN, IDENTITY_POSE
import Images as I
import Rotations as R
import GLRenderer as GL
import LinearAlgebra: norm, cross, dot


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


function open_window()
    global vis
    global camera_intrinsics
    global camera_pose

    camera_intrinsics = nothing
    camera_pose = nothing

    vis = o3d.visualization.Visualizer()
    vis.create_window()
end


function open_window(intrinsics::GL.CameraIntrinsics)
    open_window(intrinsics, IDENTITY_POSE)
end

function open_window(intrinsics::GL.CameraIntrinsics, pose::Pose)
    global vis
    global camera_intrinsics
    global camera_pose

    camera_intrinsics = intrinsics
    camera_pose = pose

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
    vis.poll_events()
    vis.update_renderer()
end

function set_camera_pose(pose::Pose)
    global camera_intrinsics
    global camera_pose

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
    camera_pose = pose

    view = vis.get_view_control()
    if isnothing(camera_intrinsics)
        intrinsics = view.convert_to_pinhole_camera_parameters().intrinsic
        camera = PyCall.py"make_camera_params"(
            intrinsics.width, intrinsics.height,
            intrinsics.get_focal_length()...,
            intrinsics.get_principal_point()...,
            pose_to_transformation_matrix(inv(camera_pose))
        )

    else
        intrinsics = camera_intrinsics
        camera = PyCall.py"make_camera_params"(
            intrinsics.width, intrinsics.height,
            intrinsics.fx, intrinsics.fy,
            intrinsics.cx, intrinsics.cy,
            pose_to_transformation_matrix(inv(camera_pose))
        )
    end

    view.convert_from_pinhole_camera_parameters(camera, allow_arbitrary=true)
    vis.poll_events()
    vis.update_renderer()
end

function sync()
    global vis
    if !isnothing(camera_pose)
        set_camera_pose(camera_pose)
    end
    vis.poll_events()
    vis.update_renderer()
end

function run()
    vis.run()
    vis.destroy_window()
    vis.close()
end

function destroy()
    vis.destroy_window()
    vis.close()
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

function capture_image(filename::String)
    vis.capture_screen_image(filename)
end


function make_axes(size::Real=1.0, update=true)
    a = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    add(a; update=update)
    a
end

function make_point_cloud(cloud::Matrix; color=nothing)
    make_point_cloud(nothing, cloud; color=color)
end

function make_point_cloud(pcd, cloud::Matrix; color=nothing, update=true)
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
    if typeof(color) <: I.Color
        colors = zeros(size(cloud)[2], 3)
        colors[:,1] .= color.r
        colors[:,2] .= color.g
        colors[:,3] .= color.b
    else
        colors = collect(permutedims(color))
    end
    pcd = PyCall.py"make_point_cloud"(pcd, collect(transpose(cloud[1:3,:])), colors)
    add(pcd; update=update)
    pcd
end

function make_bounding_box(box::S.Box, pose::Pose; color=nothing, update=true)
    make_bounding_box(nothing, box, pose; color=color, update=update)
end 

function make_bounding_box(line_set, box::S.Box, pose::Pose; color=nothing, update=true)
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
    add(line_set; update=update)
    line_set
end

function make_line_set(points, lines; color=nothing, update=true)
    make_line_set(nothing, points, lines; color=color, update=update)
end

function make_line_set(line_set, points, lines; color=nothing, update=true)
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
    points = permutedims(points)
    lines = permutedims(lines) .- 1
    line_set = PyCall.py"make_bbox"(line_set, points, lines, [color.r, color.g, color.b])
    add(line_set; update=update)
    line_set
end



function make_mesh(filename::String; color=nothing, update=true)
    m = make_mesh(nothing, filename; colors=color)
    add(m; update=update)
    m
end

function make_mesh(m, filename::String; color=nothing, update=true)
    m = o3d.io.read_triangle_mesh(filename, true)
    add(m; update=update)
    m
end


function make_mesh(mesh::GL.Mesh; color=nothing, update=true)
    m = make_mesh(nothing, mesh; color=color)
    add(m; update=update)
    m
end

function make_mesh(m, mesh::GL.Mesh; color=nothing, update=true)
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
    add(m; update=update)
    m
end

function move_mesh_to_pose(m, pose::Pose)
    m.transform(pose_to_transformation_matrix(pose))
    m
end

function make_agent(pose::Pose; size = 0.1, update=true)
    a = make_agent(nothing, pose; size=size)
    add(a; update=update)
    a
end

function make_agent(m, pose::Pose; size = 0.1, update=true)
    color1 = I.colorant"limegreen"
    color2 = I.colorant"red"
    radius = size
    h = radius * 4.0
    cone = o3d.geometry.TriangleMesh.create_cone(radius=radius*2.0, height=h)
    pretransform = Pose([0.0, 0.0, h], R.RotX(pi))
    cone = cone.transform(pose_to_transformation_matrix(pretransform))
    cone = cone.transform(pose_to_transformation_matrix(pose))
    cone.paint_uniform_color([color1.r, color1.g, color1.b])

    sphere = o3d.geometry.TriangleMesh.create_sphere(radius= radius)
    sphere.paint_uniform_color([color2.r, color2.g, color2.b])
    sphere = sphere.transform(pose_to_transformation_matrix(pose))
    a = cone + sphere
    add(a; update=update)
    a
end

function make_arrow(start::Vector{<:Real},dest::Vector{<:Real},radius; color=nothing, update=true)
    if isnothing(color)
        color = I.colorant"blue"
    end

    distance = sqrt(sum((start .- dest).^2))
    cylinder_height = distance * 0.95
    cone_height = distance - cylinder_height
    cylinder_radius = radius
    cone_radius = cylinder_radius * 2.0

    newZ = (dest  .- start) ./ distance

    zUnit = [0, 0, 1]
    if newZ ≈ -zUnit
      axis = [1, 0, 0]
      geodesicAngle = π
    elseif newZ ≈ zUnit
      axis = [1, 0, 0]
      geodesicAngle = 0
    else
      axis = cross(zUnit, newZ)
      geodesicAngle = let
        θ = asin(clamp(norm(axis), -1, 1))
        dot(zUnit, newZ) > 0 ? θ : π - θ
      end
    end
    
    rot = R.AngleAxis(geodesicAngle, axis...)
    pose = Pose(start, rot)

    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=cylinder_radius,
        cone_radius=cone_radius,
        cylinder_height=cylinder_height,
        cone_height=cone_height
    )
    arrow.transform(pose_to_transformation_matrix(pose))
    arrow.paint_uniform_color([color.r, color.g, color.b])

    add(arrow; update=update)
    arrow
end 

"""
Old Struct: 
mutable struct OccupancyGrid3D
    ...
    occupied_voxel_cells::Matrix{Int}               
    occupied_voxel_cells_3d_array                   #Array of values whether grid space is occupied 
end
"""

"""
New Struct: 
mutable struct OccupancyGrid3D_
    ...
    occupied_cells::Matrix{Int}
    dense::Array{VoxelOccupied,3}                   #Array of values whether grid space is occupied
end
"""
# Visualization Utility functions

#Returns a Pose from a given occupancy grid coordinate
function occupancy_grid_coord_to_pose(coord, resolution_per_dimension)
    Pose([(coord .* resolution_per_dimension)...], IDENTITY_ORN)
end

#Return a box from a given occupancy coordinate 
function occupancy_grid_coord_to_box_and_pose(coord, resolution_per_dimension)
    (box=S.Box(resolution_per_dimension...), pose=occupancy_grid_coord_to_pose(coord, resolution_per_dimension))
end

#Return all poses in a given occupancy grid 
function get_occupancy_grid_cell_poses(occupancy_grid::OccupancyGrid3D_)
    [Pose(p...) for p in eachcol(occupancy_grid.occupied_cells .* occupancy_grid.resolution_per_dimension)]
end

#
function get_occupancy_grid_cell_poses_occluded(occupancy_grid::OccupancyGrid3D_)
    [Pose(p...) for p in eachcol(occupancy_grid.all_point_cloud_cells_ravel[:,occupancy_grid.occluded[:]])]
end

#Return cell poses that are not occluded 
function get_occupancy_grid_cell_poses_free(occupancy_grid::OccupancyGrid3D_)
    [Pose(p...) for p in eachcol(occupancy_grid.occupied_cells[:,.!(occupancy_grid.occluded[:]) .& .!(occupancy_grid.occupied[:])])]
end

#Return a mesh from the given occupancy grid 
function make_mesh_from_occupancy_grid(occupancy_grid::OccupancyGrid3D_; wireframe=false)
    resolution_per_dimension = occupancy_grid.resolution_per_dimension
    if wireframe
        m = T.GL.box_wireframe_mesh_from_dims(resolution_per_dimension, 0.001)
    else
        m = T.GL.box_mesh_from_dims(resolution_per_dimension)
    end
    occ_grid_meshes = [T.shift_mesh_to_pose(m, pose) for pose in get_occupancy_grid_cell_poses(occupancy_grid)];
    occ_grid_mesh = sum(occ_grid_meshes);
    occ_grid_mesh
end

#Visualize a given occupancy grid 
function visualize_occupancy_grid(occupancy_grid::OccupancyGrid3D_)
    resolution_per_dimension = occupancy_grid.resolution_per_dimension
    b = S.Box((resolution_per_dimension .* 0.9)...)
    points_and_lines  = [
        T.get_bbox_points_and_lines(b, pose) for pose in get_occupancy_grid_cell_poses(occupancy_grid)
    ];
    points = hcat([x[1] for x in points_and_lines]...);
    lines = hcat([x[2].+(8*(i-1)) for (i,x) in enumerate(points_and_lines)]...);
    V.make_line_set(points, lines)
end

#Visualize a 2D region from a given occupancy grid and the matrix representing the region 
function visualize_2d_region(occupancy_grid::OccupancyGrid3D_, region::Matrix)  
    resolution_per_dimension = occupancy_grid.resolution_per_dimension
    b = S.Box(resolution_per_dimension...)
    points_and_lines  = [
        let
            y = nothing
            if isnothing(y)
                y = occupancy_grid.maxs[2]
            end
            coord = [x,y,z]
            pose = Pose([(coord .* resolution_per_dimension)...], IDENTITY_ORN)
            T.get_bbox_points_and_lines(b, pose)
        end
        for (x,z) in eachcol(region)
    ];
    points = hcat([x[1] for x in points_and_lines]...);
    lines = hcat([x[2].+(8*(i-1)) for (i,x) in enumerate(points_and_lines)]...);
    V.make_line_set(points, lines; color=T.I.colorant"red")
end

#Visualize a 2D region from a given occupancy grid and the vector representing the region
function visualize_2d_region(occupancy_grid::OccupancyGrid3D_, region::Vector)  
    visualize_2d_region(occupancy_grid, hcat(region...))
end

#Visualize the occupancy grid cells of a given occupancy grid and matrix of cells 
function visualize_occupancy_grid_cells(occupancy_grid::OccupancyGrid3D_, cells::Matrix; color=nothing)
    if isnothing(color)
        color = T.I.colorant"red"
    end

    resolution_per_dimension = occupancy_grid.resolution_per_dimension
    b = S.Box(resolution_per_dimension...)
    points_and_lines  = [
        let
            pose = Pose([(coord .* resolution_per_dimension)...], IDENTITY_ORN)
            T.get_bbox_points_and_lines(b, pose)
        end
        for coord in eachcol(cells)
    ];
    points = hcat([x[1] for x in points_and_lines]...);
    lines = hcat([x[2].+(8*(i-1)) for (i,x) in enumerate(points_and_lines)]...);
    V.make_line_set(points, lines; color=color)
end

end
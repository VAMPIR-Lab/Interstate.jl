using Interstate
using GLMakie
using Colors
using StaticArrays
using Polyhedra

function launch_mapping(; num_viewable=30, lanes=2, lanewidth=5.0, blocks_long=7, blocks_wide=4, buildings_per_block=3)
 
    CMD_EGO = Channel{VehicleControl}(1)
    EMG = Channel(1)
    KEY = Channel(1)
    SIM_ALL = Channel{Tuple{Float64,Dict{Int, Movable}}}(1)
    SENSE_EGO = Channel{OracleMeas}(1)
    SENSE_LIDAR = Channel{PointCloud}(1)

    road, buildings = random_grid(lanes=lanes, lanewidth=lanewidth, blocks_long=blocks_long, blocks_wide=blocks_wide, buildings_per_block=buildings_per_block)

    m1 = Bicycle(state=MVector{4,Float64}(0,-(lanes-1 + 0.5)*lanewidth,20,0), channel=CMD_EGO)

    movables = Dict([1:1+length(buildings)]....=> [m1; buildings])
    num_viewable = min(num_viewable, length(movables))

    oracle = Oracle(1, false, SENSE_EGO)
    lidar = Lidar(20, [π/10.0, π/8,], [0,0,1.0], 50.0, 1, SENSE_LIDAR) 
    sensors = Dict(1=>oracle, 2=>lidar)
    #sensors = Dict(1=>oracle)
    
    scene = Scene(resolution = (1200, 1200), show_axis=false)
    cam = cam3d!(scene, near=0.001, far=100.0, update_rate=0.01)
    visualize_road(scene, road)
    
    view_objs = []
       
    for i ∈ 1:num_viewable
        color = Observable(movables[i].color)
        corners = Observable{SVector{8, SVector{3, Float64}}}(get_corners(movables[i]))
        push!(view_objs, (corners, color)) 

        hull = @lift convexhull($corners...)
        poly = @lift polyhedron($hull)
        mesh = @lift Polyhedra.Mesh($poly)  
        GLMakie.mesh!(scene, mesh, color=color)
    end

    cam = (; x=Observable(0.0), y=Observable(0.0), θ=Observable(0.0))

    camera_pos = @lift Vec3{Float32}($(cam.x)-20*cos($(cam.θ)), $(cam.y)-20*sin($(cam.θ)), 5)
    lookat = @lift Vec3{Float32}($(cam.x), $(cam.y), 0)
    @lift update_cam!(scene, $camera_pos, $lookat)

    sim = Simulator(movables, view_objs, cam, road)

    display(scene)
    
    @sync begin
        @async controller(KEY, CMD_EGO, SENSE_EGO, EMG; disp=false, θ_step = 0.2, V_step=2.5 )
        @async localize(SENSE_LIDAR, EMG, scene, lidar, road, disp=false)
        @async simulate(sim, EMG, SIM_ALL; disp=true, check_collision=true)
        @async sense(SIM_ALL, EMG, sensors, road)
        @async keyboard_broadcaster(KEY, EMG)
    end
    nothing
end

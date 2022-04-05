using Interstate
using GLMakie
using Colors
using StaticArrays
using Polyhedra
using .Threads

function launch_mapping(; num_viewable=10, view_lidar=false, lanes=2, lanewidth=5.0, blocks_long=7, blocks_wide=4, buildings_per_block=3)
 
    CMD_EGO = ChannelLock{VehicleControl}(1)
    EMG = ChannelLock{Int}(1)
    KEY = ChannelLock{Char}(1)
    SIM_ALL = ChannelLock{Tuple{Float64,Dict{Int, Movable}}}(1)
    SENSE_EGO = ChannelLock{OracleMeas}(1)
    SENSE_LIDAR = ChannelLock{PointCloud}(1)

    road, buildings = random_grid(lanes=lanes, lanewidth=lanewidth, blocks_long=blocks_long, blocks_wide=blocks_wide, buildings_per_block=buildings_per_block)

    m1 = Bicycle(state=MVector{4,Float64}(0,-(lanes-1 + 0.5)*lanewidth,0,0), channel=CMD_EGO)

    movables = Dict([1:1+length(buildings)]....=> [m1; buildings])
    sim = Simulator(movables, road)



    oracle = Oracle(1, false, SENSE_EGO)
    lidar = Lidar(20, [π/50.0,], [0,0,1.0], 70.0, 1, SENSE_LIDAR) 
    sensors_oracle = Dict(1=>oracle)
    sensors_lidar = Dict(2=>lidar)
    
    num_viewable = min(num_viewable, length(movables))
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

    display(scene)
    
    @sync begin
        @async visualize(SIM_ALL, EMG, view_objs, cam)
        if view_lidar
            @async visualize(SENSE_LIDAR, EMG, lidar, scene)
        end
        @spawn simulate(sim, EMG, SIM_ALL; disp=true, check_collision=false)
        @spawn controller(KEY, CMD_EGO, SENSE_EGO, EMG; disp=false, θ_step = 0.2, V_step=2.5 )
        @spawn localize(SENSE_LIDAR, EMG, lidar, road)
        @spawn sense(SIM_ALL, EMG, sensors_oracle, road)
        @spawn sense(SIM_ALL, EMG, sensors_lidar, road)
        @spawn keyboard_broadcaster(KEY, EMG)
    end
    nothing
end

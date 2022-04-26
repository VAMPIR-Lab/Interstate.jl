using Interstate
using GLMakie
using Colors
using StaticArrays
using Polyhedra
using .Threads

function launch_perception(; num_agents=20, num_viewable=20, loop=true, loop_radius=50.0, lanes=4, lanewidth=5.0)
 
    CMD_FLEET = Dict{Int, Channel{VehicleControl}}()
    EMG = Channel{Int}(1)
    KEY = Channel{Char}(1)
    SIM_ALL = Channel{Tuple{Float64,Dict{Int, Movable}}}(1)
    SENSE_EGO = Channel{OracleMeas}(1)
    SENSE_FLEET = Channel{Dict{Int,OracleMeas}}(1)
    TRACKS = Channel{TracksMessage}(1)
    SENSE_CAM = Channel{Dict{Int, Vector{BBoxMeas}}}(1)

    road = simple_loop(radius=loop_radius, lanes=lanes, lanewidth=lanewidth)
   
    num_viewable = min(num_viewable, num_agents)

    movables = Dict{Int, Movable}()
    
    for i ∈ 1:num_agents
        extra_size = rand()
        width = 1.5 + 2.0 * extra_size
        length = 3.0 + 6.0 * extra_size
        height = 2.0 + 1.0 * extra_size
        speed = 5.0 + rand()*5.0
        lane = rand(1:lanes)
        color = parse(RGB, "rgb"*string(Tuple(rand(0:255,3))))
        θ = rand() * 2.0 * pi - pi
        rad = loop_radius + lanewidth / 2.0 + lanewidth * (lane-1)
        x = 0.0+cos(θ)*rad
        y = loop_radius+sin(θ)*rad
        state = MVector{4, Float64}(x, y, speed, θ+π/2.0)
        control = MVector{2, Float64}(0.0, 0.0)
        channel = Channel{VehicleControl}(1)
        movables[i] = Unicycle(state=state,
                               control=control,
                               width=width,
                               length=length,
                               #lr=length/2,
                               height=height,
                               color=color,
                               target_vel=speed,
                               target_lane=lane,
                               channel=channel)
        CMD_FLEET[i] = channel
    end
    
    
    scene = Scene(resolution = (1200, 1200), show_axis=false)
    cam1 = cam3d!(scene, near=0.001, far=100.0, update_rate=0.01)
    camera_pos_1 = SVector{3,Float64}(5.0/6*loop_radius, loop_radius, 20.0)
    camera_pos_2 = SVector{3,Float64}(3.0/4*loop_radius, loop_radius, 20.0)
    lookat = SVector{3,Float64}(0, 0, 0)
    update_cam!(scene, camera_pos_1, lookat)

    s1 = FleetOracle(Set(1:num_agents), SENSE_FLEET)
    c1 = PinholeCamera(focal_len=0.05, sx=.02, sy=.01, camera_pos=camera_pos_1, lookat=lookat)
    c2 = PinholeCamera(focal_len=0.05, sx=.02, sy=.01, camera_pos=camera_pos_2, lookat=lookat)
    camera_array = Dict(1=>c1, 2=>c2)
    s2 = CameraArray(camera_array, SENSE_CAM)
    sensors = Dict(1=>s1, 2=>s2)
    
    visualize_road(scene, road)
    outer_bbox = Interstate.BBoxMeas(-1.0,-1.0,1.0,1.0,0.0)
    camera_region = Interstate.draw_bbox_2_world(scene, camera_array[1], outer_bbox, z=10.0, linewidth=5, color=:black)
 
    #TODO pull view_obj stuff into function 
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

    sim = Simulator(movables, road)

    display(scene)
    @sync begin
        @async visualize(SIM_ALL, EMG, view_objs, nothing)
        @async visualize(SENSE_CAM, EMG, camera_array, scene)
        @spawn object_tracker(SENSE_CAM, TRACKS, EMG, camera_array, road)
        @spawn fleet_controller(CMD_FLEET, SENSE_FLEET, EMG, road)
        @spawn simulate(sim, EMG, SIM_ALL; disp=false, check_collision=false)
        @spawn sense(SIM_ALL, EMG, sensors, road)
        @spawn keyboard_broadcaster(KEY, EMG)
        @spawn eval_perception(SIM_ALL, TRACKS, EMG, camera_array, road; disp=true)
    end
    nothing
end

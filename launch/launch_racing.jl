using Interstate
using GLMakie
using Colors
using StaticArrays
using Polyhedra
using .Threads

function launch_racing(; num_agents=50, num_viewable=10, loop=true, loop_radius=100.0, lanes=3, lanewidth=5.0)

    CMD_EGO = ChannelLock{VehicleControl}(1)
    CMD_FLEET = Dict{Int, ChannelLock{VehicleControl}}()
    EMG = ChannelLock{Int}(1)
    KEY = ChannelLock{Char}(1)
    SIM_ALL = ChannelLock{Tuple{Float64,Dict{Int, Movable}}}(1)
    SENSE_EGO = ChannelLock{OracleMeas}(1)
    SENSE_FLEET = ChannelLock{Dict{Int,OracleMeas}}(1)

    if loop
        road = simple_loop(radius=loop_radius, lanes=lanes, lanewidth=lanewidth)
    else
        road = random_road(length=3000.0, lanes=lanes, lanewidth=lanewidth)
    end
   
    num_viewable = min(num_viewable, num_agents)

    m1 = Bicycle(state=MVector{4,Float64}(0,-(lanes-2 + 0.5)*lanewidth,20,0), channel=CMD_EGO)
    movables = Dict(1=>m1)
    
    for i ∈ 2:num_agents
        extra_size = rand()
        width = 1.5 + 2.0 * extra_size
        length = 3.0 + 6.0 * extra_size
        height = 2.0 + 1.0 * extra_size
        speed = 10.0 + rand()*20.0
        lane = rand(1:lanes)
        color = parse(RGB, "rgb"*string(Tuple(rand(0:255,3))))
        channel = ChannelLock{VehicleControl}(1)
        if loop
            θ = -π/2.0
            while -π/2.0-π/6.0 ≤ θ ≤ -π/2.0+π/6.0
                θ = rand() * 2.0 * pi - pi
            end
            rad = loop_radius + lanewidth / 2.0 + lanewidth * (lane-1)
            x = 0.0+cos(θ)*rad
            y = loop_radius+sin(θ)*rad
            state = MVector{4, Float64}(x, y, speed, θ+π/2.0)
            control = MVector{2, Float64}(0.0, 0.0)
        else
            y = -lanewidth*(lane-0.5)+randn()*1.0
            x = 10.0+rand()*110.0
            state = MVector{4, Float64}(x, y, speed, 0.0)
            control = MVector{2, Float64}(0, 0)
        end
        movables[i] = Bicycle(state=state,
                               control=control,
                               width=width,
                               lf=length/2,
                               lr=length/2,
                               height=height,
                               color=color,
                               target_vel=speed,
                               target_lane=lane,
                               channel=channel)
        CMD_FLEET[i] = channel
    end
    
    s1 = Oracle(1, false, SENSE_EGO) 
    s2 = FleetOracle(Set(2:num_agents), SENSE_FLEET)
    sensors = Dict(1=>s1, 2=>s2)
    
    scene = Scene(resolution = (1200, 1200), show_axis=false)
    cam = cam3d!(scene, near=0.001, far=100.0, update_rate=0.01)
    visualize_road(scene, road)
   

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

    cam = (; x=Observable(0.0), y=Observable(0.0), θ=Observable(0.0))

    camera_pos = @lift Vec3{Float32}($(cam.x)-20*cos($(cam.θ)), $(cam.y)-20*sin($(cam.θ)), 10)
    lookat = @lift Vec3{Float32}($(cam.x), $(cam.y), 0)
    @lift update_cam!(scene, $camera_pos, $lookat)

    sim = Simulator(movables, road)

    display(scene)
    
    @sync begin
        @async controller(KEY, CMD_EGO, SENSE_EGO, EMG; disp=false, V=speed(m1), θ=heading(m1), θ_step = 0.3)
        @async fleet_controller(CMD_FLEET, SENSE_FLEET, EMG, road)
        @async simulate(sim, EMG, SIM_ALL; disp=false, check_road_violation=[1,])
        @async visualize(SIM_ALL, EMG, view_objs, cam)
        @async sense(SIM_ALL, EMG, sensors, road)
        @async keyboard_broadcaster(KEY, EMG) 
    end
    nothing
end

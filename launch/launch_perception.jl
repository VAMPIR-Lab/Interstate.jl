using Interstate
using GLMakie
using Colors
using StaticArrays
using Polyhedra

function launch_perception(; num_agents=50, num_viewable=50, loop=true, loop_radius=30.0, lanes=4, lanewidth=5.0)
 
    CMD_FLEET = Channel{Dict{Int, VehicleControl}}(1)
    EMG = Channel(1)
    KEY = Channel(1)

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
        movables[i] = Unicycle(state=state,
                               control=control,
                               width=width,
                               length=length,
                               #lr=length/2,
                               height=height,
                               color=color,
                               target_vel=speed,
                               target_lane=lane,
                               channel=CMD_FLEET)
    end
    
    
    scene = Scene(resolution = (1200, 1200), show_axis=false)
    cam = cam3d!(scene, near=0.001, far=100.0, update_rate=0.01)
    camera_pos = Vec3{Float32}(loop_radius/2.0, loop_radius, 10.0)
    lookat = Vec3{Float32}(0, 0, 0)
    update_cam!(scene, camera_pos, lookat)

    visualize_road(scene, road)
    
    SENSE_FLEET = Channel{Dict{Int,OracleMeas}}(1)
    SENSE_CAM = Channel{Set{BBoxMeas}}(1)
    s1 = FleetOracle(Set(1:num_agents), SENSE_FLEET)
    s2 = Camera(camera_pos, lookat, SENSE_CAM)
    sensors = Dict(1=>s1)
 
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


    #lines!(scene, Rect2f(-0.5, -0.5, 1, 1), linewidth=5, color=:red)

    sim = Simulator(movables, sensors, view_objs, nothing, road)

    display(scene)
    
    @sync begin
        @async fleet_controller(CMD_FLEET, SENSE_FLEET, EMG, road)
        @async simulate(sim, EMG; disp=false)
        @async keyboard_broadcaster(KEY, EMG)
    end
    #GLMakie.destroy!(GLMakie.global_gl_screen())
    nothing
end

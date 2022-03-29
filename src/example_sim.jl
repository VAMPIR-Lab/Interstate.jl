using Interstate
using GLMakie

function launch_racing()

    cm1 = Channel(1)
    EMG = Channel(1)
    KEY = Channel(1)
    
    radius = 100.0
    lanewidth = 5.0
    env = Loop(lanes=3, radius=radius)
    
    num_agents = 10
    m1 = Bicycle(x=Observable(102.5), θ=Observable(pi/2.0), v=Observable(20.0), color=:green, channel=cm1)
    movables = Dict{Int, Movable}(1=>m1)
    
    for i ∈ 2:num_agents
        θ = rand() * pi/2.0
        lane = rand(0:2)
        lane = radius + 2.5 + rand(0:2) * 5.0
        extra_size = rand()
        width = 1.5 + 2.0 * extra_size
        length = 3.0 + 6.0 * extra_size
        height = 2.0 + 1.0 * extra_size
        speed = 15.0 + rand()*15.0
        color = parse(RGB, "rgb"*string(Tuple(rand(0:255,3))))
        movables[i] = Unicycle(x=Observable(cos(θ)*lane),
                               y=Observable(sin(θ)*lane), 
                               θ=Observable(θ+π/2.0), 
                               v=Observable(speed), 
                               ω=speed/lane, 
                               width=width,
                               length=length,
                               height=height,
                               color=color)
    end
    
    cs1 = Channel{OracleMeas}(1)
    s1 = Oracle(1, cs1) 
    sensors = Dict(1=>s1)
    sim = Simulator(movables, sensors, env)
    
    scene = Scene(resolution = (1200, 1200), show_axis=false)
    cam = cam3d!(scene, near=0.001, far=100.0, update_rate=0.001)
        #if !isnothing(scene)
    for (i,m) ∈ sim.movables    
        Makie.mesh!(scene, get_mesh(m), color=m.color)
    end
    view_world(scene, sim.env)
    x = sim.movables[1].x
    y = sim.movables[1].y
    θ = sim.movables[1].θ
            
    campos = @lift Vec3{Float32}($x-20*cos($θ), $y-20*sin($θ), 10)
    lookat = @lift Vec3{Float32}($x, $y, 0)
    @lift update_cam!(scene, $campos, $lookat)
    display(scene)
    
    @sync begin
        @async controller(KEY, cm1, EMG, cs1; disp=true, V=speed(m1), θ=heading(m1))
        @async simulate(sim, EMG; disp=false)
        @async keyboard_broadcaster(KEY, EMG)
    end
end

#f = Figure(resolution = (1200, 1200))
#ax = Axis3(f[1,1], 
#            aspect=:data, 
#            perspectiveness = 1.0, 
#            elevation=0.1π,
#            viewmode=:fit, 
#            zgridvisible=false,
#            zlabelvisible=false,
#            zspinevisible=false,
#            zticksvisible=false,
#            zticklabelsvisible=false,
#            xlabelvisible=false,
#            xspinevisible=false,
#            xgridvisible=true,
#            xticksvisible=false,
#            xticklabelsvisible=false,
#            ylabelvisible=false,
#            yspinevisible=false,
#            ygridvisible=true,
#            yticksvisible=false,
#            yticklabelsvisible=false,
#            xzpanelvisible=false,
#            xypanelvisible=false,
#            yzpanelvisible=false)
#
#ax[:showaxis] = (false, false, false)
#GLMakie.destroy!(GLMakie.global_gl_screen())

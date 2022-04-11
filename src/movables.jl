abstract type Movable end
abstract type Immovable end

VehicleControl = SVector{2, Float64}

Base.@kwdef struct Unicycle <: Movable
    state::MVector{4, Float64} = MVector{4, Float64}(0,0,0,0)
    control::MVector{2, Float64} = MVector{2, Float64}(0,0)
    length::Float64 = 3.0
    width::Float64 = 1.5
    height::Float64 = 1.5
    color = parse(RGB, "rgb"*string((0,0,255)))
    target_vel::Float64 = 0
    target_lane::Int = 0
    channel::Channel{VehicleControl} = Channel{VehicleControl}(0)
end

Base.@kwdef struct Bicycle <: Movable
    state::MVector{4, Float64} = MVector{4, Float64}(0,0,0,0)
    control::MVector{2, Float64} = MVector{2, Float64}(0,0)
    lf::Float64 = 1.5
    lr::Float64 = 1.5
    width::Float64 = 1.5
    height::Float64 = 1.5
    color = parse(RGB, "rgb"*string((0,0,255)))
    target_vel::Float64 = 0
    target_lane::Int = 0
    channel::Channel{VehicleControl} = Channel{VehicleControl}(0)
end

Base.@kwdef struct Building <: Movable
    position::SVector{2, Float64} = [0,0]
    heading::Float64 = 0.0
    width::Float64 = 10.0
    length::Float64 = 10.0
    height::Float64 = 20.0
    color = parse(RGB, "rgb"*string((0,0,0)))
    channel::Channel{Int} = Channel{Int}(0)
end

Base.copy(x::T) where T<:Movable = T([deepcopy(getfield(x, k)) for k ∈ fieldnames(T)]...)

function position(m::Building)
    m.position
end
function heading(m::Building)
    m.heading
end
function speed(m::Building)
    0.0
end

function update_state!(m::Building, Δ)
    return
end

function insert_random_building!(buildings, x_range, y_range)
    existing = [Box2(building) for building ∈ buildings]
    block = Box2(x_range, y_range)
    tries = 0
    while true
        x = x_range[1] + rand()*(x_range[2]-x_range[1])
        y = y_range[1] + rand()*(y_range[2]-y_range[1])
        position=[x,y]
        #heading = rand()*2*pi-pi
        heading = 0.0
        width = 5.0 + rand()*10.0
        length = 5.0 + rand()*10.0
        height = 10.0 + rand()*20.0
        color = parse(RGB, "rgb"*string(Tuple(rand(0:255,3))))
        building = Building(position=position, 
                            heading=heading, 
                            width=width, 
                            length=length, 
                            height=height, 
                            color=color)
        box = Box2(building)
        if inside(box, block) && !any(intersect(box, other).collision for other ∈ existing)
            push!(buildings, building)
            break
        else
            tries += 1
        end
        if tries > 50
            error("Can't make a random building, try different sizes")
        end
    end
end

function state(m)
    m.state
end

function control(m)
    m.control
end

function get_corners(m)
    pts = Vector{Vector{Float64}}()
    x = position(m)[1]
    y = position(m)[2]
    θ = heading(m)
    for i ∈ [rear(m), front(m)] 
        for j ∈ [right(m), left(m)]
            for k ∈ [bottom(m), top(m)]
                push!(pts, [x,y,0.0] + [cos(θ) -sin(θ) 0; sin(θ) cos(θ) 0; 0 0 1] * [i, j, k])
            end
        end
    end
    pts
end

function update_command!(m)
    m.control .= @fetch_or_return(m.channel)
end
function update_command!(m::Building)
    return
end

function update_state!(m::Unicycle, Δ)
    θ = m.state[4]
    v = m.state[3]
    a = m.control[1]
    ω = m.control[2]
    m.state[1] += Δ * cos(θ) * v
    m.state[2] += Δ * sin(θ) * v
    m.state[3] += Δ * a
    m.state[4] += Δ * ω
    return
end


function position(m)
    SVector{2, Float64}(m.state[1], m.state[2])
end

function heading(m)
    m.state[4]
end
function speed(m)
    m.state[3]
end

function rear(m::Unicycle)
    -0.5*m.length
end
function front(m::Unicycle)
    0.5*m.length
end
function right(m)
    -0.5*m.width
end
function left(m)
    0.5*m.width
end
function bottom(m)
    0.0
end
function top(m)
    m.height
end

function update_state!(m::Bicycle, Δ)
    θ = m.state[4]
    v = m.state[3]
    a = m.control[1]
    δ = m.control[2]
    lf = m.lf
    lr = m.lr 
    β = atan( lr*tan(δ) / (lf + lr) )
    
    m.state[1] += Δ * v * cos(θ + β)
    m.state[2] += Δ * v * sin(θ + β)
    m.state[3] += Δ * a
    m.state[4] += Δ * v * sin(β) / lr
    return
end

function rear(m::Bicycle)
    -m.lr
end
function front(m::Bicycle)
    m.lf
end

function rear(b::Building)
    -0.5*b.length
end

function front(b::Building)
    0.5*b.length
end

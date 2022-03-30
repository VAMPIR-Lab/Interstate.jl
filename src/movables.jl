abstract type Movable end

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
    channel = Channel(0)
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
    channel = Channel(0)
end

function state(m)
    m.state
end

function control(m)
    m.control
end

function get_corners(m)
    pts = Vector{Vector{Float64}}()
    x = m.state[1]
    y = m.state[2]
    θ = m.state[4]
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
    if length(m.channel.data) > 0
        m.control .= take!(m.channel)
    end  
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

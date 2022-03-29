abstract type Sensor end
abstract type Observation end

struct Oracle<:Sensor
    m_id
    channel
end

struct OracleMeas <: Observation 
    position
    speed
    heading
    road_segment_id
    target_lane
    target_vel
    time
end

struct FleetOracle<:Sensor
    m_ids
    channel
end

struct BBoxMeas <: Observation
    left::Float64
    top::Float64
    right::Float64
    bottom::Float64
    time
end

Base.@kwdef struct Camera<:Sensor
    focal_len::Float64
    fov::SMatrix{2,2, Float64}
    R::SMatrix{3,3,Float64}
    t::SVector{3, Float64}
    channel::Channel
end

function transform!(camera::Camera, pts...)
    for pt ∈ pts
        pt .= camera.R*pt + camera.t 
    end
end

function infov(pts, camera)
    for pt ∈ pts
        x = camera.fov[1,1] ≤ camera.focal_len * pt[1] / pt[3] ≤ camera.fov[1,2]
        y = camera.fov[2,1] ≤ camera.focal_len * pt[2] / pt[3] ≤ camera.fov[2,2]
        z = pt[3] > 0 
        if x && y && z
            return true
        end
    end
    return false
end


function expected_bbox(camera, pts, gt)
    left = Inf
    top = Inf
    right = -Inf
    bottom = -Inf
    for pt ∈ pts
        px = camera.focal_len * pt[1] / pt[3]
        py = camera.focal_len * pt[2] / pt[3]
        if px < top
            top = px
        end
        if px > bottom
            bottom = px
        end
        if py < left
            left = py
        end
        if py > right
            right = py
        end
    end
    BBoxMeas(left, top, right, bottom, gt)
end

function update_sensor(sensor::Oracle, gt, ms, road)
    m = ms[sensor.m_id]
    meas = OracleMeas(position(m), speed(m), heading(m), road_segment(m,road), m.target_lane, m.target_vel, gt)
    while length(sensor.channel.data) > 0
        take!(sensor.channel)
    end
    put!(sensor.channel, meas)
end

function update_sensor(sensor::FleetOracle, gt, ms, road)
    meas = Dict{Int, OracleMeas}()
    for id ∈ sensor.m_ids
        m = ms[id]
        omeas = OracleMeas(position(m), speed(m), heading(m), road_segment(m,road), m.target_lane, m.target_vel, gt)
        meas[id] = omeas
    end
    while length(sensor.channel.data) > 0
        take!(sensor.channel)
    end
    put!(sensor.channel, meas)
end

function update_sensor(sensor::Camera, gt, ms, road)
    meas = Vector{BBoxMeas}
    for (id, m) ∈ ms
        pts = get_corners(m)
        transform!(sensor, pts...)
        if infov(pts, sensor)
            bbox = expected_bbox(pts, gt)
            push!(meas, bbox)
        end
    end
    while length(sensr.channel.data) > 0
        take!(sensor.channel)
    end
    put!(sensor.channel, meas)
end


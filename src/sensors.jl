abstract type Sensor end
abstract type Observation end

struct GPS<:Sensor
    m_id
    channel
end

struct GPSMeas <: Observation 
    xyz
    time
end

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
end

struct Camera<:Sensor
    focal_len::Float64
    R::SMatrix{3,3,Float64}
    t::SVector{3, Float64}
    channel::Channel
end

function update_sensor(sensor::GPS, gt, ms)
    m = ms[sensor.m_id]
    meas = GPSMeas([position(m);0.0], gt)
    if length(sensor.channel.data) > 0
        take!(sensor.channel)
    end
    put!(sensor.channel, meas)
end

function update_sensor(sensor::Oracle, gt, ms, road)
    m = ms[sensor.m_id]
    meas = OracleMeas(position(m), speed(m), heading(m), road_segment(m,road), m.target_lane, m.target_vel, gt)
    if length(sensor.channel.data) > 0
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
    if length(sensor.channel.data) > 0
        take!(sensor.channel)
    end
    put!(sensor.channel, meas)
end


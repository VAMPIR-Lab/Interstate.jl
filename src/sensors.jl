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

struct PinholeCamera<:Sensor
    focal_len::Float64
    sx::Float64
    sy::Float64
    R::SMatrix{3,3,Float64}
    t::SVector{3, Float64}
    channel::Channel
end

struct CameraArray<:Sensor
    cameras::Dict{Int, PinholeCamera}
    channel::Channel
end

function get_transform(pos, lookat)
    Δ = lookat-pos  
    z⃗ = Δ / norm(Δ)
    x⃗ = cross(z⃗, [0.0,0.0,1.0])
    x⃗ /= norm(x⃗)
    y⃗ = cross(z⃗, x⃗)
    y⃗ /= norm(y⃗)
    R = [x⃗ y⃗ z⃗]
    t = -R'*pos
    R', t
end

function PinholeCamera(; focal_len::Float64=0.05, sx::Float64=10, sy::Float64=10, camera_pos::SVector{3, Float64}, lookat::SVector{3, Float64}, channel::Channel)
    R, t = get_transform(camera_pos, lookat)
    PinholeCamera(focal_len, sx, sy, R, t, channel)
end

function transform!(camera::PinholeCamera, pts...)
    for pt ∈ pts
        pt .= camera.R*pt + camera.t 
    end
end

function infov(pts, camera)
    for pt ∈ pts
        x = -1.0 ≤ camera.focal_len * pt[1] / (pt[3]*camera.sx) ≤ 1.0
        y = -1.0 ≤ camera.focal_len * pt[2] / (pt[3]*camera.sy) ≤ 1.0
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
        px = max(min(camera.focal_len * pt[1] / (pt[3] * camera.sx), 1.0), -1.0)
        py = max(min(camera.focal_len * pt[2] / (pt[3] * camera.sy), 1.0), -1.0)
        if py < top
            top = py
        end
        if py > bottom
            bottom = py
        end
        if px < left
            left = px
        end
        if px > right
            right = px
        end
    end
    BBoxMeas(left, top, right, bottom, gt)
end

function draw_bbox_2_world(scene, camera, bbox; z=1.0, color=:red, linewidth=1)
    left = bbox.left
    top = bbox.top
    right = bbox.right
    bot = bbox.bottom

    z = 1.0
    x_left = left*z*camera.sx / camera.focal_len
    x_right = right*z*camera.sx / camera.focal_len
    y_top = top*z*camera.sy / camera.focal_len
    y_bot = bot*z*camera.sy / camera.focal_len

    pts = [x_left x_left x_right x_right x_left;
           y_top y_bot y_bot y_top y_top;
           z    z     z      z     z]
    for i in 1:5
        pts[:,i] .= camera.R'*(pts[:,i]-camera.t)
    end
    line = lines!(scene, pts[1,:], pts[2,:], pts[3,:], color=color, linewidth=linewidth)
    return line
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

function get_camera_meas(sensor, gt, ms, road)
    meas = Vector{BBoxMeas}()
    for (id, m) ∈ ms
        pts = get_corners(m)
        transform!(sensor, pts...)
        if infov(pts, sensor)
            bbox = expected_bbox(sensor, pts, gt)
            push!(meas, bbox)
        end
    end
    meas
end

function update_sensor(sensor::PinholeCamera, gt, ms, road)
    meas = get_camera_meas(sensor, gt, ms, road)
    while length(sensor.channel.data) > 0
        take!(sensor.channel)
    end
    put!(sensor.channel, meas)
end

function update_sensor(sensor::CameraArray, gt, ms, road)
    meas = Dict{Int, Vector{BBoxMeas}}()
    for (id, camera) ∈ sensor.cameras
        m = get_camera_meas(camera, gt, ms, road)
        meas[id] = m
    end
    while length(sensor.channel.data) > 0
        take!(sensor.channel)
    end
    put!(sensor.channel, meas)
end


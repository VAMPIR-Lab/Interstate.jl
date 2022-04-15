
function sense(simulator_state::Channel, emg::Channel, sensors, road)  
    println("Sensing on thread ", Threads.threadid())
    while true
        sleep(0)
        @return_if_told(emg)
        (timestamp, movables) = @fetch_or_continue(simulator_state)
        for (id, sensor) ∈ sensors
            meas = update_sensor(sensor, timestamp, movables, road)
            @replace(sensor.channel, meas)
        end
    end
end


abstract type Sensor end
abstract type Observation end

struct GPS<:Sensor
    m_id::Int
    noise_std::Float64
    channel::Channel
end

struct GPSMeas <: Observation
    position
    time
end

struct Oracle<:Sensor
    m_id::Int
    find_road_segment::Bool
    channel::Channel
end

struct OracleMeas <: Observation 
    position
    speed
    heading
    road_segment_id
    target_lane
    target_vel
    front #offset from position to front of vehicle (positive value)
    rear  #offset from position to rear of vehicle (negative value)
    left  #offset from position to left of vehicle (positive value)
    right #offset from position to right of vehicle (negative value)
    time
end

struct FleetOracle<:Sensor
    m_ids
    channel::Channel
end

struct BBoxMeas <: Observation
    left::Float64
    top::Float64
    right::Float64
    bottom::Float64
    time::Float64
end

struct PointCloud <: Observation
    points::Vector{SVector{3, Float64}}
    origin::SVector{3, Float64}
    time::Float64
end
    

struct PinholeCamera<:Sensor
    focal_len::Float64
    sx::Float64
    sy::Float64
    R::SMatrix{3,3,Float64}
    t::SVector{3, Float64}
end

struct CameraArray<:Sensor
    cameras::Dict{Int, PinholeCamera}
    channel::Channel
end

struct Lidar<:Sensor
    angular_resolution::Int
    beam_elevations::Vector{Float64}
    offset::SVector{3, Float64} # relative to top center of movable[m_id]
    max_beam_length::Float64
    m_id::Int
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

function PinholeCamera(; focal_len::Float64=0.05, sx::Float64=10, sy::Float64=10, camera_pos::SVector{3, Float64}, lookat::SVector{3, Float64})
    R, t = get_transform(camera_pos, lookat)
    PinholeCamera(focal_len, sx, sy, R, t)
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

function draw_lidar_beams_2_world(scene, lidar, point_cloud; color=:red, linewidth=1)
    o = point_cloud.origin
    ll = [lines!(scene, [o[1],pt[1]], [o[2],pt[2]], [o[3],pt[3]], color=color, linewidth=linewidth) for pt ∈ point_cloud.points]
end

function update_sensor(sensor::Oracle, gt, ms, road)
    m = ms[sensor.m_id]
    seg = sensor.find_road_segment ? road_segment(m, road) : -1
    meas = OracleMeas(position(m), speed(m), heading(m), seg, m.target_lane, m.target_vel, front(m), rear(m), left(m), right(m), gt)
end

function update_sensor(sensor::GPS, gt, ms, road)
    m = ms[sensor.m_id]
    noisy_pos = position(m) + sensor.noise_std * randn(2)
    meas = GPSMeas(position(m), gt)
end

function update_sensor(sensor::FleetOracle, gt, ms, road)
    meas = Dict{Int, OracleMeas}()
    for id ∈ sensor.m_ids
        m = ms[id]
        omeas = OracleMeas(position(m), speed(m), heading(m), road_segment(m,road), m.target_lane, m.target_vel, front(m), rear(m), left(m), right(m), gt)
        meas[id] = omeas
    end
    meas
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
end

function update_sensor(sensor::CameraArray, gt, ms, road)
    meas = Dict{Int, Vector{BBoxMeas}}()
    for (id, camera) ∈ sensor.cameras
        m = get_camera_meas(camera, gt, ms, road)
        meas[id] = m
    end
    meas
end

function expected_lidar_return(pos, α_min, ϕ, θ, ms, road)
    x = cos(θ)
    y = sin(θ)
    z = -atan(ϕ, 1.0)
    beam = [x, y, z]
    beam /= norm(beam)
    ray = Ray3(pos, beam)
    α_ground = abs(pos[3]/beam[3])
    α_min = min(α_min, α_ground)
    pt_min = pos + α_min * beam
    for (id, m) ∈ ms
        box = Box3(m)
        #(; collision, p, α) = intersect(box, ray, max_dist=α_ground)
        (; collision, p, α) = intersect(box, ray, max_dist=Inf)
        if collision && α < α_min
            α_min = α
            pt_min = p
        end
    end
    return pt_min
end

function update_sensor(sensor::Lidar, gt, ms, road)
    m_ego = ms[sensor.m_id]
    lidar_pos = [position(m_ego); top(m_ego)] + sensor.offset
    θ₀ = heading(m_ego)
    pts = Vector{SVector{3, Float64}}()
    N = sensor.angular_resolution
    angles = LinRange(-π, π-2*π/N, N)
    for ϕ ∈ sensor.beam_elevations
        for θ ∈ angles
            pt = expected_lidar_return(lidar_pos, sensor.max_beam_length, ϕ, θ+θ₀, ms, road)
            push!(pts, pt)
        end
    end
    meas = PointCloud(pts, lidar_pos, gt)
end

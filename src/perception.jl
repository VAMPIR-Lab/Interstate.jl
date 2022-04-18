#= struct ObjectState
    x::Float64
    y::Float64
    θ::Float64
    length::Float64
    width::Float64
    height::Float64
end

struct TracksMessage
    timestamp::Float64
    tracks::Dict{Int, ObjectState}
end =#

struct ObjectState
    x::Float64
    y::Float64
    v::Float64
    θ::Float64
    length::Float64
    width::Float64
    height::Float64
end

struct TracksMessage
    timestamp::Float64
    tracks::Dict{Int, Tuple{ObjectState, Matrix{Float64}}} # Hold ObjectState, covariance
end


# This function returns random Gaussian noise
function random_sample(Σ)
    sqrt(Σ) * randn(size(Σ,1))
end

# This function returns the time difference
function get_time(prev_t, cur_t)
    Δt = cur_t - prev_t
end

# TODO - find velocity ???
function get_velocity()
    v = 5.0 + rand()*5.0
end

# Converts type BBoxMeas to a vector
function convert_to_vector(bbox::BBoxMeas)
    z = [bbox.left, bbox.top, bbox.right, bbox.bottom]
end

# Converts type ObjectState to a moveable
function convert_to_ms(x_predicted::ObjectState)
    state = MVector{4, Float64}(x_predicted.x, x_predicted.y, x_predicted.v, x_predicted.θ) 
    m = Unicycle(state=state,
        length=x_predicted.length,
        width=x_predicted.width,
        height=x_predicted.height) # Other vals are defaulted
end

# TODO initialize x,y
# This function initializes random initial conditions (as done in launch_perception)
function initialize(bbox::BBoxMeas)
    x0 = round((bbox.left + bbox.right)/2)
    y0 = round((bbox.top + bbox.bottom)/2)

    v0 = 5.0 + rand()*5.0
    θ0 = rand() * 2.0 * pi - pi
    extra_size = rand()
    l0 = 3.0 + 6.0 * extra_size
    w0 = 1.5 + 2.0 * extra_size
    h0 = 2.0 + 1.0 * extra_size
    
    x = ObjectState(x0, y0, v0, θ0, l0, w0, h0)
    P = diagm(0.1*ones(Float64, 7))
    (x, P)
end

# This function updates the vehicle state based off of a previous state estimate saved in tracks
function state_dynamics(state::ObjectState, v::Float64, Δt::Float64, P)
    next_state = copy(state)
    w = random_sample(P) # Gaussian noise
    next_state += Δt * [v*cos(state.θ), v*sin(state.θ), 0, 0, 0, 0, 0] + w
end

# Calculates jacobian of f(x, u) = F
function jac_f(Δt::Float64, v::Float64, θ::Float64)
    F = [1 0 Δt*cos(θ) -Δt*v*sin(θ) 0 0 0;
        0 1 Δt*sin(θ) Δt*v*cos(θ) 0 0 0;
        0 0 1 0 0 0 0;
        0 0 0 1 0 0 0;
        0 0 0 0 1 0 0;
        0 0 0 0 0 1 0;
        0 0 0 0 0 0 1]
end

# This function gets the expected bbox given the predicted position and cameras in frame
function get_predicted_bbox(x_predicted::ObjectState, camera_array, cur_time, road, out_of_frame)
    moveables = Dict{Int,Movable}()
    moveables[1] = convert_to_ms(x_predicted)

    c1 = camera_array[1] # ::PinholeCamera
    c2 = camera_array[2] # ::PinholeCamera

    z_left = []
    z_right = []
    if out_of_frame !== 1 # Left camera is in frame
        bbox_left = update_sensor(c1, cur_time, moveables, road)
        z_left = convert_to_vector(bbox_left)
    end
    if out_of_frame !== 2 # Right camera is in frame
        bbox_right = update_sensor(c2, cur_time, moveables, road)
        z_right = convert_to_vector(bbox_right)
    end
    [z_left; z_right]
end

# TODO
# Calculates jacobian of expected bounding box = H
function jac_h(Δt::Float64, v::Float64, θ::Float64)
    H = [1 0 Δt*cos(θ) -Δt*v*sin(θ) 0 0 0;
        0 1 Δt*sin(θ) Δt*v*cos(θ) 0 0 0;
        0 0 1 0 0 0 0;
        0 0 0 1 0 0 0;
        0 0 0 0 1 0 0;
        0 0 0 0 0 1 0;
        0 0 0 0 0 0 1]
end

# TODO - alter Qk?
function predict_cov(F, P_prev)
    Qk = diagm(ones(Float64, 7))
    P = F * P_prev * transpose(F) + Qk
end

function residual_meas(zk, x_predicted::ObjectState, camera_array, cur_time, road, out_of_frame)
    yk = zk - get_predicted_bbox(x_predicted, camera_array, cur_time, road, out_of_frame)
end

# TODO - alter Rk?
function residual_cov(H, P_predicted, zk_length)
    Rk = diagm(ones(Float64, length(zk_length)))
    P = H * P_predicted * transpose(H) + Rk
end

function kalman_gain(P_predicted, H, S)
    K = P_predicted * transpose(H) * inv(S)
end

function update_state(x_predicted, K, yk)
    X = x_predicted + K * yk
end

function update_cov(P_predicted, H, K)
    KH = K * H
    P = (diagm(ones(Float64, length(KH))) - KH) * P_predicted
end

# Implementation for one vehicle.
function object_tracker(SENSE::Channel, TRACKS::Channel, EMG::Channel, camera_array, road)
    lines = []

    while true
        sleep(0)
        @return_if_told(EMG)
        meas = @fetch_or_continue(SENSE)

        left_bboxes = meas[1]
        right_bboxes = meas[2]
        if length(left_bboxes) > 0 || length(right_bboxes) > 0
            println()
            println("LEFT: ", left_bboxes)
            println("RIGHT: ", right_bboxes)
            println()

            cur_t = (length(left_bboxes) > 0) ? left_bboxes[1].time : right_bboxes[1].time

            # TODO associate left and right bboxes - for bbox ∈ left_bboxes ...
            z_left = []
            z_right = []
            out_of_frame = 0  # If this value >0, one camera is out of frame

            # TODO alter algo for more than one vehicle on road
            if length(left_bboxes) > 0
                bbox = left_bboxes[1]
                z_left = convert_to_vector(bbox)
            else
                out_of_frame = 1 # Left camera out of frame
            end
            if length(right_bboxes) > 0
                bbox = right_bboxes[1]
                z_right = convert_to_vector(bbox)
            else
                out_of_frame = 2 # Right camera out of frame
            end
            zk = [z_left; z_right] # Allows for zt to change shape if a camera is out of frame

            # Get tracks or initialize
            prev_t = 0
            tracks = Dict{Int, Tuple{ObjectState, Matrix{Float64}}}()
            if isready(TRACKS) # Tracks has a value
                tracks_obj = @fetch_or_continue(TRACKS) # ::TracksMessage
                prev_t = tracks_obj[1] # Timestamp
                tracks = tracks_obj[2] # ::Dict{Int, Tuple{ObjectState, Matrix{Float64}}}
            end

            println("TRACKS ===", tracks)

            # Predict state and covariance
            if length(tracks) > 0
                Δt = get_time(prev_t, cur_t)

                # TODO find asssociated track, if one exists that corresponds
                (x_prev, P_prev) = tracks[1]
                # If no associated track, create new state entry?
                # (x_predicted, P_predicted) = initialize(bbox)

                # If associated track: predict new state and covariance
                x_predicted = state_dynamics(x_prev, v, Δt, P_prev) # = f(x, u)
                F = jac_f(Δt, v, x_prev.θ) # Q: x_prev or x_predicted???
                P_predicted = predict_cov(F, P_prev) # FkPk-1 * Fk' + Qk
            else
                # If no tracks exist yet
                (x_predicted, P_predicted) = initialize(bbox)
            end

            # Update state and covariance
            yk = residual_meas(zk, x_predicted, camera_array, cur_time, road, out_of_frame)
            H = jac_h(Δt, v, θ) # TODO idk what we need here tbh
            S = residual_cov(H, P_predicted, length(zk))
            K = kalman_gain(P_predicted, H, S)
            x = update_state(x_predicted, K, yk)
            P = update_cov(P_predicted, H, K)

            # Save updated state, covariance in tracks
            tracks[1] = (x, P)
            new_TRACKS = TracksMessage(cur_t, tracks)
            @replace(TRACKS, new_TRACKS)
        else
            # No cars in either camera frame, do nothing
        end
    end
end

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

# Converts type BBoxMeas to a vector
function convert_to_vector(bbox::BBoxMeas)
    z = [bbox.left, bbox.top, bbox.right, bbox.bottom]
end

# Converts type ObjectState to a moveable
function convert_to_ms(x_predicted::ObjectState)
    # v does not matter - set to 0
    state = MVector{4, Float64}(x_predicted.x, x_predicted.y, 0, x_predicted.θ) 
    m = Unicycle(state=state,
        length=x_predicted.length,
        width=x_predicted.width,
        height=x_predicted.height) # Other vals are defaulted
end


# ---------------------------

# This function returns an ObjectState with random initial conditions as done in launch_perception
function initialize_state(bbox::BBoxMeas)
    # TODO initialize x,y
    x0 = round((bbox.left + bbox.right)/2)
    y0 = round((bbox.top + bbox.bottom)/2)

    θ0 = rand() * 2.0 * pi - pi
    extra_size = rand()
    w0 = 1.5 + 2.0 * extra_size
    l0 = 3.0 + 6.0 * extra_size
    h0 = 2.0 + 1.0 * extra_size
    
    ObjectState(x0, y0, θ0, w0, l0, h0)
end

# This function updates the vehicle state based off of a previous state estimate saved in tracks
# Q: is this with P or Q?
function state_dynamics(state::ObjectState, v::Float64, Δt::Float64, P)
    next_state = copy(state)
    w = random_sample(P) # Gaussian noise
    next_state += Δt * [v*cos(state.θ), v*sin(state.θ), 0, 0, 0, 0] + w
end

# -------------------

function jac_f(Δt::Float64, v::Float64, θ::Float64)
    F = [1 0 Δt*cos(θ) -Δt*v*sin(θ) 0 0 0;
        0 1 Δt*sin(θ) Δt*v*cos(θ) 0 0 0;
        0 0 1 0 0 0 0;
        0 0 0 1 0 0 0;
        0 0 0 0 1 0 0;
        0 0 0 0 0 1 0;
        0 0 0 0 0 0 1]
end

# what if only in one camera?? alter camera_array?
function get_predicted_bbox(x_predicted::ObjectState, camera_array, cur_time, road)
    moveables = Dict{Int,Movable}()
    moveables[1] = convert_to_ms(x_predicted)
    predicted_bboxes = update_sensor(camera_array, cur_time, moveables, road)
    left_bboxes = predicted_bboxes[1]
    right_bboxes = predicted_bboxes[2]

    # Should only be one entry in each
    z_left = bboxmeas_to_vector(left_bboxes[1])
    z_right = bboxmeas_to_vector(right_bboxes[1])
    z_predicted = [z_left; z_right]
end

# TODO - bounding box expected jacobian
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
    Qk = diagm(ones(Float64, length(P_prev)))
    P = F * P_prev * transpose(F) + Qk
end

function residual_meas(zk, x_predicted::ObjectState, camera_array, cur_time, road)
    yk = zk - get_predicted_bbox(x_predicted, camera_array, cur_time, road)
end

# TODO - alter Rk?
function residual_cov(H, P_predicted)
    Rk = diagm(ones(Float64, length(P_predicted)))
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


function object_tracker(SENSE::Channel, TRACKS::Channel, EMG::Channel, camera_array, road)
    lines = []

    while true
        sleep(0)
        @return_if_told(EMG)
        meas = @fetch_or_continue(SENSE)

        println(meas)

        left_bboxes = meas[1]
        if length(left_bboxes)>1
            println("TIME : ", left_bboxes[1].time)
        end
        println("LEFT:  ", left_bboxes)
        right_bboxes = meas[2]
        println("RIGHT:  ", right_bboxes)

    


       #=  # Q: what if tracks is empty - then tracks message?? do we have access to prev tracks??
        tracks_obj = @fetch_or_continue(TRACKS) # ::TracksMessage
        
        prev_t = tracks_obj[1]
        tracks = tracks_obj[2] # ::Dict{Int, Tuple{ObjectState, Matrix{Float64}}}


        

        #tracks = TracksMessage(...),   timestamp::Float64, tracks::Dict{Int, Tuple{ObjectState, Matrix{Float64}}} # Hold ObjectState, covariance

        # ORG BELOW.. 
        left_bboxes = meas[1]
        right_bboxes = meas[2]

        c1 = camera_array[1]
        c2 = camera_array[2]

        # TODO translate pos estimate to actual points 
 
        measData = Dict{Int, Tuple{Vector{Float64}, Float64}}()
        out_of_frame = 0 # If this value >0, one camera is out of frame
        z_left = []
        z_right = []

        cur_t = left_bboxes[1].time # TODO: change - all same timestep tho

        # TODO associate left and right bboxes
        # for bbox ∈ left_bboxes
        # end
        # for bbox ∈ right_bboxes
        # end

        # TODO alter boolean values accordingly for camera frames
        if length(left_bboxes) == 1
            bbox = left_bboxes[1]
            z_left = bboxmeas_to_vector(bbox)
        else
            out_of_frame = 1 # Left camera out of frame
        end

        if length(right_bboxes) == 1
            bbox = right_bboxes[1]
            z_right = bboxmeas_to_vector(bbox)
        else
            out_of_frame = 2 # Right camera out of frame
        end
        
        zt = [z_left; z_right] # Allows for zt to change shape if a camera is out of frame
        measData[1] = (zt, out_of_frame)

        covData = Dict{Int, Tuple{ObjectState, Vector{Float64}}}() 
        # - so int to measdata, object state, frame, P
        #object state, sigma p - then use measurement data 
        


        # Parse through each associated measurement
        for (key, val) in measData
            zt = val[1]
            out_of_frame = val[2] # Keep track of which camera out of frame, if any

            prev_t = tracks_obj[1]
            tracks = tracks_obj[2] # Dict{Int, ObjectState}
            Δt = get_time(prev_t, cur_t)

            # TODO: get velocity??? 

            # Predict new state and covariance

            # TODO: find associated track, if one exists
            # If associated track: update state entry using new bbox meas
            if length(tracks) >= 1 
                state = tracks[1]
                x = state_dynamics(state, v, Δt, P)
                
                # what to do with q here?
            # If no associated track: create new state entry
            else
                x = initialize_state(bbox) # 

                P_prev = diagm(0.1*ones(Float64, 6))
                Q = diagm(ones(Float64, 6))
            end

            # need to be storing P_prev - in tuple with object state
            # x_predicted = f(x, u)

            # TODO function that calculates v
            # get heading from prev or initial est state
            F = jac_f(Δt, v, Θ)
            P = predict_cov(F, P_prev) # FkPk-1 * Fk' + Qk
            # save p_prev now - track newest COV measurements
            #P = get_P(F, last(P_list)) # FkPk-1 • transpose(Fk) + Qk
            #push!(P_list, P)



            # Update state and covariance
            yk = residual_meas(zk, x_predicted, camera_array, cur_time, road)
            H = jac_h(Δt, v, θ) # idk what we need here tbh
            S = residual_cov(H, P_predicted)
            K = kalman_gain(P_predicted, H, S)
            X = update_state(x_predicted, K, yk)
            P = update_cov(P_predicted, H, K)

            # Set old state = new state
            tracks = tracks_obj[2]
            cur_time = bbox.time
            tracks[1] = TracksMessage(cur_time, X)

            # Print results
            get_predicted_bbox(bbox, updated_state, c1)
        end



        
         =#
        #@replace(TRACKS, tracks)  
    end
end

struct ObjectState
    x::Float64
    y::Float64
    θ::Float64
    length::Float64
    width::Float64
    height::Float64
end

struct ObjectStatePlus
    x::Float64
    y::Float64
    v::Float64
    θ::Float64
    length::Float64
    width::Float64
    height::Float64
end

function position(o::ObjectState)
    [o.x, o.y]
end

function rear(o::ObjectState)
    -o.length/2.0
end
   
function front(o::ObjectState)
    o.length/2.0
end

function heading(o::ObjectState)
    o.θ
end

struct TracksMessage
    timestamp::Float64
    tracks::Dict{Int, ObjectState}
end

# TODO
# 2. Threshold for the matching
# 3. Estimate x0, y0?




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

# Converts type ObjectStatePlus to a moveable
function convert_to_ms(x_predicted::ObjectStatePlus)
    state = MVector{4, Float64}(x_predicted.x, x_predicted.y, x_predicted.v, x_predicted.θ) 
    m = Unicycle(state=state,
        length=x_predicted.length,
        width=x_predicted.width,
        height=x_predicted.height) # Other vals are defaulted
end

# Converts ObjectState to ObjectStatePlus
function convert_to_stateplus(state::ObjectState, v::Float64)
    ObjectStatePlus(state.x, state.y, v, state.θ, state.length, state.width, state.height)
end

# Converts ObjectStatePlus to ObjectState
function convert_to_state(state::ObjectStatePlus)
    ObjectState(state.x, state.y, state.θ, state.length, state.width, state.height)
end

# This function initializes random initial conditions (as done in launch_perception)
function initialize()
    x0 = 0
    y0 = 0
    v0 = 5.0 + rand()*5.0
    θ0 = rand() * 2.0 * pi - pi
    extra_size = rand()
    l0 = 3.0 + 6.0 * extra_size
    w0 = 1.5 + 2.0 * extra_size
    h0 = 2.0 + 1.0 * extra_size
    
    x = ObjectStatePlus(x0, y0, v0, θ0, l0, w0, h0)
    P = diagm(0.1*ones(Float64, 7))
    (x, P)
end

# This function updates the vehicle state based off of a previous state estimate saved in tracks
function state_dynamics(state::ObjectStatePlus, Δt::Float64, P)
    ω = random_sample(P) # Gaussian noise

    x = state.x + Δt*state.v*cos(state.θ) + ω[1]
    y = state.y + Δt*state.v*sin(state.θ) + ω[2]
    v = state.v + ω[3]
    θ = state.θ + ω[4]
    l = state.length + ω[5]
    w = state.width + ω[6]
    h = state.height + ω[7]

    ObjectStatePlus(x, y, v, θ, l, w, h)
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
function get_predicted_bbox(moveables::Dict{Int,Movable}, camera_array, cur_time, road)
    left_bboxes = update_sensor(camera_array[1], cur_time, moveables, road)
    right_bboxes = update_sensor(camera_array[2], cur_time, moveables, road)
    z_left = length(left_bboxes) > 0 ? convert_to_vector(left_bboxes[1]) : []
    z_right = length(right_bboxes) > 0 ? convert_to_vector(right_bboxes[1]) : []
    (z_left, z_right)
end

# Returns the coordinates of the points achieving the corners of bbox
function expected_bbox_pts(camera, pts)
    left = Inf
    top = Inf
    right = -Inf
    bottom = -Inf
    pt_left = []
    pt_top = []
    pt_right =  []
    pt_bottom = []
    for pt ∈ pts
        px = max(min(camera.focal_len * pt[1] / (pt[3] * camera.sx), 1.0), -1.0)
        py = max(min(camera.focal_len * pt[2] / (pt[3] * camera.sy), 1.0), -1.0)
        if py < top
            top = py
            pt_top = pt
        end
        if py > bottom
            bottom = py
            pt_bottom = pt
        end
        if px < left
            left = px
            pt_left = pt
        end
        if px > right
            right = px
            pt_right = pt
        end
    end
    [pt_left, pt_top, pt_right, pt_bottom]
end

# Calculates jacobian of expected bounding box = H for a camera (4x7 or 8x7)
# H = ones(length(z_predicted), 7) # dummy val
function jac_h(x_predicted::ObjectStatePlus, moveables::Dict{Int,Movable}, camera::PinholeCamera)
    θ = x_predicted.θ
    l = x_predicted.length
    w = x_predicted.width
    f = camera.focal_len
    sx = camera.sx
    sy = camera.sy

    pts = get_corners(moveables[1])
    transform!(camera, pts...)
    corner_pts = expected_bbox_pts(camera, pts)
    pt_left = corner_pts[1]
    pt_top = corner_pts[2]
    pt_right = corner_pts[3]
    pt_bottom = corner_pts[4]

    jac_px_x_l = [f/(pt_left[3] * sx) 0.0 (-pt_left[1]/(pt_left[3]^2))*(f/sx)]
    jac_px_x_r = [f/(pt_right[3] * sx) 0.0 (-pt_right[1]/(pt_right[3]^2))*(f/sx)]
    jac_py_x_t = [f/(pt_top[3] * sy) 0.0 (-pt_top[1]/(pt_top[3]^2))*(f/sy)]
    jac_py_x_b = [f/(pt_bottom[3] * sy) 0.0 (-pt_bottom[1]/(pt_bottom[3]^2))*(f/sy)]

    # l, -w, h
    jac_q_x_tl = [1 0 0 (-l*sin(θ)+w*cos(θ))/2 cos(θ)/2 sin(θ)/2 0;
                0 1 0 (l*cos(θ)+w*sin(θ))/2 sin(θ)/2 -cos(θ)/2 0;
                0 0 0 0 0 0 1]
    # -l, w, h
    jac_q_x_br = [1 0 0 -(-l*sin(θ)+w*cos(θ))/2 -cos(θ)/2 -sin(θ)/2 0;
                0 1 0 -(l*cos(θ)+w*sin(θ))/2 -sin(θ)/2 cos(θ)/2 0;
                0 0 0 0 0 0 1] 

    jac_x_q = camera.R 
    jac_l_px = [1.0]
    
    jac_l = jac_l_px * jac_px_x_l * jac_x_q * jac_q_x_tl
    jac_t = jac_l_px * jac_py_x_t * jac_x_q * jac_q_x_tl
    jac_r = jac_l_px * jac_px_x_r * jac_x_q * jac_q_x_br
    jac_b = jac_l_px * jac_py_x_b * jac_x_q * jac_q_x_br

    H = [jac_l; jac_t; jac_r; jac_b]
end

function compute_H(left_predicted, right_predicted, x_predicted, moveables, camera_array)
    H = reshape([],0,7)
    # Only if in camera frame can we compute the jacobian
    if length(left_predicted) > 0 
        H_left = jac_h(x_predicted, moveables, camera_array[1])
        H = vcat(H, H_left)
    end
    if length(right_predicted) > 0 
        H_right = jac_h(x_predicted, moveables, camera_array[2])
        H = vcat(H, H_right)
    end
    println("H: $H")
    H
end

function predict_cov(F, P_prev)
    Qk = diagm(ones(Float64, 7))
    P = F * P_prev * transpose(F) + Qk
end

function residual_meas(zk, z_predicted)
    yk = zk - z_predicted
end

function residual_cov(H, P_predicted, zk_length)
    Rk = diagm(ones(Float64, zk_length))
    P = H * P_predicted * transpose(H) + Rk
end

function kalman_gain(P_predicted, H, S)
    K = P_predicted * transpose(H) * inv(S)
end

function update_state(x_predicted::ObjectStatePlus, K, yk)
    Ky = K * yk

    x = x_predicted.x + Ky[1]
    y = x_predicted.y + Ky[2]
    v = x_predicted.v + Ky[3]
    θ = x_predicted.θ + Ky[4]
    l = x_predicted.length + Ky[5]
    w = x_predicted.width + Ky[6]
    h = x_predicted.height + Ky[7]

    ObjectStatePlus(x, y, v, θ, l, w, h)
end

function update_cov(P_predicted, H, K)
    KH = K * H
    P = (diagm(ones(Float64, size(KH, 1))) - KH) * P_predicted
end

# Computes norm for predicted_bbox to each bbox in bbox_list
function add_asso_matrix_row(predicted_bbox, bbox_list::Vector{BBoxMeas}, asso_matrix)
    row = []
    # If out of frame on one camera, append a row of infinity
    if length(predicted_bbox) == 0
        row = fill(Inf, length(bbox_list))
    else
        for bbox ∈ bbox_list
            norm_bbox = norm(predicted_bbox - convert_to_vector(bbox))
            append!(row, norm_bbox)
        end
    end

    if length(asso_matrix) == 0
        return row'
    else
        return [asso_matrix; row']
    end
end

# Associates left and the minimum norm (first found) right bboxes
function associate_left_right(left_bboxes::Vector{BBoxMeas}, right_bboxes::Vector{BBoxMeas})
    asso_matrix = []
    thres = 0.11

    if length(left_bboxes) > 0 && length(right_bboxes) > 0 # Match left and right together
        for left_bbox ∈ left_bboxes
            row = []
            z_left = convert_to_vector(left_bbox)
        
            for right_bbox ∈ right_bboxes
                norm_bbox = norm(z_left - convert_to_vector(right_bbox))
                append!(row, norm_bbox)
            end

            if length(asso_matrix) == 0
                asso_matrix = row'
            else
                asso_matrix = [asso_matrix; row']
            end
        end

        num_matched = 0
        # Get minimum of each row to match
        for i in 1:length(left_bboxes) 
            match_i = argmin(asso_matrix[i,:]) # Index of min in row
            if asso_matrix[i, match_i] <= thres
                # Consider a match
                asso_matrix[:,match_i] .= Inf # Set column to inf so won't be selected again
                num_matched += 1
            end
        end
    
        return length(left_bboxes) + length(right_bboxes) - num_matched
    elseif length(left_bboxes) > 0 # and length(right) == 0
        return length(left_bboxes)
    else # length(right_bboxes) > 0
        return length(right_bboxes)
    end
end

# If unmatched bboxes, assume new vehicles
function resolve_unmatched_bboxes(left_bboxes, right_bboxes, new_tracks, new_vel_dict, new_cov_dict, k)
    if length(left_bboxes) > 0 && length(right_bboxes) > 0
        # Associate the left and right bboxes so we know how many vehicles to add
        num_vehicles = associate_left_right(left_bboxes, right_bboxes)
        for i in 1:num_vehicles
            (x_predicted, P_predicted) = initialize()
            
            # Store values at next index k
            (new_tracks, new_vel_dict, new_cov_dict, k) = store_vehicle(x_predicted, P_predicted, new_tracks, new_vel_dict, new_cov_dict, k)
        end
    else # Add # of bboxes as cars
        for bbox ∈ left_bboxes
            (x_predicted, P_predicted) = initialize()
            
            # Store values at next index k
            (new_tracks, new_vel_dict, new_cov_dict, k) = store_vehicle(x_predicted, P_predicted, new_tracks, new_vel_dict, new_cov_dict, k)
        end
        for bbox ∈ right_bboxes
            (x_predicted, P_predicted) = initialize()
            
            # Store values at next index k
            (new_tracks, new_vel_dict, new_cov_dict, k) = store_vehicle(x_predicted, P_predicted, new_tracks, new_vel_dict, new_cov_dict, k)
        end
    end

    (new_tracks, new_vel_dict, new_cov_dict)
end

# Stores vehicle information at next index k in their respective dicts
function store_vehicle(x, P, new_tracks, new_vel_dict, new_cov_dict, k)
    new_tracks[k] = convert_to_state(x)
    new_vel_dict[k] = x.v
    new_cov_dict[k] = P
    (new_tracks, new_vel_dict, new_cov_dict)
end


function object_tracker(SENSE::Channel, TRACKS::Channel, EMG::Channel, camera_array, road)
    lines = []

    vel_dict = Dict{Int, Float64}() # Stores velocity
    cov_dict = Dict{Int, Matrix{Float64}}() # Stores covariance
    while true
        sleep(0)
        @return_if_told(EMG)
        meas = @fetch_or_continue(SENSE)

        left_bboxes = meas[1]
        right_bboxes = meas[2]
        num_left_bboxes = length(left_bboxes)
        num_right_bboxes = length(right_bboxes)

        new_tracks = Dict{Int, ObjectState}() # Initialize
        new_vel_dict = Dict{Int, Float64}() # Initialize
        new_cov_dict = Dict{Int, Matrix{Float64}}() # Initialize
        cur_t = 0 # Initialize
    
        if num_left_bboxes > 0 || num_right_bboxes > 0 # No needed update if no new data
            # TODO comment out below
            println("LEFT: $left_bboxes")
            println("RIGHT: $right_bboxes")

            cur_t = (num_left_bboxes > 0) ? left_bboxes[1].time : right_bboxes[1].time
            
            # Get previous tracks if TRACKS has a value
            prev_t = 0 # Initialize
            tracks = []
            if isready(TRACKS)
                tracks_obj = @fetch_or_continue(TRACKS) # ::TracksMessage
                prev_t = tracks_obj.timestamp
                tracks = tracks_obj.tracks # ::Dict{Int, ObjectState}
            end

            if length(tracks) == 0 # No tracks exist yet, but there are cars on the road now
               # Associate the left and right bboxes so we know how many vehicles to add
               num_vehicles = associate_left_right(left_bboxes, right_bboxes)
               for i in 1:num_vehicles
                   (x_predicted, P_predicted) = initialize()
                   
                   # Store values at index i
                   (new_tracks, new_vel_dict, new_cov_dict) = store_vehicle(x_predicted, P_predicted, new_tracks, new_vel_dict, new_cov_dict, i)
               end
            else # Tracks exist
                Δt = get_time(prev_t, cur_t)

                left_asso_matrix = []
                right_asso_matrix = []

                k = 1
                predicted_dict = Dict{Int, Vector{Any}}()
                for i in 1:length(tracks)
                    # Convert from ObjectState to ObjectStatePlus
                    x_prev = convert_to_stateplus(tracks[i], vel_dict[i])
                    P_prev = cov_dict[i]

                    # Predict state
                    x_predicted = state_dynamics(x_prev, Δt, P_prev) # = f(x, u), ::ObjectStatePlus

                    # Use x_predicted to get predicted left and right bboxes
                    moveables = Dict{Int,Movable}(1=>convert_to_ms(x_predicted))
                    (left_predicted, right_predicted) = get_predicted_bbox(moveables, camera_array, cur_t, road)

                    # If vehicle now predicted to be out of frame, do not track
                    if !(length(left_predicted) == 0 && length(right_predicted) == 0)
                        # Predict covariance
                        F = jac_f(Δt, x_prev.v, x_prev.θ)
                        P_predicted = predict_cov(F, P_prev) # FkPk-1 * Fk' + Qk
                        
                        # If we are here, then at least one of num_left, num_right > 0, so it should get an entry in this dictionary
                        predicted_dict[k] = [left_predicted, right_predicted, x_predicted, P_predicted, [], []]
                        k += 1

                        # Construct association matrices, only if there are bboxes to match to
                        if num_left_bboxes > 0
                            left_asso_matrix = add_asso_matrix_row(left_predicted, left_bboxes, left_asso_matrix)
                        end
                        if num_right_bboxes > 0
                            right_asso_matrix = add_asso_matrix_row(right_predicted, right_bboxes, right_asso_matrix)
                        end
                    end
                end

                # -------------------------------------

                # TODO - edge cases

                # 4. Association
                # 5. Resolve resolve_unmatched_bboxes
                # 6. associate left_right
                # if associaitng 2 bounindg bboxes, print them with many cars so we can really see how similar they are lol
                # Verify things flow.... added to matrix?

                # -------------------------------------

                # Associate previous tracks with incoming measurements, then update
                num_predicted = length(predicted_dict)
                println("num predicted: $num_predicted")
                for i in 1:num_predicted

                    # 1. determine some sort of threshold fo better matching?

                    # If num_predicted > num_bboxes, then more vehicles are expected to be in frame than actually are...leaving some predicted_bboxes without matches when they should have one according to our estimate -- whhat would this look like in matrix??? again a threshold might be helpful to get better matches - if min < thres, move on
                    # well if entire row is inf eventually, just returns 1 - so if returns 1 AND val there is inf, thenno match
                    #also the thing returnde ... is it? 

                    # TODO - threshold - maybe by seeing mtach 
                   

                    
                    left_predicted = predicted_dict[i][1]
                    right_predicted = predicted_dict[i][2]
                    z_predicted = [left_predicted; right_predicted]
            
                    # If left_predicted exists and left_asso matrix exists, find a match
                    if length(left_predicted) > 0 && num_left_bboxes > 0 
                        l = argmin(left_asso_matrix[i,:]) # Index of min in row
                        println("left_asso_matrix before: $left_asso_matrix")
                        if l == 1 && left_asso_matrix[i, l] == Inf
                            println("OUT OF MATCHES")
                        end
                        left_asso_matrix[:,l] .= Inf # Set column to inf so won't be selected again
                        println("left_asso_matrix after: $left_asso_matrix")
   
                        predicted_dict[i][5] = convert_to_vector(left_bboxes[l])

                        # if no match - all cols are infso ret 1and inf, then say no match and exit loop -- set  some boolean to all  matched?
                        
                        
                    end

                    # If right_predicted exists and right_asso matrix exists, find a match
                    if length(right_predicted) > 0 && num_right_bboxes > 0 #
                        r = argmin(right_asso_matrix[i,:]) # Index of min in row
                        right_asso_matrix[:,r] .= Inf
                        predicted_dict[i][6] = convert_to_vector(right_bboxes[r])
                    end

                    # Assume new vehicles for unmatched bboxes
                     # If num_predicted < num_bboxes, assume new vehicles entered that are unaccounted for - the unresolved bounding boxes are new vehicles - we can make the algo better by adding a threshold to get more accurate matchings
                    #(new_tracks, new_vel_dict, new_cov_dict) = resolve_unmatched_bboxes(left_bboxes, right_bboxes, new_tracks, new_vel_dict, new_cov_dict, k)

                    # ----------------------------------------------

                    # Update state and covariance if zk (associated meas) exists
                    zk_left = predicted_dict[i][5]
                    zk_right = predicted_dict[i][6]
                    zk = [zk_left; zk_right]
                    if length(left_predicted) > 0 && length(zk_left) == 0
                        # Should've gotten a match, but didn't - then trust associated meas, disregard left_predicted so we have matching dims
                        z_predicted = right_predicted
                        left_predicted = []
                    end
                    if length(right_predicted) > 0 && length(zk_right) == 0
                        # Should've gotten a match, but didn't - then trust associated meas, disregard right_predicted so we have matching dims
                        z_predicted = left_predicted
                        right_predicted = []
                    end

                    println("zk: $zk")
                    println("z_p: $z_predicted")
                    # If no matches at all, when there should have been one at least - assume that prediction was wrong and vehicle is now out of frame - do not track
                    if length(zk) != 0
                        x_predicted = predicted_dict[i][3]
                        P_predicted = predicted_dict[i][4]

                        yk = residual_meas(zk, z_predicted)
                        moveables = Dict{Int,Movable}(1=>convert_to_ms(x_predicted))
                        H = compute_H(left_predicted, right_predicted, x_predicted, moveables, camera_array)
                        S = residual_cov(H, P_predicted, length(zk))
                        K = kalman_gain(P_predicted, H, S)
                        x = update_state(x_predicted, K, yk) # ::ObjectStatePlus
                        P = update_cov(P_predicted, H, K)

                        # Store values at next index
                        (new_tracks, new_vel_dict, new_cov_dict) = store_vehicle(x, P, new_tracks, new_vel_dict, new_cov_dict, i)
                    end
                end
            end

            # Replace old values with updated values
            vel_dict = new_vel_dict
            cov_dict = new_cov_dict
            @replace(TRACKS, TracksMessage(cur_t, new_tracks))
        end
    end
end

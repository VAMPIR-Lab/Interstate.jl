function eval_perception(SIM_STATE::Channel, TRACKS::Channel, EMG::Channel, sensors, road; retention=0.25, disp=false, print_gap=10)
    movables = Dict{Float64, Dict{Int, Movable}}()
    sim_state = fetch(SIM_STATE)
    
    track_message = TracksMessage(0.0, Dict{Int, ObjectState}())
    cycles_to_print = print_gap
    total_iters = 0
    average_track_score = 1.0

    while true
        sleep(0)
        @break_if_told(EMG)

        sim_state = @fetch_or_default(SIM_STATE, sim_state)

        sim_time = sim_state[1]
        movables[sim_time] = sim_state[2]
        for t in keys(movables)
            if t < sim_time - retention
                delete!(movables, t)
            end
        end

        track_message = @deepcopy_or_default(TRACKS, track_message)
        
        ts = track_message.timestamp
        tracks = deepcopy(track_message.tracks)

        gts = keys(movables)
        earliest_gt = minimum(gts)
        latest_gt = maximum(gts)

        if ts < earliest_gt
            ts = earliest_gt
            tracks = Dict{Int, ObjectState}()
        end

        t_closest = Inf
        for t ∈ keys(movables)
            if abs(t-ts) < abs(t_closest - ts)
                t_closest = t
            end
        end
        
        ground_truth = deepcopy(movables[t_closest])
        for (id, m) ∈ ground_truth       
            pts_left = get_corners(m)
            pts_right = deepcopy(pts_left)
            transform!(sensors[1], pts_left...)
            transform!(sensors[2], pts_right...)
            if !(infov(pts_left, sensors[1]) ||  infov(pts_right, sensors[1]))
                delete!(ground_truth, id) 
            end
        end
        
        assignments = Dict{Int,ObjectState}()
        for (id, m) ∈ ground_truth
            min_d = Inf
            min_tid = 0
            for (tid, t) ∈ tracks
                centroid_dist = norm(position(m) - [t.x, t.y])
                if centroid_dist < min_d
                    min_d = centroid_dist
                    min_tid = tid
                    min_t = t
                end
            end
            if min_tid ≠ 0
                assignments[id] = min_t
                delete!(tracks, min_tid)
            end
        end
        
        total_iou = 0.0
        assigned_ids = keys(assignments)
        for (id, m) ∈ ground_truth
            iou = (id ∈ assigned_ids) ? intersection_over_union(m, assignments[id]) : 0.0
            total_iou += iou
        end

        fp = length(tracks)
        if length(ground_truth) == 0 
            track_score = 1.0 / (1 + fp)
        else
            track_score = total_iou / (length(ground_truth) * (1 + fp))
        end

        total_iters += 1
        average_track_score = ((total_iters - 1) * average_track_score + track_score) / total_iters

        if disp
            if cycles_to_print ≤ 0
                print("\e[2K")
                print("\e[1G")
                @printf("Current time is %f. Track eval delay: %f. Average score is %f.", latest_gt, latest_gt-t_closest, average_track_score) 
                cycles_to_print = print_gap
            else
                cycles_to_print -= 1
            end
        end
    end
end


function get_road_angle(ego::Movable, road)
    pos = position(ego)
    vec = pos - road.segments[1].center
    θ = atan(vec[2], vec[1])
end


function eval_racing(SIM_STATE::Channel, EMG::Channel, road; disp=false, print_gap=10, time_limit = 60.0)
    total_turns = 0.0     
    sim_state = fetch(SIM_STATE)
    initial_time = sim_state[1]

    ego = sim_state[2][1]
    θ_prev = get_road_angle(ego, road)
    cycles_to_print = print_gap

    while true
        sleep(0)
        @break_if_told(EMG)

        sim_state = @fetch_or_default(SIM_STATE, sim_state)

        sim_time = sim_state[1]
        ego = sim_state[2][1]
        θ = get_road_angle(ego, road)

        added_turns = wrap(θ - θ_prev)

        total_turns += added_turns
        θ_prev = θ
           
        time_expired = (sim_time > initial_time + time_limit)

        if time_expired || (disp && cycles_to_print ≤ 0)
            print("\e[2K")
            print("\e[1G")
            print("Total turns completed in time limit: ", total_turns / (2π))
            cycles_to_print = print_gap
        elseif disp
            cycles_to_print -= 1
        end
        if time_expired 
            @replace(EMG, 1)
            break
        end
    end
end

function eval_localization(SIM_STATE::Channel, LOCALIZE::Channel, EMG::Channel; retention=0.25, print_gap=10, disp=true)
    sim_state = fetch(SIM_STATE)
    sim_time = sim_state[1]
    ego = sim_state[2][1]
    ego_states = Dict{Float64, Movable}()
    ego_states[sim_time] = ego
         
    localization_message = nothing

    cycles_to_print = print_gap
    total_iters = 0
    average_localization_err = 0.0

    while true
        sleep(0)
        @break_if_told(EMG)

        sim_state = @fetch_or_default(SIM_STATE, sim_state)

        sim_time = sim_state[1]
        ego = sim_state[2][1]
        ego_states[sim_time] = ego

        for t in keys(ego_states)
            if t < sim_time - retention
                delete!(ego_states, t)
            end
        end

        localization_message = @fetch_or_default(LOCALIZE, localization_message)
       
        if !isnothing(localization_message)
            ts = localization_message.timestamp
            pos = localization_message.position
        else
            ts = 0.0
            pos = [0.0,0.0]
        end
        
        gts = keys(ego_states)
        earliest_gt = minimum(gts)
        latest_gt = maximum(gts)

        if ts < earliest_gt
            ts = earliest_gt
            pos = [0.0, 0.0]
        end

        t_closest = Inf
        for t ∈ gts
            if abs(t-ts) < abs(t_closest - ts)
                t_closest = t
            end
        end

        localization_err = norm(position(ego_states[t_closest]) - pos)
        total_iters += 1
        average_localization_err = ((total_iters - 1) * average_localization_err + localization_err) / total_iters
        if disp
            if cycles_to_print ≤ 0
                print("\e[2K")
                print("\e[1G")
                @printf("Current time is %f. Track eval delay: %f. Average error is %f.", latest_gt, latest_gt-t_closest, average_localization_err) 
                cycles_to_print = print_gap
            else
                cycles_to_print -= 1
            end
        end
    end
end

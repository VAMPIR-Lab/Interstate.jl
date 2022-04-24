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

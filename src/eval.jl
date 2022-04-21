
function eval_perception(SIM_STATE::Channel, TRACKS::Channel, EMG::Channel, sensors, road; retention=0.25, disp=false, print_gap=10)
    movables = Dict{Float64, Dict{Int, Movable}}()
    sim_state = fetch(SIM_STATE)
    
    track_message = TracksMessage(0.0, Dict{Int, ObjectState}())
    cycles_to_print = print_gap

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

        track_message = @fetch_or_default(TRACKS, track_message)
        
        ts = track_message.timestamp
        tracks = track_message.tracks

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
        
        fp = 0
        md = 0

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

        md = length(ground_truth) - length(tracks)

        if disp
            if cycles_to_print ≤ 0
                print("\e[2K")
                print("\e[1G")
                @printf("Current time is %f. Track eval delay: %f. Number of missed detections is %d.", latest_gt, latest_gt-t_closest, md) 
                cycles_to_print = print_gap
            else
                cycles_to_print -= 1
            end

        end
    end
end

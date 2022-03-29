function fleet_controller(CMD::Channel, SENSE::Channel, EMG::Channel, road; K₁=0.5, K₂=.5, disp=true)
    while true
        sleep(0.001)
        if length(EMG.data) > 0
            return
        end

        if length(SENSE.data) > 0
            meas = fetch(SENSE)
        else
            continue
        end
        command = Dict{Int, VehicleControl}()
        for (id, m) ∈ meas
            seg = road.segments[m.road_segment_id]
            cte, ctv = get_crosstrack_error(m.position, m.heading, m.speed, m.target_lane, seg, road.lanes, road.lanewidth)
            δ = -K₁*cte-K₂*ctv
            δ = max(min(δ, π/4.0), -π/4.0)
            command[id] = [0.0, δ]
        end

        while length(CMD.data) > 0
            take!(CMD)
        end
        put!(CMD, command)
    end
end

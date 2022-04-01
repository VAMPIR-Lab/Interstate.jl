function fleet_controller(CMD::Dict{Int,Channel{T}}, SENSE::Channel, EMG::Channel, road; K₁=0.5, K₂=.5, disp=true) where T
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
            while length(CMD[id].data) > 0
                take!(CMD[id])
            end
            put!(CMD[id], [0.0, δ])
        end
    end
end

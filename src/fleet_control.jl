function fleet_controller(CMD::Dict{Int,Channel{T}}, SENSE::Channel, EMG::Channel, road; K₁=0.5, K₂=.5, disp=true) where T
    while true
        sleep(0.001)
        @return_if_told(EMG)

        meas = @fetch_or_continue(SENSE)

        command = Dict{Int, VehicleControl}()
        for (id, m) ∈ meas
            seg = road.segments[m.road_segment_id]
            cte, ctv = get_crosstrack_error(m.position, m.heading, m.speed, m.target_lane, seg, road.lanes, road.lanewidth)
            δ = -K₁*cte-K₂*ctv
            command = [0.0 max(min(δ, π/4.0), -π/4.0)]
            @replace(CMD[id], command)
        end
    end
end

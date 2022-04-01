function object_tracker(SENSE::ChannelLock, TRACKS::ChannelLock, EMG::ChannelLock, scene, camera_array, road)
    lines = []

    while true
        sleep(0.001)
        @return_if_told(EMG)
        meas = @fetch_or_continue(SENSE)

        for line ∈ lines
            delete!(scene, line)
        end
        lines = []

        left_bboxes = meas[1]
        right_bboxes = meas[2]

        for bbox ∈ meas[1]
            line = draw_bbox_2_world(scene, camera_array[1], bbox, z=10.0, linewidth=5)
            push!(lines, line)
        end

        tracks = Dict{Int, OracleMeas}()
        #TODO your code here

        @replace(TRACKS, tracks)    
    end
end

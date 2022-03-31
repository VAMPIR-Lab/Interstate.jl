function object_tracker(SENSE::Channel, TRACKS::Channel, EMG::Channel, scene, camera_array, road)
    lines = []

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

        
        while length(TRACKS.data) > 0
            take!(TRACKS)
        end
        put!(TRACKS, tracks)
    end
end

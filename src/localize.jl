function localize(SENSE::Channel, EMG::Channel, scene, lidar, road)
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

        lines = draw_lidar_beams_2_world(scene, lidar, meas)

        # TODO
        # Your code here
        
     
        

    end
end

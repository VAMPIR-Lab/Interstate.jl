function localize(SENSE::ChannelLock, EMG::ChannelLock, scene, lidar, road; disp=false)
    lines = []
    while true
        sleep(0)
        @return_if_told(EMG)
        if disp
            for line ∈ lines
                delete!(scene, line)
            end

            lines = draw_lidar_beams_2_world(scene, lidar, meas)
        end

        # TODO
        # Your code here
        
     
        

    end
end



function localize(SENSE::ChannelLock, LOCALIZE::ChannelLock, EMG::ChannelLock, lidar, road)
    while true
        sleep(0)
        @return_if_told(EMG)

        # TODO
        # Your code here
        
        # ego_state = ... 
        #@replace(LOCALIZE, ego_state) 

    end
end

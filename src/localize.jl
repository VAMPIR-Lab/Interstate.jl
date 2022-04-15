function localize(SENSE_LIDAR::Channel, SENSE_GPS::Channel, LOCALIZE::Channel, EMG::Channel, lidar, road, buildings)
    while true
        sleep(0)
        @return_if_told(EMG)

        # TODO
        # Your code here
        
        #meas = @fetch_or_continue(SENSE)

        #list_of_points = meas.points
        #lidar_center = meas.origin
        #time_of_meas = meas.time

        
        # ego_state = ... 
        #@replace(LOCALIZE, ego_state) 

    end
end

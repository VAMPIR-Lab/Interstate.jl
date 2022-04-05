function object_tracker(SENSE::ChannelLock, TRACKS::ChannelLock, EMG::ChannelLock, camera_array, road)
    lines = []

    while true
        sleep(0)
        @return_if_told(EMG)
        meas = @fetch_or_continue(SENSE)

        tracks = Dict{Int, OracleMeas}()
        #TODO your code here

        @replace(TRACKS, tracks)    
    end
end

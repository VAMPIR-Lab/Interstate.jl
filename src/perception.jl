
function object_tracker(SENSE::Channel, TRACKS::Channel, EMG::Channel, road)
    t = Terminals.TTYTerminal("", stdin, stdout, stderr) 
    while true
        sleep(0.001)
        if length(EMG.data) > 0
            return
        end

        if length(SENSE.data) > 0
            meas = fetch(SENSE)
        else
            println("nada")
            continue
        end
        Terminals.clear(t)
        println(length(meas), " bboxes detected.")


        tracks = Dict{Int, OracleMeas}()
        #TODO your code here
        
        while length(TRACKS.data) > 0
            take!(TRACKS)
        end
        put!(TRACKS, tracks)
    end
end

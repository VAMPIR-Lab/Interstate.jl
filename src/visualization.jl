function update_display!(viewables, movables, closest_ids)
    for (i,id) ∈ enumerate(closest_ids)
        pts = get_corners(movables[id])
        viewables[i][1][] = pts
        viewables[i][2][] = movables[id].color
    end
end

function find_closest!(ids, movables, n)
    dists = []
    for (i, m) ∈ movables
        push!(dists, (norm(position(movables[1])-position(movables[i])), i))
    end

    sort!(dists; alg=Base.Sort.PartialQuickSort(n), by=x->x[1])
    for i ∈ 1:length(ids)
        ids[i] = dists[i][2]
    end
end

function visualize(SIM::ChannelLock, EMG::ChannelLock, viewables, camera)  
    num_viewed = length(viewables)
    closest_ids = zeros(Int, num_viewed)
    println("Visualizing on thread ", Threads.threadid())
    while true
        sleep(0)
        @return_if_told(EMG) 
        (time,movables) = @fetch_or_continue(SIM)
    
        find_closest!(closest_ids, movables, num_viewed)
        update_display!(viewables, movables, closest_ids)
        if !isnothing(camera)
            pos = position(movables[1])
            θ = heading(movables[1])
            camera.x[] = pos[1]
            camera.y[] = pos[2]
            camera.θ[] = θ
        end
    end
end

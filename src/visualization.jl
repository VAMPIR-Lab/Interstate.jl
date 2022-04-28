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

function visualize(MEAS::Channel{PointCloud},
                   EMG::Channel,
                   lidar::Lidar,
                   scene::Scene)
    lines = []
    while true
        sleep(0)
        @return_if_told(EMG)
        meas = @fetch_or_continue(MEAS)
        origin = meas.origin

        for line ∈ lines
            delete!(scene, line)
        end
        lines = []
        for pt ∈ meas.points
            line = lines!(scene, [origin[1],pt[1]],[origin[2],pt[2]], [origin[3],pt[3]], color=:red, linewidth=3)
            push!(lines, line)
        end
    end
end

function visualize(MEAS::Channel{Dict{Int, Vector{BBoxMeas}}},
                   EMG::Channel,
                   camera_array::Dict{Int, PinholeCamera},
                   scene::Scene)
    lines = []
    while true
        sleep(0)
        @return_if_told(EMG)
        meas = @fetch_or_continue(MEAS)
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
    end 
end


function visualize(SIM::Channel, 
                   EMG::Channel,
                   viewables, 
                   follow_cam)
    num_viewed = length(viewables)
    closest_ids = zeros(Int, num_viewed)
    println("Visualizing on thread ", Threads.threadid())
    while true
        sleep(0)
        @return_if_told(EMG) 
        (time,movables) = @fetch_or_continue(SIM)
    
        find_closest!(closest_ids, movables, num_viewed)
        update_display!(viewables, movables, closest_ids)
        if !isnothing(follow_cam)
            pos = position(movables[1])
            θ = heading(movables[1])
            follow_cam.x[] = pos[1]
            follow_cam.y[] = pos[2]
            follow_cam.θ[] = θ
        end
    end
end

function visualize(TRACKS::Channel{TracksMessage}, 
                   EMG::Channel,
                   viewables, 
                   dummy_pts)

    num_viewed = length(viewables)
    closest_ids = zeros(Int, num_viewed)
    println("Visualizing on thread ", Threads.threadid())
    while true
        sleep(0)
        @return_if_told(EMG) 
        tracks_msg = @fetch_or_continue(TRACKS)
        id = 1
        for (tid, track) ∈ tracks_msg.tracks
            pts = get_corners(track)
            viewables[id][1][] = pts
            id += 1
        end
        while id < num_viewed
            viewables[id][1][] = dummy_pts
            id += 1
        end
    end
end

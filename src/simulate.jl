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

function collision(m1::Movable, m2::Movable)
    b1 = Box2(m1)
    b2 = Box2(m2)
    intersect(b1,b2).collision
end

function collision(movables, closest_ids)
    for id ∈ closest_ids
        if id != 1 
            col = collision(movables[1], movables[id])
            if col
                return true
            end
        end
    end
    return false
end

function road_violation(m::Movable, road)
    segment_id = road_segment(m, road)  
    mbox = Box2(m)
    seg = road.segments[segment_id]
    if isa(seg, CurvedSegment)
        if seg.θ₁ ≤ seg.θ₂
            inner = Circle(seg.center..., seg.radius)
            outer = Circle(seg.center..., seg.radius+road.lanes*road.lanewidth)
        else
            outer = Circle(seg.center..., seg.radius)
            inner = Circle(seg.center..., seg.radius-road.lanes*road.lanewidth)
        end
        c1 = inside(mbox, outer)
        c2 = !intersect(mbox, inner).collision
        violation = !inside(mbox, outer) || intersect(mbox, inner).collision
        return violation
    else
        rbox = Box2(seg)
        return inside(mbox, rbox)
    end
end

function update_display!(viewables, movables, closest_ids)
    for (i,id) ∈ enumerate(closest_ids)
        pts = get_corners(movables[id])
        viewables[i][1][] = pts
        viewables[i][2][] = movables[id].color
    end
end

struct Simulator
    movables::Dict{Int,Movable}
    viewables
    camera
    road
end

function simulate(sim::Simulator, emg, channel;
                  Δ=0.001,              
                  disp=false,
                  check_collision=true,
                  check_road_violation=[],
                  print_increment=0.01,
                  )

    t0 = time_ns()
    simulated_time = 0.0
    display_time = 0.0
    num_viewed = length(sim.viewables)
    closest_ids = zeros(Int, num_viewed)
    #try
        while true
            sleep(0.001)
            simulated_time += Δ
            display_time += Δ

            for (id, movable) ∈ sim.movables
                update_command!(movable)
                update_state!(movable, Δ)
            end

            while length(channel.data) > 0
                take!(channel)
            end
            put!(channel, (simulated_time, sim.movables))

            find_closest!(closest_ids, sim.movables, num_viewed) 
            if check_collision && collision(sim.movables, closest_ids)
                println()
                println("Collision!")
                put!(emg, 1)
                return
            end

            if length(check_road_violation) > 0
                for id ∈ check_road_violation
                    if road_violation(sim.movables[id], sim.road)
                        println()
                        println("Road boundary violation!")
                        put!(emg, 1)
                        return
                    end
                end
            end

            update_display!(sim.viewables, sim.movables, closest_ids)
            if !isnothing(sim.camera)
                sim.camera.x[] = sim.movables[1].state[1]
                sim.camera.y[] = sim.movables[1].state[2]
                sim.camera.θ[] = sim.movables[1].state[4]
            end

            if display_time > print_increment
                if disp
                    print("\e[2K")
                    print("\e[1G")
                    @printf("Loop time: %f.", Δ)
                end
                display_time -= print_increment
            end
            if length(emg.data) > 0
                println()
                println("Interrupt!")
                return
            end
            err = (time_ns()-t0)/1e9 - simulated_time
            Δ = max(0.0, min(5e-1, err))
        end
    #catch e #    if e isa InterruptException
    #        println(e)
    #        println()
    #        println("Interrupt!")
    #end
end


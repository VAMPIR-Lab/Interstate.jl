function collision(m1::Movable, m2::Movable)
    b1 = Box2(m1)
    b2 = Box2(m2)
    intersect(b1,b2).collision
end

function collision(movables; max_dist=50.0)
    pos1 = position(movables[1])
    for (id, movable) ∈ movables
        if id != 1
            dist = norm(pos1 - position(movable))
            if dist < max_dist && collision(movables[1], movable)
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
        rbox = Box2(road, segment_id)
        violation = !inside(mbox, rbox)
        return violation
    end
end

struct Simulator
    movables::Dict{Int,Movable}
    road
end

function simulate(sim::Simulator, emg, channel;
                  Δ=0.001,              
                  disp=false,
                  check_collision=true,
                  check_road_violation=[],
                  print_increment=0.1,
                  log_survival_time=false,
                  )

    t0 = time_ns()
    simulated_time = 0.0
    display_time = 0.0
    println("Simulating on thread ", Threads.threadid())
    iters = 0
    while true
        sleep(0)
        @break_if_told(emg)
        simulated_time += Δ
        display_time += Δ
        iters += 1

        for (id, movable) ∈ sim.movables
            update_command!(movable)
            update_state!(movable, Δ)
        end
       
        @replace(channel, (simulated_time, sim.movables))

        if check_collision && collision(sim.movables)
            println()
            println("Collision!")
            @replace(emg, 1)
            break
        end

        if length(check_road_violation) > 0
            for id ∈ check_road_violation
                if road_violation(sim.movables[id], sim.road)
                    println()
                    println("Road boundary violation!")
                    @replace(emg, 1)
                    break
                end
            end
        end

        if display_time > print_increment
            if disp
                print("\e[2K")
                print("\e[1G")
                @printf("Loop time: %f.", simulated_time / iters)
            end
            display_time -= print_increment
        end
        err = (time_ns()-t0)/1e9 - simulated_time
        Δ = max(0.0, min(5e-1, err))
    end
    if log_survival_time
        println("Survived for ", simulated_time, " seconds.")
    end
end


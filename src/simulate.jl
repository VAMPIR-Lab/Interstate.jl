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
    sensors::Dict{Int,Sensor}
    viewables
    camera
    road
end

function simulate(sim::Simulator, e;
                  Δ=0.001,              
                  disp=false,
                  check_collision=true,
                  check_road_violation=[],
                  K = 0.01,
                  print_increment=0.01,
                  )

    t0 = time_ns()
    simulated_time = 0.0
    display_time = 0.0
    num_viewed = length(sim.viewables)
    closest_ids = zeros(Int, num_viewed)
    if length(sim.movables) > 1
        CMD_FLEET = sim.movables[2].channel
    else
        CMD_FLEET = Channel(0)
    end
    #try
        while true
            sleep(0.001)
            simulated_time += Δ
            display_time += Δ

            if length(CMD_FLEET.data) > 0
                fleet_control = take!(CMD_FLEET)
                for (id, cmd) ∈ fleet_control
                    sim.movables[id].control .= cmd
                end
            end  
            for (id, movable) ∈ sim.movables
                if id == 1
                    update_command!(movable)
                end

                update_state!(movable, Δ)
            end


            find_closest!(closest_ids, sim.movables, num_viewed) 

            for (id, sensor) ∈ sim.sensors
                update_sensor(sensor, simulated_time, sim.movables,sim.road)
            end

            if check_collision && collision(sim.movables, closest_ids)
                println()
                println("Collision!")
                put!(e, 1)
                return
            end

            if length(check_road_violation) > 0
                for id ∈ check_road_violation
                    if road_violation(sim.movables[id], sim.road)
                        println()
                        println("Road boundary violation!")
                        put!(e, 1)
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
            if length(e.data) > 0
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


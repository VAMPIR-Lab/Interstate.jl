
# generate a random particle
function feasible_point(gps_position)
    pointEstimate =  [gps_position[1], gps_position[2]]
    pointEstimate += 5 * randn(2)

    theta = (rand() * 2*pi) - pi
    vel = rand() * 5

    particle = [pointEstimate[1], pointEstimate[2], theta, vel]
    return particle
end


# calculates new position given the change in angular velocity and acceleration
function update_position(p, deltat, u)
    new_particle = p

    x = p[1]
    y = p[2]
    theta = p[3]
    vel = p[4]

    new_particle[1] += deltat * cos(theta) * vel
    new_particle[2] += deltat * sin(theta) * vel
    new_particle[3] += deltat * u[2]
    new_particle[4] += deltat * u[1]

    return new_particle
end 



function sample_from(weights)
    @assert sum(weights) ≈ 1.0 #make sure weights are normalized
    @assert all(weights .≥ 0)

    r = rand()
    t = 0
    ind = 0 
    while t < r
        ind += 1 
        t += weights[ind]
    end
    ind
end



function compare_expected_and_actual_lidar(point_cloud_expected, point_cloud_actual)
    total_error = 0
    for (pt_e, pt_a) ∈ zip(point_cloud_expected, point_cloud_actual)
        total_error += norm(pt_e - pt_a)
    end
    return -total_error
end



function expected_lidar_return(pos, α_min, ϕ, θ, ms, road)
    x = cos(θ)
    y = sin(θ)
    z = -atan(ϕ, 1.0)
    beam = [x, y, z]
    beam /= norm(beam)
    ray = Ray3(pos, beam)
    α_ground = abs(pos[3]/beam[3])
    α_min = min(α_min, α_ground)
    pt_min = pos + α_min * beam
    for (id, m) ∈ ms
        box = Box3(m)
        #(; collision, p, α) = intersect(box, ray, max_dist=α_ground)
        (; collision, p, α) = intersect(box, ray, max_dist=Inf)
        if collision && α < α_min
            α_min = α
            pt_min = p
        end
    end
    return pt_min
end



function expected_lidar_beams(sensor::Lidar, x, y, θ, buildings, road; height=1.5)
    lidar_pos = [x, y, height] + sensor.offset
    θ₀ = θ
    pts = Vector{SVector{3, Float64}}()
    N = sensor.angular_resolution
    angles = LinRange(-π, π-2*π/N, N)
    for ϕ ∈ sensor.beam_elevations
        for θ ∈ angles
            pt = expected_lidar_return(lidar_pos, sensor.max_beam_length, ϕ, θ+θ₀, buildings, road)
            push!(pts, pt)
        end
    end
    meas = PointCloud(pts, lidar_pos, 0)
end



function localize(SENSE_LIDAR::Channel, SENSE_GPS::Channel, LOCALIZE::Channel, EGO_CMD::Channel, EMG::Channel, lidar, road, buildings)
    meas = fetch(SENSE_LIDAR)
    ego_meas = fetch(EGO_CMD)
    gps = fetch(SENSE_GPS)

    
    num_particles = 100
    t_f = time_ns()/1e9
    
    #generate random particles
    particles = [zeros(4) for _ in 1:num_particles]
    for i in 1:num_particles
        particles[i] = feasible_point(gps.position)
    end

    p_birth = 0.1

    while true
        sleep(0)
        @return_if_told(EMG)
        
        meas = @fetch_or_default(SENSE_LIDAR, meas)
        ego_meas = @fetch_or_default(EGO_CMD, ego_meas)
        gps = @fetch_or_default(SENSE_GPS, gps)
        

        #get deltat
        current_time = (time_ns()/1e9)
        deltat = current_time - t_f
      
        
        dists = meas.points

        #calculate new position of ego_vehicle
        d2_origin = [meas.origin[1], meas.origin[2], 0, 0]
        new_position = update_position(d2_origin, deltat, ego_meas)
        final_position = [new_position[1], new_position[2]]
    
        #update position of particles
        for particle in particles
            update_position(particle, deltat, ego_meas)
        end 

        #get weights of the particles
        candidate_weights = []
        for particle ∈ particles
            meas = expected_lidar_beams(lidar, particle[1], particle[2], particle[3], buildings, road)
            expected_meas = meas.points
            weight = compare_expected_and_actual_lidar(expected_meas, dists)

            push!(candidate_weights, weight)
        end


        #normalize weights to one
        candidate_weights = candidate_weights ./sum(candidate_weights)

        #re-sample the particles
        for particle ∈ particles 
            r = rand()
            if r < p_birth 
                particle[] = feasible_point(gps_meas)
            else
                particle[] = candidate_particles[sample_from(candidate_weights)]
            end
        end
       
        #replace position of ego_vehicle
        ego_state = OracleMeas(final_position, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0)
        print(final_position) 
        @replace(LOCALIZE, ego_state) 

        t_f = current_time
    end
end

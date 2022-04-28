
function feasible_point(gps_position)
    pointEstimate = gps_position
    for i in gps_position
    # generated rand +/-1 around gps_pos. Adjust rangeVal to adjust area covered
        rangeVal = 2 #noise.std
        pointEstimate[i] = rangeVal*rand() .+ (gps_position[i]-(rangeVal/2))
     #make sure nothing is out of map bounds
        if pointEstimate[i]>40
            pointEstimate[i]=40
        end
    end

    theta = (rand() * 2*pi) - pi
    vel = rand() * 5

    particle = [pointEstimate[1], pointEstimate[2], theta, vel]
    return particle
end



function make_2d(points)
    #3x2 matrix, each row stores points. Easier to access that point cloud
    V = SVector{2, Float64}
        #goes through every vector point
    for i in points
        #goes through the x and y value of every vector point
        for j ∈ 1:2
            V[i][j] = points[i][j]
        end
    end
    V
end



function update_position!(p, deltat, u)
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



function compute_range(pos, θ, buildings, max_beam_length; σ = 0.05)
    c = cos(θ)
    s = sin(θ)
    α_min = max_beam_length
    for seg ∈ buildings

        A = [c seg[2][1]-seg[1][1];
             s seg[2][2]-seg[1][2]]
        b = seg[2] - [pos[1],pos[2]]
        α = A\b
        if 0 ≤ α[2] ≤ 1 && α[1] ≥ 0 # beam intersects map segment
            α_range = α[1] + randn()*σ
            α_min = min(α_min, α_range)
        end
    end

    r = rand()
    if r < p_missed
        α_min = α_max
    elseif r < p_missed + p_scatter
        α_min = α_max * rand()
    end
    (Point2(pos[1]+α_min*c, pos[2]+α_min*s), α_min)
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



function expected_prob(α_meas, α_expec, σ, α_max)

    if ((0 ≤ α_meas) && (α_meas ≤ α_max))
        norm_constant = (0.5 * (1 +  erf((α_max - α_expec) / (σ * sqrt(2))))) - (0.5 * (1 +  erf((0 - α_expec) / (σ * sqrt(2)))))
        return ((1 / (σ * sqrt(2 * π))) * exp(-0.5 * (((α_meas - α_expec)/σ)^2))) * (1/norm_constant)
    else 
        return 0
    end
end



function scatter_prob(α_meas, α_max)
    if ((0 ≤ α_meas) && (α_meas ≤ α_max))
        return 1/α_max
    else 
        return 0.0
    end
end



function missed_prob(α_meas, α_max)
    if(α_meas == α_max)
         return 1
    else 
        return 0
    end
end



function measurement_prob(pos, 
                          dists,
                          buildings, 
                          max_beam_length;
                          σ=0.25,
                          p_missed=0.001,
                          p_scatter=0.001,
                          sub_sample_factor::Int=1)
    Nb = length(dists)
    p = 1.0
    p_expected = 1.0 - p_missed - p_scatter
    for n ∈ 1:sub_sample_factor:Nb
        θ = 2*π*n/Nb
        _, expected_dist = compute_range(pos, θ, buildings, max_beam_length; σ=0.0, p_missed=0.0, p_scatter=0.0)
        measured_dist = dists[n]
        pₙ = p_expected * expected_prob(measured_dist, expected_dist, σ, max_beam_length) + 
             p_missed * missed_prob(measured_dist, max_beam_length) +
             p_scatter * scatter_prob(measured_dist, max_beam_length)
        p *= pₙ
    end
    p  
end



function update_particles!(particles, dists, delta, buildings, max_beam_length; σ_move=0.3, ssf = 10, p_birth=0.1)
    candidate_particles = []
    candidate_weights = []
    for particle ∈ particles
        old_position = [particle[][1], particle[][2]] 
        new_position = update_position(old_position, delta, ego_meas)
        weight = measurement_prob(new_position, 
                                dists,
                                buildings,
                                max_beam_length;
                                σ=σ_move,
                                p_missed=0.001,
                                p_scatter=0.001,
                                sub_sample_factor=ssf)
        
        push!(candidate_particles, new_position)
        push!(candidate_weights, weight)
    end

    #normalize weights to one
    candidate_weights = candidate_weights ./sum(candidate_weights)
    
    for particle ∈ particles 
        r = rand()
        if r < p_birth 
            particle[] = feasible_point()
        else
            particle[] = candidate_particles[sample_from(candidate_weights)]
        end
    end
end



function localize(SENSE_LIDAR::Channel, SENSE_GPS::Channel, LOCALIZE::Channel, EGO_CMD::Channel, EMG::Channel, lidar, road, buildings)
    meas = fetch(SENSE_LIDAR)
    ego_meas = fetch(EGO_CMD)
    gps = fetch(SENSE_GPS)

    # define time variable here and give it a default
    #initial_time = meas.time

    Δ=0.001
    t0 = time_ns()
    deltat = 0.0

    while true
        sleep(0)
        @return_if_told(EMG)
        
        meas = @fetch_or_default(SENSE_LIDAR, meas)
        ego_meas = @fetch_or_default(EGO_CMD, ego_meas)
        gps = @fetch_or_default(SENSE_GPS, gps)
        

        #time since last iteration 
        #need to capture the difference
        #capture time here
        #deltat = rand()

        deltat += Δ

        # generate random particles within gps_position. 
        # return for feasible point - [x,y] points for a possible article. 
        # return for particles - [[x1,y1], [x2,y2], .... ]
        particles = [feasible_point(gps.position) for _ in 1:100]

        for particle in particles
            update_position(particle, deltat, ego_meas);
        end 
      
        #convert to 2D
        dists = make_2d(meas.points)

       
        # origin::SVector{3, Float64}
        d2_origin = [meas.orgin[1], meas.origin[2], 0, 0]

        new_position = update_position(d2_origin, deltat, ego_meas)
        final_position = [new_position[1], new_position[2]]
    
        update_particles!(particles, dists, deltat, buildings, ego_meas, lidar.max_beam_length)
       
        ego_state = OracleMeas(final_position, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0)
        print(final_position) 
        @replace(LOCALIZE, ego_state) 

        err = (time_ns()-t0)/1e9 - deltat
        Δ = max(0.0, min(5e-1, err))

    end
end




 















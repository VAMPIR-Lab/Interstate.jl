function getc1()
    ret = ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, true)
    ret == 0 || error("unable to switch to raw mode")
    c = read(stdin, Char)
    ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, false)
    c
end

function keyboard_broadcaster(KEY::ChannelLock, EMG::ChannelLock)
    println("Press 'q' at any time to terminate simulator.")
    while true
        @return_if_told(EMG)
        key = getc1()
        @replace(KEY, key)
        if key == 'q'
            @replace(EMG, 1)
            return
        end
    end
end

function wrap(θ)
    mod(θ += pi, 2*pi) - pi
end

function clip(x, l)
    max(-l, min(l, x))
end

function controller(KEY::ChannelLock, CMD::ChannelLock, SENSE::ChannelLock, EMG::ChannelLock; K1=5, K2=.5, disp=false, V=0.0, θ = 0.0, V_max = 100.0, θ_step=0.1, V_step = 1.5)
    println("Keyboard controller in use.")
    println("Press 'i' to speed up,")
    println("Press 'k' to slow down,")
    println("Press 'j' to turn left,")
    println("Press 'l' to turn right.")

    while true
        sleep(0.001)
        @return_if_told(EMG)
        
        key = ' '
        lock(KEY.lock)
        try
            if length(KEY.channel.data) > 0
                key = take!(KEY.channel)
            end
        finally
            unlock(KEY.lock)
        end
        meas = @fetch_or_continue(SENSE)
        speed = meas.speed
        heading = meas.heading
        segment = meas.road_segment_id
        if key == 'i'
            V = min(V_max, V+V_step)
        elseif key == 'j' 
            θ += θ_step
        elseif key == 'k'
            V = max(0, V-V_step)
        elseif key == 'l'
            θ -= θ_step
        end
        err_1 = V-speed
        err_2 = clip(θ-heading, π/2)
        cmd = [K1*err_1, K2*err_2]
        @replace(CMD, cmd)
        if disp
            print("\e[2K")
            print("\e[1G")
            @printf("Command: %f %f, speed: %f, segment: %d", cmd..., speed, segment)
        end
    end
end

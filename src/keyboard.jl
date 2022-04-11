function getc1()
    ret = ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, true)
    ret == 0 || error("unable to switch to raw mode")
    c = read(stdin, Char)
    ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, false)
    c
end

function keyboard_broadcaster(KEY::Channel, EMG::Channel)
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

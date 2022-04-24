macro replace(clk, val)
    quote
        local channel = $(esc(clk))
        local value = $(esc(val))
        lock(channel)
        try
            while isready(channel)
                take!(channel)
            end
            put!(channel, value)
        finally
            unlock(channel)
        end
    end
end

macro fetch_or_continue(clk)
    quote
        local channel = $(esc(clk))
        lock(channel)
        try
            isready(channel) ? fetch(channel) : continue
        finally
            unlock(channel)
        end
    end
end

macro fetch_or_default(clk, default)
    quote
        local channel = $(esc(clk))
        local default = $(esc(default))
        lock(channel)
        try
            isready(channel) ? fetch(channel) : default
        finally
            unlock(channel)
        end
    end
end

macro deepcopy_or_default(clk, default)
    quote
        local channel = $(esc(clk))
        local default = $(esc(default))
        lock(channel)
        try
            isready(channel) ? deepcopy(fetch(channel)) : default
        finally
            unlock(channel)
        end
    end
end

macro take_or_default(clk, default)
    quote
        local channel = $(esc(clk))
        local default = $(esc(default))
        lock(channel)
        try
            isready(channel) ? take!(channel) : default
        finally
            unlock(channel)
        end
    end
end

macro fetch_or_return(clk)
    quote
        local channel = $(esc(clk))
        lock(channel)
        try
            if isready(channel)
                fetch(channel)
            else
                return
            end
        finally
            unlock(channel)
        end
    end
end

macro return_if_told(clk)
    quote
        local channel = $(esc(clk))
        if isready(channel)
            return
        end
    end
end

macro break_if_told(clk)
    quote
        local channel = $(esc(clk))
        if isready(channel)
            break
        end
    end
end

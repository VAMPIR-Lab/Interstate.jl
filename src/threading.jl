struct ChannelLock{T}
    channel::Channel{T}
    lock::ReentrantLock
end

function ChannelLock{T}(size::Int) where T
    ChannelLock(Channel{T}(size), ReentrantLock())
end

macro replace(clk, val)
    quote
        local channellock = $(esc(clk))
        local value = $(esc(val))
        lock(channellock.lock)
        try
            while length(channellock.channel.data) > 0
                take!(channellock.channel)
            end
            put!(channellock.channel, value)
        finally
            unlock(channellock.lock)
        end
    end
end

macro fetch_or_continue(clk)
    quote
        local channellock = $(esc(clk))
        lock(channellock.lock)
        try
            if length(channellock.channel.data) > 0
                fetch(channellock.channel)
            else
                continue
            end
        finally
            unlock(channellock.lock)
        end
    end
end

macro fetch_or_return(clk)
    quote
        local channellock = $(esc(clk))
        lock(channellock.lock)
        try
            if length(channellock.channel.data) > 0
                fetch(channellock.channel)
            else
                return
            end
        finally
            unlock(channellock.lock)
        end
    end
end

macro return_if_told(clk)
    quote
        local channellock = $(esc(clk))
        lock(channellock.lock)
        try
            if length(channellock.channel.data) > 0
                return
            end
        finally
            unlock(channellock.lock)
        end
    end
end
    

        
    

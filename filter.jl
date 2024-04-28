using Plots

Ts = 0.1
signal = 0.0
old_signal = 0.0
delayed_signal = 0.0
biggest_change = 0.0
filtered_signal = 0.0

test_signals = []
delayed_signals = []
filtered_signals = []
filtered_signals2 = []
velocities = []

a = 0.01
velocities = zeros(Int(1/a))
# Delay in number of timesteps = 1/a
delay = 1/a*Ts

function filter(signal, velocity, t)
    global old_signal, delayed_signal, a
    delayed_signal = delayed_signal*(1-a) + signal*a
    println("Velocity: ", velocity)
    
    pushfirst!(velocities, velocity)
    pop!(velocities)

    position = delayed_signal
    for velocity in velocities
        position = position + velocity*Ts
    end
    # for i in 1:length(velocities)-1
    #     position = position + (velocities[i]+velocities[i+1])*Ts/2
    # end
    println("Length of velocities: ", length(velocities))
    filtered_signal = position


    old_signal = signal
    push!(delayed_signals, delayed_signal)
    push!(filtered_signals, filtered_signal)
    # push!(velocities, velocity)
    return filtered_signal
end


function filter_two(signal, t)
    global velocity, old_signal, delayed_signal, a, biggest_change, filtered_signal
    change = abs(signal-old_signal)
    if change > biggest_change
        biggest_change = change
    end
    weight = change/biggest_change / 100
    println("Biggest change: ", biggest_change)
    filtered_signal = filtered_signal + weight*(signal - filtered_signal)
    push!(filtered_signals2, filtered_signal)
    old_signal = signal
    return filtered_signal
end



function test_filter()
    global Ts
    test_velocity = 1.0
    test_signal = 0.0
    for t in 1:1000
        if t>100 && t<200
            test_velocity += 0.1
        elseif t>200 && t<300
            test_velocity -= 0.2
        elseif t>500
            test_velocity += 0.05
        end

        test_signal = test_signal + test_velocity*Ts
        noisy_test_signal = test_signal + rand()*20

        push!(test_signals, noisy_test_signal)
        filter_two(noisy_test_signal, t)
        filter(noisy_test_signal, test_velocity, t)
    end
end


test_filter()

plot()
plot!(delayed_signals[490:510], label="Delayed Signal")
plot!(test_signals[490:510], label="Test Signal")
plot!(filtered_signals[490:510], label="Filtered Signal")
plot!(filtered_signals2[490:510], label="Filtered Signal")
# plot!(velocities[4500:5500], label="Velocity")

plot(test_signals, label="Test Signal")
plot!(delayed_signals, label="Delayed Signal")
plot!(filtered_signals, label="Filtered Signal")
plot!(filtered_signals2, label="Filtered2 Signal")
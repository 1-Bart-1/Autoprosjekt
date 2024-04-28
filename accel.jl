speed = 0.0
lastSpeed = 0.0
lastOutput = 0.0
ValveSpeed = 100.0/30.0
svingTid = 0.5
ValveAccel = (ValveSpeed * 2.0) / svingTid
Ts = 0.1
function test_accel(output)
    global speed, lastOutput, ValveSpeed, ValveAccel, Ts, lastSpeed
    # output = clamp(output, 0.0, 100.0)
    speed = (output - lastOutput) / Ts

    if speed > ValveSpeed
        speed = ValveSpeed
        println("speed is valvespeed ", speed)
    elseif speed < -ValveSpeed
        speed = -ValveSpeed
    end
    output = lastOutput + speed * Ts

    accel = (speed - lastSpeed) / Ts

    if accel > ValveAccel
        println("accel ", accel)
        speed = lastSpeed + ValveAccel * Ts
        println("speed is lastspeed + accel ", speed)
    elseif accel < -ValveAccel
        speed = lastSpeed - ValveAccel * Ts
    end
    output = lastOutput + speed * Ts

    lastOutput = output
    lastSpeed = speed
    println("lastSpeed ", lastSpeed)
    println("output ", output)
    output = clamp(output, 0.0, 100.0)
    return(output)
end
using Plots
results = Float64[]  # Initialize an empty array to store the results
for i in 1:10
    push!(results, test_accel(100.))  # Store the result of each iteration
end

for i in 1:20
    push!(results, test_accel(-100.))  # Store the result of each iteration
end

for i in 1:10
    push!(results, test_accel(100.))  # Store the result of each iteration
end

plot(results, title="Second order LPF - a = 2.0", xlabel="Time", ylabel="Output")
using Plots
using DiscretePIDs, ControlSystemsBase, OrdinaryDiffEq, DiffEqCallbacks
using Optim

function calc_outflow_line(p)
    outflow_rate_75 =  (p.tank_height/2) / p.empty_upper_halve_time
    outflow_rate_25 = (p.tank_height/2) / p.empty_lower_halve_time 
    a = (outflow_rate_75 - outflow_rate_25) / (0.75 * p.tank_height - 0.25 * p.tank_height)
    b = outflow_rate_25 - a * 0.25 * p.tank_height
    line = (a=a, b=b)
    return line
end

function calc_outflow_rate(line, water_level)
    outflow_rate = line.a * water_level + line.b
    return outflow_rate
end

# Define the water reservoir ODE function
function water_reservoir_ode(du, u, p, t)
    # inflow_rate, max_outflow_rate, opening_speed, desired_water_level = p
    water_level, valve_position, delta_valve_position, outflow_rate = u
    
    u[4] = outflow_rate = calc_outflow_rate(p.line, u[1])
    
    max_inflow_rate = p.tank_height/p.filling_time
    valve_position = clamp(valve_position, 0.0, 1.0)
    inflow_rate = max_inflow_rate * valve_position
    
    disturbance = 0
    if p.Tf/3 < t < 2*p.Tf/3
        disturbance = p.tank_height / p.valve_emptying_time
    elseif 2*p.Tf/3 < t < p.Tf
        disturbance = 2 * p.tank_height / p.valve_emptying_time
    end
    # disturbance += rand() * 0.0001
    du[1] = delta_water_level = inflow_rate - outflow_rate - disturbance # Water level equation
    # du[2] = delta_valve_position = 0.0 # Placeholder for valve position update, will be updated by callback
    du[2] = delta_valve_position = delta_valve_position
    du[3] = accel_valve_position = 0.0
end

function pid_callback(integrator)
    u = integrator.u
    p = integrator.p
    water_level = u[1] # Current water level
    delta_valve_position = clamp(p.pid(p.desired_water_level, water_level), -p.opening_speed, p.opening_speed) # Update valve position using PID controller
    u[3] = delta_valve_position # Update valve position in the integrator's state
end

function simulate(pid_params)
    K, Ti, Td = max.(pid_params, [0.,0.,0.])

    # Define the PID controller
    Tf = 200.0   # Simulation time
    Ts = 0.1     # Sample time
    
    initial_conditions = [0.0, 0.0, 0.0, 0.0] # Initial water level and valve position
    p = (
        tank_height = 0.63,
        valve_emptying_time = 100, # amount of time to empty the tank with one open valve
        filling_time = 20, # one minute to fully fill the tank with open valve and max inflow rate
        empty_upper_halve_time = 50, # 15 seconds to empty half of the tank
        empty_lower_halve_time = 55,
        opening_speed = 1/15, # 1 / amount of seconds to open the valve completely
        desired_water_level = 0.315, # desired height
        pid = DiscretePID(; K, Ts, Ti, Td),
        Tf = Tf,
    )

    line = calc_outflow_line(p)
    p = (p..., line=line)

    cb = PeriodicCallback(pid_callback, Ts)
    tspan = (0.0, Tf)
    prob = ODEProblem(water_reservoir_ode, initial_conditions, tspan, p, callback=cb)

    # Solve the problem
    sol = solve(prob, Tsit5(), saveat=Ts)
    
    max_level = maximum([u[1] for u in sol.u])
    time_reached_level = Tf*2
    for (t, u) in zip(sol.t, sol.u)
        if u[1] >= p.desired_water_level*0.99
            time_reached_level = t
            break
        end
    end
    # println(time_reached_level, "\t", abs(max_level - p.desired_water_level), "\t", pid_params)
    # if max_level > p.desired_water_level * 1.01
    #     cost = time_reached_level * 1000
    # else
    #     cost = time_reached_level
    # end
    cost = max(abs(max_level - p.desired_water_level), 0.10*p.desired_water_level) * time_reached_level
    println(time_reached_level)
    display(plot(sol, title="Water Reservoir with Valve", xlabel="Time", ylabel="Water Level"))

    # p = plot(sol.t, [u ./ maximum(sol.u, dims=1) for u in sol.u], title="Water Reservoir with variable outflow rate", xlabel="Time", ylabel="Max outflow rate")
    # savefig(p, "Valve simulation.png")
    return cost
end

# simulate([100.15373308782321, 581.0078385651256, 21.137621880761632])

lower = [0, 0, 0]
upper = [Inf, Inf, Inf]
initial_pid_params = [150.54558899817698, 523.2017234966673, 8.77637495098878]

results = optimize(simulate, lower, upper, initial_pid_params, NelderMead(), Optim.Options(time_limit=10.0))
println(summary(results))
println(Optim.minimizer(results))
println(Optim.minimum(results))
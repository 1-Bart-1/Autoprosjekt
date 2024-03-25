using Plots
using DiscretePIDs, ControlSystemsBase, OrdinaryDiffEq, DiffEqCallbacks
using Optim

function calc_max_outflow_rate(p, u)
    outflow_rate_75 = p.empty_upper_halve_time / (p.tank_height/2)
    outflow_rate_25 = p.empty_lower_halve_time / (p.tank_height/2)
    a = (outflow_rate_75 - outflow_rate_25) / (75 - 25)
    b = outflow_rate_25 - a * 25
    outflow_rate = a * u[1] + b
    return outflow_rate
end

# Define the water reservoir ODE function
function water_reservoir_ode(du, u, p, t)
    # inflow_rate, max_outflow_rate, opening_speed, desired_water_level = p
    water_level, valve_position, delta_valve_position, max_outflow_rate = u
    
    inflow_rate = p.tank_height/p.filling_time
    u[4] = max_outflow_rate = calc_max_outflow_rate(p, u)

    du[1] = delta_water_level = inflow_rate - valve_position * max_outflow_rate # Water level equation
    # du[2] = delta_valve_position = 0.0 # Placeholder for valve position update, will be updated by callback
    du[2] = delta_valve_position = delta_valve_position
    du[3] = accel_valve_position = 0.0
end

function pid_callback(integrator)
    u = integrator.u
    p = integrator.p
    water_level = u[1] # Current water level
    delta_valve_position = -clamp(p.pid(p.desired_water_level, water_level), -p.opening_speed, p.opening_speed) # Update valve position using PID controller
    u[3] = delta_valve_position # Update valve position in the integrator's state
end

function simulate(pid_params)
    K, Ti, Td = pid_params
    # Define the PID controller
    Tf = 200.0   # Simulation time
    Ts = 0.1     # Sample time
    
    initial_conditions = [0.0, 0.0, 0.0, 0.0] # Initial water level and valve position
    p = (
        tank_height = 100,
        filling_time = 60, # one minute to fully fill the tank
        empty_upper_halve_time = 20, # 15 seconds to empty half of the tank
        empty_lower_halve_time = 18,
        opening_speed = 1/7.5, # 1 / amount of seconds to open the valve completely
        desired_water_level = 50.0,
        pid = DiscretePID(; K, Ts, Ti, Td),
    )

    cb = PeriodicCallback(pid_callback, Ts)
    tspan = (0.0, Tf)
    prob = ODEProblem(water_reservoir_ode, initial_conditions, tspan, p, callback=cb)

    # Solve the problem
    sol = solve(prob, Tsit5(), saveat=Ts)
    
    max_level = maximum([u[1] for u in sol.u])
    time_reached_level = Inf
    for (t, u) in zip(sol.t, sol.u)
        if u[1] >= p.desired_water_level*0.99
            time_reached_level = t
            break
        end
    end
    # println(time_reached_level)
    # println(abs(max_level - p.desired_water_level) * 100)
    # if max_level > p.desired_water_level * 1.01
    #     cost = time_reached_level * 1000
    # else
    #     cost = time_reached_level
    # end
    cost = abs(max_level - p.desired_water_level) * time_reached_level
    # println(cost, pid_params)

    display(plot(sol, title="Water Reservoir with Valve", xlabel="Time", ylabel="Water Level"))
    # p = plot(sol.t, [u ./ maximum(sol.u, dims=1) for u in sol.u], title="Water Reservoir with variable outflow rate", xlabel="Time", ylabel="Max outflow rate")
    # savefig(p, "Valve simulation.png")
    return cost
end

# simulate([100.15373308782321, 581.0078385651256, 21.137621880761632])
lower = [0, 0, 0]
upper = [Inf, Inf, Inf]
initial_pid_params = [1.5, 2.0, 3.0]

results = optimize(simulate, lower, upper, initial_pid_params, NelderMead(), Optim.Options(time_limit=10.0))
println(summary(results))
println(Optim.minimizer(results))


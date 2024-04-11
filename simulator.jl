using Plots
using DiscretePIDs, ControlSystemsBase, OrdinaryDiffEq, DiffEqCallbacks
using Optim
using Polynomials

include("process_data.jl")

desired_water_level = 0.315
control_method = "position"

# Define the water reservoir ODE function
function water_reservoir_ode(du, u, p, t)
    # inflow_rate, max_outflow_rate, opening_speed, desired_water_level = p
    water_level, valve_position, delta_valve_position, disturbance_level, inflow_rate, wanted_valve_position = u
        
    inflow_rate = max(p.valve_fill_rate(valve_position), 0.0)
    
    disturbance = 0.0
    if 0 < t < p.Tf/4
        disturbance = disturbance_to_rate(p.disturbance2, u[1])
    elseif p.Tf/4 < t < 2*p.Tf/4
        disturbance = disturbance_to_rate(p.disturbance3, u[1])
    elseif 2*p.Tf/4 < t < 3*p.Tf/4
        disturbance = disturbance_to_rate(p.disturbance1, u[1])
    elseif 3*p.Tf/4 < t < p.Tf
        disturbance = disturbance_to_rate(p.bigdisturbance, u[1])
    end

    # disturbance += rand() * 0.0001
    du[1] = delta_water_level = inflow_rate + disturbance # Water level equation
    # du[2] = delta_valve_position = 0.0 # Placeholder for valve position update, will be updated by callback

    if p.control_method == "speed"
        du[2] = delta_valve_position = delta_valve_position
    elseif p.control_method == "position"
        if wanted_valve_position < valve_position
            du[2] = -p.opening_speed
        elseif wanted_valve_position > valve_position
            du[2] = p.opening_speed
        else
            du[2] = 0.0
        end
    end

    du[3] = accel_valve_position = 0.0
    u[4] = disturbance
    u[5] = inflow_rate
    u[2] = clamp(valve_position, 0.0, 1.0)
    du[6] = 0.0
end

function pid_callback(integrator)
    u = integrator.u
    p = integrator.p
    water_level = u[1] # Current water level
    if p.control_method == "speed"
        delta_valve_position = clamp(p.pid(p.desired_water_level, water_level), -p.opening_speed, p.opening_speed)
        u[3] = delta_valve_position # Update valve speed in the integrator's state
        return u[3]
    else p.control_method == "position"
        wanted_valve_position = clamp(p.pid(p.desired_water_level, water_level), 0, 1) # Update valve position using PID controller
        u[6] = wanted_valve_position # Update valve position in the integrator's state
        return u[6]
    end
end

function simulate(pid_params)
    K, Ti, Td = max.(pid_params, [0.,0.,0.])

    # Define the PID controller
    Tf = 200.0   # Simulation time
    Ts = 0.1     # Sample time
    
    disturbance1, disturbance2, disturbance3, bigdisturbance, frequency_fill_rate, valve_fill_rate = get_polynomials()

    initial_conditions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Initial water level and valve position
    p = (
        tank_height = 0.63,
        disturbance1 = disturbance1,
        disturbance2 = disturbance2,
        disturbance3 = disturbance3,
        bigdisturbance = bigdisturbance,
        frequency_fill_rate = frequency_fill_rate,
        valve_fill_rate = valve_fill_rate,
        opening_speed = 1/15, # 1 / amount of seconds to open the valve completely
        desired_water_level = desired_water_level, # desired height
        control_method = control_method, # Control position or speed
        pid = DiscretePID(; K, Ts, Ti, Td),
        Tf = Tf,
    )

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
    
    cost = max(abs(max_level - p.desired_water_level), 0.0*p.desired_water_level) * time_reached_level
    # println(time_reached_level)
    # println(pid_params, " -> ", cost)

    # maxes = [maximum(abs.([u[i] for u in sol.u])) for i in 1:length(sol.u[1])]
    # plot_u = [[u[i] / maxes[i] for u in sol.u] for i in 1:length(sol.u[1])]
    # p = plot(sol.t, plot_u, title="Water Reservoir with variable outflow rate", xlabel="Time", ylabel="Water level")

    p = plot(sol, title="Water Reservoir", xlabel="Time", ylabel="Water level")
    display(p)
    # savefig(p, "Realistic sim - position method.png")
    return cost
end

# simulate([1.15373308782321, 581.0078385651256, 21.137621880761632])

lower = [0, 0, 0]
upper = [Inf, Inf, Inf]
initial_pid_params = [5.050970853758569, 60.74808584643459, 1.0497595666451431]

# i = 100
# minimum_cost = 1000000.0
# while minimum_cost > 20.0
#     local initial_pid_params = [float(i), 0.0, 0.0]
#     local results = optimize(simulate, lower, upper, initial_pid_params, NelderMead(), Optim.Options(time_limit=10.0))
#     global minimum_cost = Optim.minimum(results)
#     println(summary(results))
#     println(Optim.iterations(results))
#     println(Optim.minimizer(results))
#     println(Optim.minimum(results))
#     global i += 10
# end

results = optimize(simulate, lower, upper, initial_pid_params, NelderMead(), Optim.Options(time_limit=50.0))
println(summary(results))
println(Optim.minimizer(results))
println(Optim.minimum(results))
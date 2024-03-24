using Plots
using DiscretePIDs, ControlSystemsBase, OrdinaryDiffEq, DiffEqCallbacks
using Optim

# Define the water reservoir ODE function
function water_reservoir_ode(du, u, p, t)
    inflow_rate, max_outflow_rate, opening_speed, desired_water_level = p
    water_level, valve_position, delta_valve_position = u
    
    du[1] = delta_water_level = inflow_rate - valve_position * max_outflow_rate # Water level equation
    # du[2] = delta_valve_position = 0.0 # Placeholder for valve position update, will be updated by callback
    du[2] = delta_valve_position = delta_valve_position
    du[3] = accel_valve_position = 0.0
end

function pid_callback(integrator)
    pid = integrator.p[5]
    desired_water_level = integrator.p[4] # Reference value
    water_level = integrator.u[1] # Current water level
    delta_valve_position = -clamp(pid(desired_water_level, water_level), -integrator.p[3], integrator.p[3]) # Update valve position using PID controller
    integrator.u[3] = delta_valve_position # Update valve position in the integrator's state
end

function simulate(pid_params)
    K, Ti, Td = pid_params
    # Define the PID controller
    Tf = 200.0   # Simulation time
    Ts = 0.1     # Sample time
    pid = DiscretePID(; K, Ts, Ti, Td)
    
    # Initial conditions
    initial_conditions = [0.0, 0.0, 0.0] # Initial water level and valve position
    
    # Parameters
    parameters = [0.1, 0.2, 1.0/7.5, 10.0, pid] # Inflow rate, max outflow rate, opening speed, desired_water_level, pid
    
    # Time span for the simulation
    tspan = (0.0, Tf)
    
    # Define the callback function to update the valve position using the PID controller
    # cb = PeriodicCallback(Ts) do integrator
    #     desired_water_level = integrator.p[4] # Reference value
    #     water_level = integrator.u[1] # Current water level
    #     delta_valve_position = -clamp(pid(desired_water_level, water_level), -integrator.p[3], integrator.p[3]) # Update valve position using PID controller
    #     integrator.u[3] = delta_valve_position # Update valve position in the integrator's state
    # end

    cb = PeriodicCallback(pid_callback, Ts)

    # Create the ODE problem
    prob = ODEProblem(water_reservoir_ode, initial_conditions, tspan, parameters, callback=cb)

    # Solve the problem
    sol = solve(prob, Tsit5(), saveat=Ts)
    
    max_level = maximum([u[1] for u in sol.u])
    time_reached_level = Tf
    for (t, u) in zip(sol.t, sol.u)
        if u[1] >= parameters[4]*0.99
            time_reached_level = t
            break
        end
    end
    # println(time_reached_level)
    # println(abs(max_level - parameters[4]) * 100)
    cost = abs(max_level - parameters[4]) * 100 + time_reached_level
    # # Plot the solution
    display(plot(sol, title="Water Reservoir with Valve", xlabel="Time", ylabel="Water Level"))
    return cost
end

simulate([1.2726757601615177e8, 2.4704983054502317e8, 9.646333666824853])
# lower = [0, 0, 0]
# upper = [Inf, Inf, Inf]
# initial_pid_params = [1.2726757601615177e8, 2.4704983054502317e8, 9.646333666824853]

# results = optimize(simulate, lower, upper, initial_pid_params)
# println(summary(results))
# println(Optim.minimizer(results))


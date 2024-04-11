using Plots
using DiscretePIDs, ControlSystemsBase, OrdinaryDiffEq, DiffEqCallbacks
using Optim
using Polynomials

include("process_data.jl")

disturbance1, disturbance2, disturbance3, bigdisturbance, frequency_fill_rate, valve_fill_rate = get_polynomials()

# Define the water reservoir ODE function
function water_reservoir_ode(du, u, p, t)
    # inflow_rate, max_outflow_rate, opening_speed, desired_water_level = p
    water_level, valve_position, delta_valve_position, disturbance_level, inflow_rate, wanted_valve_position = u
        
    inflow_rate = max(p.valve_fill_rate(valve_position), 0.0)
    
    if p.test_type == "no-disturbance"
        disturbance = 0.0
    elseif p.test_type == "disturbance1"
        disturbance = disturbance_to_rate(p.disturbance1, u[1])
    elseif p.test_type == "disturbance2"
        disturbance = disturbance_to_rate(p.disturbance2, u[1])
    elseif p.test_type == "disturbance3"
        disturbance = disturbance_to_rate(p.disturbance3, u[1])
    elseif p.test_type == "big-disturbance"
        disturbance = disturbance_to_rate(p.bigdisturbance, u[1])
    else
        throw(ArgumentError("Invalid test type"))
    end

    # disturbance += rand() * 0.0001
    du[1] = delta_water_level = inflow_rate + disturbance # Water level equation
    # du[2] = delta_valve_position = 0.0 # Placeholder for valve position update, will be updated by callback

    if p.control_method == "speed" || p.control_method == "none"
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
    u[1] = clamp(water_level, 0.0, p.tank_height)
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
    elseif p.control_method == "position"
        wanted_valve_position = clamp(p.pid(p.desired_water_level, water_level), 0, 1) # Update valve position using PID controller
        u[6] = wanted_valve_position # Update valve position in the integrator's state
        return u[6]
    elseif p.control_method == "none"
        u[3] = 0.0
        return u[3]
    end
end

function simulate(pid_params, Tf, Ts, desired_water_level, control_method, test_type)
    K, Ti, Td = max.(pid_params, [0.,0.,0.])

    initial_conditions = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0] # Initial water level and valve position
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
        test_type = test_type,
        pid = DiscretePID(; K, Ts, Ti, Td),
        Tf = Tf,
    )

    cb = PeriodicCallback(pid_callback, Ts)
    tspan = (0.0, Tf)
    prob = ODEProblem(water_reservoir_ode, initial_conditions, tspan, p, callback=cb)

    # Solve the problem
    sol = solve(prob, Tsit5(), saveat=Ts)
    return sol, p
end



using Plots
using ControlSystemsBase, DelayDiffEq, DiffEqCallbacks
using Optim
using Polynomials

include("process_data.jl")
include("pid.jl")

using .PID

disturbance1, disturbance2, disturbance3, bigdisturbance, frequency_fill_rate, valve_fill_rate, frequency_to_rate, valve_to_rate = get_polynomials()

alg = MethodOfSteps(Tsit5())
total_time = 0.0
total_solves = 0

# Define the water reservoir ODE function
function water_reservoir_ode(du, u, h, p, t)
    # inflow_rate, max_outflow_rate, opening_speed, desired_water_level = p
    water_level, valve_position, disturbance_rate, inflow_rate, wanted_valve_position, delayed_valve_position = u
    
    delayed_valve_position = h(p, t-p.delay)[2]
    if p.control_method == "valve"
        inflow_rate = max(p.valve_fill_rate(delayed_valve_position), 0.0)
    elseif p.control_method == "frequency"
        inflow_rate = max(p.frequency_fill_rate(delayed_valve_position), 0.0)
    else
        inflow_rate = 0.0
    end
    
    if p.test_type == "no-disturbance"
        disturbance_rate = 0.0
    elseif p.test_type == "disturbance1"
        disturbance_rate = disturbance_to_rate(p.disturbance1, u[1])
    elseif p.test_type == "disturbance2"
        disturbance_rate = disturbance_to_rate(p.disturbance2, u[1])
    elseif p.test_type == "disturbance3"
        disturbance_rate = disturbance_to_rate(p.disturbance3, u[1])
    elseif p.test_type == "big-disturbance"
        disturbance_rate = disturbance_to_rate(p.bigdisturbance, u[1])
    else
        throw(ArgumentError("Invalid test type"))
    end
    
    # disturbance += rand() * 0.0001
    du[1] = delta_water_level = inflow_rate + disturbance_rate # Water level equation
    # du[2] = delta_valve_position = 0.0 # Placeholder for valve position update, will be updated by callback
    
    if p.control_method != "none"
        if wanted_valve_position < valve_position
            du[2] = delta_valve_position = -p.opening_speed
        elseif wanted_valve_position > valve_position
            du[2] = delta_valve_position = p.opening_speed
        else
            du[2] = 0.0
        end
    end
    u[6] = delayed_valve_position
    
    u[3] = disturbance_rate
    u[4] = inflow_rate
    u[2] = clamp(valve_position, 0.0, 1.0)
    u[1] = clamp(water_level, 0.0, p.tank_height)
end

function pid_callback(integrator)
    u = integrator.u
    p = integrator.p
    water_level = u[1] # Current water level
    if p.control_method == "none"
        return nothing
    elseif p.control_method == "valve"
        wanted_valve_position = PID.pid(p.desired_water_level, water_level) # Update valve position using PID controller
        # println(wanted_valve_position)
        u[5] = wanted_valve_position / 4082 # Update valve position in the integrator's state
        return u[5]
    elseif p. control_method == "frequency"
        wanted_valve_position = PID.pid(p.desired_water_level, water_level) # Update valve position using PID controller
        u[5] = wanted_valve_position / 16384 * 50 # Update valve position in the integrator's state
        return u[5]
    end
end

function simulate(pid_params, Tf, Ts, desired_water_level, control_method, pid_method, test_type, delay)
    global total_time, total_solves

    # while length(pid_params) < 3
    #     push!(params, 0.0)
    # end
    # K, Ti, Td = max.(pid_params, [0.,0.,0.])
    
    PID.reset()
    PID.set_parameters(pid_params, pid_method)
    
    if test_type == "no-disturbance"
        start_water_level = 0.0
    else
        start_water_level = desired_water_level
    end
    
    initial_conditions = [start_water_level, 0.0, 0.0, 0.0, 0.0, 0.0] # Initial water level and valve position
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
        delay = delay,
        Tf = Tf,
        )

    h(p, t) = initial_conditions
    cb = PeriodicCallback(pid_callback, Ts)

    tspan = (0.0, Tf)
    # prob = ODEProblem(water_reservoir_ode, initial_conditions, tspan, p, callback=cb)
    prob = DDEProblem(water_reservoir_ode, initial_conditions, h, tspan, p, callback=cb)

    # Solve the problem
    total_time += @elapsed sol = solve(prob, alg, saveat=Ts)
    total_solves += 1
    return sol, p
end

function calc_avg_time()
    global total_time, total_solves
    return total_time / total_solves
end
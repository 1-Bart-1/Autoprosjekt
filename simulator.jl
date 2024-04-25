
using Plots
using ControlSystemsBase, DelayDiffEq, DiffEqCallbacks
using Optim
using Polynomials

include("process_data.jl")
include("pid.jl")

disturbance1, disturbance2, disturbance3, bigdisturbance, frequency_fill_rate, valve_fill_rate, rate_to_frequency, rate_to_valve = get_polynomials()

alg = MethodOfSteps(Rosenbrock23(autodiff=false))
total_time = 0.0
total_solves = 0

# Define the water reservoir ODE function
function water_reservoir_ode(du, u, h, p, t)
    water_level, valve_position, disturbance_rate, inflow_rate, wanted_valve_position, delayed_valve_position, delayed_water_level = u
    
    delayed_water_level = h(p, t-0.25)[1]
    delayed_valve_position = h(p, t-p.delay)[2]
    if p.control_method == "valve"
        inflow_rate = max(p.valve_fill_rate(delayed_valve_position), 0.0)
    elseif p.control_method == "frequency"
        inflow_rate = max(p.frequency_fill_rate(delayed_valve_position), 0.0)
    else
        inflow_rate = 0.0
    end
    
    if p.test_type == "nominal" || p.test_type == "top"
        disturbance_rate = disturbance_to_rate(p.disturbance2, u[1])
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
    du[5] = 0.0 # Placeholder for valve position update, will be updated by callback
    
    if p.control_method != "none"
        if wanted_valve_position < valve_position
            if p.control_method == "valve"
                du[2] = delta_valve_position = -p.valve_speed
            elseif p.control_method == "frequency"
                du[2] = delta_valve_position = -p.frequency_speed
            end
        elseif wanted_valve_position > valve_position
            if p.control_method == "valve"
                du[2] = delta_valve_position = p.valve_speed
            elseif p.control_method == "frequency"
                du[2] = delta_valve_position = p.frequency_speed
            end
        else
            du[2] = 0.0
        end
    end
    u[6] = delayed_valve_position
    u[7] = delayed_water_level
    
    u[3] = disturbance_rate
    u[4] = inflow_rate
    if p.control_method == "frequency"
        u[2] = clamp(valve_position, 0.0, 50.0)
    elseif p.control_method == "valve"
        u[2] = clamp(valve_position, 0.0, 1.0)
    end
    u[1] = clamp(water_level, 0.0, p.tank_height)
end

function pid_callback(integrator)
    u = integrator.u
    p = integrator.p
    delayed_water_level = u[7] # Use the delayed water level
    outflow_rate = -u[3]
    # println("outflow rate: ", outflow_rate)
    # println("time: ", integrator.t)
    if p.control_method == "none"
        return nothing
    elseif p.control_method == "valve"
        wanted_valve_position = pid(
            p.pid, 
            Float32(p.desired_water_level/p.tank_height*100.0), 
            Float32(delayed_water_level/p.tank_height*100.0 + rand()*0.3), 
            Float32(outflow_rate)
        )
        # println(wanted_valve_position)
        u[5] = wanted_valve_position / 100. # Update valve position in the integrator's state
        return u[5]
    elseif p.control_method == "frequency"
        wanted_valve_position = pid(
            p.pid, 
            Float32(p.desired_water_level/p.tank_height*100.0), 
            Float32(delayed_water_level/p.tank_height*100.0 + rand()*0.3), 
            Float32(outflow_rate)
        )
        u[5] = wanted_valve_position / 100. * 50. # Update frequency in the integrator's state
        return u[5]
    end
end

function simulate(pid_params, Tf, Ts, desired_water_level, control_method, pid_method, test_type, delay)
    global total_time, total_solves
    # while length(pid_params) < 3
    #     push!(params, 0.0)
    # end
    # K, Ti, Td = max.(pid_params, [0.,0.,0.])
    pid_params = convert(Vector{Float32}, pid_params)
    pid = PIDState()
    set_parameters!(pid, pid_params, pid_method, Float32(Ts))
    change_control_mode!(pid, control_method == "frequency")
    
    if test_type == "nominal"
        start_water_level = 0.0
    elseif test_type == "top"
        start_water_level = 0.63
    else
        start_water_level = desired_water_level
    end
    
    initial_conditions = [start_water_level, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Initial water level and valve position

    p = (
        tank_height = 0.63,
        disturbance1 = disturbance1,
        disturbance2 = disturbance2,
        disturbance3 = disturbance3,
        bigdisturbance = bigdisturbance,
        frequency_fill_rate = frequency_fill_rate,
        valve_fill_rate = valve_fill_rate,
        valve_speed = 1/30, # 1 / amount of seconds to open the valve completely
        frequency_speed = 50/3.25, # 50 Hz / amount of seconds to open the pump completely
        desired_water_level = desired_water_level, # desired height
        control_method = control_method, # Control frequency or valve
        test_type = test_type,
        delay = delay,
        pid = pid,
        Tf = Tf,
        )

    h(p, t) = initial_conditions
    cb = PeriodicCallback(pid_callback, Ts)

    tspan = (0.0, Tf)
    # prob = ODEProblem(water_reservoir_ode, initial_conditions, tspan, p, callback=cb)
    prob = DDEProblem(water_reservoir_ode, initial_conditions, h, tspan, p, callback=cb)

    # Solve the problem
    total_time += @elapsed sol = solve(prob, alg, saveat=Ts)
    # println("length of sol: ", length(sol.t))
    total_solves += 1
    return sol, p
end

function calc_avg_time()
    global total_time, total_solves
    return total_time / total_solves
end
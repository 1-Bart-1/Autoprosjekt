using DifferentialEquations
using Plots

integral = 0.0
derivative = 0.0
previous_error = 0.0
delta_valve_position = 0.0

function pid_controller(error, t)
    global integral, derivative, previous_error
    Kp = 0.1 # Proportional gain
    Ki = 0.0 # Integral gain
    Kd = 0.0 # Derivative gain
    delta_t = 0.1
    integral += error * delta_t # Update integral
    derivative = (error - previous_error) / delta_t # Update derivative
    previous_error = error # Store current error for next iteration
    return Kp * error + Ki * integral + Kd * derivative
end

function water_reservoir_ode(du, u, p, t)
    global delta_valve_position
    inflow_rate, max_outflow_rate, opening_speed, desired_water_level = p
    water_level, valve_position = u
    
    error = water_level - desired_water_level
    delta_valve_position = clamp(pid_controller(error, t), -opening_speed, opening_speed)

    # if t < 5.0 && valve_position < 1.0
    #     # Opening phase
    #     delta_valve_position = opening_speed
    # elseif t >= 5.0 && valve_position > 0.0
    #     # Closing phase
    #     delta_valve_position = -opening_speed
    # else
    #     delta_valve_position = 0.0
    # end

    # # Calculate outflow rate based on valve position
    # outflow_rate = valve_position * max_outflow_rate

    # Update du based on the calculated values
    du[1] = delta_water_level = inflow_rate - valve_position * max_outflow_rate # Water level equation
    du[2] = delta_valve_position # Update valve position directly in du[2]
end

# Initial conditions
initial_conditions = [5.0, 0.0] # Initial water level and valve position

# Parameters
parameters = [1.0, 2.0, 1.0/2.5, 10.0] # Inflow rate, max outflow rate, opening speed, desired_water_level

# Time span for the simulation
tspan = (0.0, 100.0)

# Create the ODE problem
prob = ODEProblem(water_reservoir_ode, initial_conditions, tspan, parameters)

# Solve the problem
sol = solve(prob, ABM54(), adaptive=false, dt=0.01)

# Plot the solution
plot(sol, title="Water Reservoir with Valve", xlabel="Time", ylabel="Water Level")
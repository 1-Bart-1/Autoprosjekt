using DiscretePIDs, ControlSystemsBase, OrdinaryDiffEq, DiffEqCallbacks, Plots

Tf = 15   # Simulation time
K  = 1    # Proportional gain
Ti = 1    # Integral time
Td = 1    # Derivative time
Ts = 0.01 # sample time

P = ss(tf(1, [1, 1]))  # Process to be controlled in continuous time
A, B, C, D = ssdata(P) # Extract the system matrices
pid = DiscretePID(; K, Ts, Ti, Td)

function dynamics!(dxu, xu, p, t)
    A, B, C, r, d = p   # We store the reference and disturbance in the parameter object
    x = xu[1:P.nx]      # Extract the state
    u = xu[P.nx+1:end]  # Extract the control signal
    dxu[1:P.nx] .= A*x .+ B*(u .+ d) # Plant input is control signal + disturbance
    dxu[P.nx+1:end] .= 0             # The control signal has no dynamics, it's updated by the callback
end

cb = PeriodicCallback(Ts) do integrator
    p = integrator.p    # Extract the parameter object from the integrator
    (; C, r, d) = p     # Extract the reference and disturbance from the parameter object
    x = integrator.u[1:P.nx] # Extract the state (the integrator uses the variable name `u` to refer to the state, in control theory we typically use the variable name `x`)
    y = (C*x)[]         # Simulated measurement
    u = pid(r, y)       # Compute the control signal
    integrator.u[P.nx+1:end] .= u # Update the control-signal state variable 
end

parameters = (; A, B, C, r=0, d=1) # reference = 0, disturbance = 1
xu0 = zeros(P.nx + P.nu) # Initial state of the system + control signals
prob = ODEProblem(dynamics!, xu0, (0, Tf), parameters, callback=cb) # reference = 0, disturbance = 1
sol = solve(prob, Tsit5(), saveat=Ts)

plot(sol, layout=(2, 1), ylabel=["x" "u"], lab="")
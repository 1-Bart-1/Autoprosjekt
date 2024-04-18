using Optim
include("simulator.jl")

Tf = 200.0
Ts = 0.1
desired_water_level = 0.315
control_methods = ["valve", "frequency", "none"]
test_types = ["disturbance1","disturbance2", "disturbance3", "nominal"]
pid_methods = ["PID", "PI", "P", "PD", "LL", "FF"]

control_method = "valve"
pid_method = "PID"
optimizing = false

# test_types = ["disturbance1"]
delay = 0.05
overswing_percentage = 0.00
deviation_percentage = 0.00
save = false
name = "Ts - 0.5s - PID.png"

initial_pid_params = []
lower = []
upper = []

if pid_method == "P"
    lower = [0.0]
    upper = [Inf]
    initial_pid_params = [37.0, 30.0]
elseif pid_method == "PI"
    lower = [0.0, 0.0]
    upper = [Inf, Inf]
    initial_pid_params = [37.78129011219762, 18.449799061858812]
elseif pid_method == "PD"
    lower = [0.0, 0.0]
    upper = [Inf, Inf]
    initial_pid_params = [37.78129011219762, 0.06200201920575654, 23.71259411841547]
elseif pid_method == "PID"
    lower = [0.0, 0.0, 0.0]
    upper = [Inf, Inf, Inf]
    initial_pid_params = [37.78129011219762, 0.06200201920575654, 5.628083016278622]
elseif pid_method == "LL"
    lower = [0.0, 0.0, 0.0]
    upper = [Inf, Inf, Inf]
    initial_pid_params = [37.78129011219762, 0.06200201920575654, 5.628083016278622]
elseif pid_method == "FF"
    lower = [0.0, 0.0, 0.0]
    upper = [Inf, Inf, Inf]
    initial_pid_params = [373.0, 10.0, 10.0]
end

function plot_sim(sol, title="Water Reservoir")
    
    # maxes = [maximum(abs.([u[i] for u in sol.u])) for i in 1:length(sol.u[1])]
    # plot_u = [[u[i] / maxes[i] for u in sol.u] for i in 1:length(sol.u[1])]
    # p = plot(sol.t, plot_u, title="Water Reservoir with variable outflow rate", xlabel="Time", ylabel="Water level")
    
    p = plot(sol.t, [u[1] for u in sol.u], title=title, xlabel="Time", ylabel="Water level")
    p = plot!(sol.t, [u[5] for u in sol.u], title=title, xlabel="Time", ylabel="Water level")
    return p
end

function objective(pid_params, print=false)
    global Tf
    cost = 0.0
    plots = []
    times = []
    println(pid_params)
    for test_type in test_types
        sol, p = simulate(pid_params, Tf, Ts, desired_water_level, control_method, pid_method, test_type, delay)
        # println(Tf)
        
        max_level = maximum([u[1] for u in sol.u])
        time_reached_level = 10000
        for (t, u) in zip(sol.t, sol.u)
            if t >= 1 && u[1] >= p.desired_water_level*0.99
                time_reached_level = t
                break
            end
        end

        push!(times, time_reached_level)
        
        overswing = max_level - p.desired_water_level
        overswing_cost = max(overswing, overswing_percentage*p.desired_water_level) / p.desired_water_level

        time_cost = time_reached_level

        stable_deviation = abs(sol.u[end][1] - p.desired_water_level)
        stable_deviation_cost = max(stable_deviation, deviation_percentage*p.desired_water_level) / p.desired_water_level

        cost += overswing_cost*10 + time_cost / 100 + stable_deviation_cost*10
        if print
            push!(plots, plot_sim(sol, test_type))
            println("Objective summary:")
            println("\t Test type: ", test_type)
            println("\t Max level: ", max_level)
            println("\t Desired level: ", p.desired_water_level)
            println("\t Overswing percentage: ", overswing/p.desired_water_level*100, "%")
            println("\t Stable deviation percentage: ", (stable_deviation)/p.desired_water_level*100, "%")
            println("\t Time to reach level: ", time_reached_level)
            println("\t Pid params: ", pid_params)
            println("\t Cost: ", cost)
        end
    end

    Tf = min(max(times...) * 2, 200)

    if print
        p = plot(plots..., layout=(length(test_types), 1), size=(800, 1600))
        display(p)
        if save
            savefig(p, name)
        end
    end
    println(cost)
    return cost
end

function optimize()
    global initial_pid_params, pid_method, lower, upper


    results = Optim.optimize(objective, lower, upper, initial_pid_params, SimulatedAnnealing(), Optim.Options(time_limit=100.0))
    println(summary(results))
    objective(Optim.minimizer(results), true)
end

if optimizing
    println("optimizing...")
    optimize()
    println("avg sol time:")
    calc_avg_time()
else
    objective(initial_pid_params, true)
end
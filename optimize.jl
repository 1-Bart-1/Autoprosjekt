using Optim
using Statistics
include("simulator.jl")

Tf = 100.0
Ts = 0.1
desired_water_level = 0.315
control_methods = ["valve", "frequency", "none"]
# available types: disturbance1, disturbance2, disturbance3, big-disturbance, no-disturbance, nominal, top
# test_types = ["disturbance1","disturbance2", "disturbance3", "no-disturbance", "nominal"]
test_types = ["nominal"]
pid_methods = ["PID", "PI", "P", "PD", "LL", "FF"]

control_method = "frequency"
pid_method = "PID"
optimizing = true
gaining = true

delay = 0.5
overswing_percentage = 0.00
deviation_percentage = 0.00
save = false
name = "data/PID-Ui-starts-at-100.png"

initial_pid_params = []
lower = []
upper = []

if pid_method == "P"
    lower = [0.0]
    upper = [Inf]
    initial_pid_params = [1.7932707800610634, 52.]
elseif pid_method == "PI"
    lower = [0.0, 0.0]
    upper = [Inf, Inf]
    initial_pid_params = [1.684339927650809, 14.739090829129387]
elseif pid_method == "PD"
    lower = [0.0, 0.0]
    upper = [Inf, Inf]
    # [2.6265510101279665, 0.3644522179166053, 53.59498590505952]
    initial_pid_params = [2.7932707800610634, 0.15638595060619714, 51.66289693699911] # optimized small delay
elseif pid_method == "PID"
    lower = [0.0, 0.0, 0.0]
    upper = [Inf, Inf, Inf]
    initial_pid_params = [1.370754931955977, 14.87793054360137, 0.26689473166867783]
elseif pid_method == "LL"
    lower = [0.0, 0.0, 0.0]
    upper = [Inf, Inf, Inf]
    initial_pid_params = [37.78129011219762, 0.06200201920575654, 5.628083016278622]
elseif pid_method == "FF"
    lower = [0.0, 0.0, 0.0]
    upper = [Inf, Inf, Inf]
    initial_pid_params = [2.7932707800610634, 0.15638595060619714] # optimized big delay and disturbance
end

function plot_sim(sol, title="Water Reservoir")

    # p = plot(sol, title=title, xlabel="Time", ylabel="Water level")
    p = plot(sol.t, [u[1] for u in sol.u], title=title, xlabel="Time", ylabel="Water level")
    p = plot!(sol.t, [u[2]/50 for u in sol.u], title=title, xlabel="Time", ylabel="Water level")
    # p = plot!(sol.t, [u[6] for u in sol.u], title=title, xlabel="Time", ylabel="Water level")
    return p
end

function objective(pid_params, print=false)
    global Tf
    cost = 0.0
    plots = []
    times = []
    # println(pid_params)
    for test_type in test_types
        sol, p = simulate(pid_params, Tf, Ts, desired_water_level, control_method, pid_method, test_type, delay)
        # println(Tf)
        
        time_reached_level = 10000.0
        overswing = 0.0
        index = 1
        if test_type == "top"
            extreme_level = minimum([u[1] for u in sol.u])
            for (i, (t, u)) in enumerate(zip(sol.t, sol.u))
                if t >= 5 && u[1] <= p.desired_water_level*1.01
                    time_reached_level = t
                    index = i
                    println(i)
                    break
                end
            end
            overswing = p.desired_water_level - extreme_level
        else
            extreme_level = maximum([u[1] for u in sol.u])
            for (i, (t, u)) in enumerate(zip(sol.t, sol.u))
                if t >= 5 && u[1] >= p.desired_water_level*0.99
                    time_reached_level = t
                    index = i
                    break
                end
            end
            overswing = extreme_level - p.desired_water_level
        end

        push!(times, time_reached_level)
        
        overswing_cost = max(overswing, overswing_percentage*p.desired_water_level) / p.desired_water_level
        
        time_cost = time_reached_level
        
        # stable_deviation = abs(sol.u[end][1] - p.desired_water_level)
        u_values_after_time = [u[1] for u in sol.u[index:end]]
        differences = abs.(u_values_after_time .- desired_water_level)
        stable_deviation = sum(differences) / length(u_values_after_time)
        stable_deviation_cost = max(stable_deviation, deviation_percentage*p.desired_water_level) / p.desired_water_level

        cost += overswing_cost*10 + time_cost * 0.001 + stable_deviation_cost*10
        # println("over ",overswing_cost*10)
        # println("time ",time_cost / 1000)
        # println("stable ",stable_deviation_cost*10)
        if print
            push!(plots, plot_sim(sol, test_type))
            println("Objective summary:")
            println("\t Test type: ", test_type)
            println("\t Control method: ", p.control_method)
            println("\t PID method: ", pid_method)
            println("\t Extreme level: ", extreme_level)
            println("\t Desired level: ", p.desired_water_level)
            println("\t Overswing percentage: ", overswing/p.desired_water_level*100, "%")
            println("\t Stable deviation percentage: ", (stable_deviation)/p.desired_water_level*100, "%")
            println("\t Time to reach level: ", time_reached_level)
            println("\t Pid params: ", pid_params)
            println("\t Cost: ", cost)
        end
    end

    # Tf = min(max(times...) * 2, 200)

    if print
        p = plot(plots..., layout=(length(test_types), 1), size=(800, 1600))
        if save
            savefig(p, name)
        else
            display(p)
        end
    end
    # println(cost)
    return cost
end

function optimize()
    global initial_pid_params, pid_method, lower, upper, desired_water_level

    if gaining
        schedules = []
        percentage = 0
        for percentage in 25:5:75
            desired_water_level = percentage/100*0.63
            # results = Optim.optimize(objective, lower, upper, initial_pid_params, SimulatedAnnealing())
            results = Optim.optimize(objective, lower, upper, initial_pid_params, SimulatedAnnealing(), Optim.Options(time_limit=60.0))
            println(summary(results))
            objective(Optim.minimizer(results), true)
            push!(schedules, [Optim.minimizer(results), percentage])
            println("Kps: ", [schedule[1][1] for schedule in schedules])
            println("Tis: ", [schedule[1][2] for schedule in schedules])
            println("Tds: ", [schedule[1][3] for schedule in schedules])
            println("Level: ", [schedule[2] for schedule in schedules])
        end
    else
        # results = Optim.optimize(objective, lower, upper, initial_pid_params, SimulatedAnnealing())
        results = Optim.optimize(objective, lower, upper, initial_pid_params, SimulatedAnnealing(), Optim.Options(time_limit=60.0*5))
        println(summary(results))
        objective(Optim.minimizer(results), true)
    end
end

if optimizing
    println("optimizing...")
    optimize()
    println("avg sol time:")
    calc_avg_time()
else
    objective(initial_pid_params, true)
end
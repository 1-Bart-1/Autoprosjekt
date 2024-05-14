using Optim
using Statistics
using ProgressBars
using YAML
# using Base.Threads

include("simulator.jl")

"""
USER INPUT
"""

control_method = "valve" # available control_methods = ["valve", "frequency", "none"]
pid_method = "PID" # available pid_methods = ["PID", "PI", "P", "PD", "LL", "FF"]

Tf = 200.0 # total simulation time
Ts = 0.1 # sample time of pid controller
delay = 0.15 # delay from output of pid controller to change in inflow rate
desired_water_level = 0.63*0.5 # desired water level in meter
test_types = ["nominal"] # available test_types = ["disturbance1", "disturbance2", "disturbance3", "big-disturbance", "no-disturbance", "nominal"]

optimizing = true
seconds = 60
gain_scheduling = true # gain scheduling is only implemented for PID
plot_all = false
optimize_t_i = false

overswing_percentage = 0.00
deviation_percentage = 0.00
save = false
name = "data/pid_sim_new_params.png"
Ui_start = Dict("frequency" => 45.0, "valve" => 28)


"""
OPTIMIZATION
"""

initial_pid_params = YAML.load_file("params.yaml")

start_time = time()
last_time = time()
# progress_bar = Progress(seconds, 1, "optimizing... ")
progress_bar = ProgressBar(total=seconds)
set_description(progress_bar, "optimizing... ")

function plot_sim(sol, title="Water Reservoir")
    title = title * " - Ui_start = $(Ui_start[control_method]) - optimized"
    # p = plot(sol, title=title, xlabel="Time", ylabel="Water level")
    p = plot(sol.t, [u[1]/0.63 for u in sol.u], title=title, xlabel="Time", ylabel="Water level", ylims=(0, 1))
    n = control_method == "frequency" ? 50 : 1
    p = plot!(sol.t, [u[2]/n for u in sol.u], title=title, xlabel="Time", ylabel="Water level", ylims=(0, 1), alpha = 1)
    p = plot!(sol.t, [u[5]/n for u in sol.u], title=title, xlabel="Time", ylabel="Water level", ylims=(0, 1), alpha = 1)
    p = plot!(sol.t, [t < Tf/2 ? 0.25 : 0.50 for t in sol.t], title=title, xlabel="Time", ylabel="Water level", ylims=(0, 1))
    return p
end

function objective(pid_params, print=false)
    global Tf, initial_pid_params, progress_bar, last_time
    cost = 0.0
    plots = []
    times = []
    if any(pid_params .< 0.0)
        return 1000
    end
    pid_params = [max(param, 0.01) for param in pid_params]
    if length(pid_params) == 2 && pid_method == "PID" && !optimize_t_i
        push!(pid_params, pid_params[2])
        pid_params[2] = initial_pid_params[control_method]["PID"][2]
    end
    for test_type in test_types
        sol, p = simulate(pid_params, Tf, Ts, desired_water_level, control_method, pid_method, test_type, delay, Ui_start[control_method])

        if sol.t[end] < Tf * 0.9
            cost += 1000
        end

        time_reached_level = 10000.0
        overswing = 0.0
        index = 1
        if test_type == "top"
            extreme_level = minimum([u[1] for u in sol.u])
            for (i, (t, u)) in enumerate(zip(sol.t, sol.u))
                if t >= 5 && u[1] <= p.desired_water_level*1.05
                    time_reached_level = t
                    index = i
                    break
                end
            end
            overswing = p.desired_water_level - extreme_level
        else
            extreme_level = maximum([u[1] for u in sol.u])
            for (i, (t, u)) in enumerate(zip(sol.t, sol.u))
                if t >= 5 && u[1] >= p.desired_water_level*0.95
                    time_reached_level = t
                    index = i
                    break
                end
            end
            overswing = extreme_level - p.desired_water_level
        end

        push!(times, time_reached_level)
        
        overswing_cost = max(overswing, overswing_percentage*p.desired_water_level) / p.desired_water_level
        
        time_reached_level = (test_type == "nominal") ? (time_reached_level - Tf/2) : time_reached_level
        time_cost = time_reached_level
        
        # stable_deviation = abs(sol.u[end][1] - p.desired_water_level)
        u_values_after_time = [u[1] for u in sol.u[index:end]]
        differences = abs.(u_values_after_time .- desired_water_level)
        stable_deviation = sum(differences) / length(u_values_after_time)
        stable_deviation_cost = max(stable_deviation, deviation_percentage*p.desired_water_level) / p.desired_water_level

        if overswing < 0.0
            overswing_cost = 100.0
        end

        cost += overswing_cost*10 + time_cost * 0.001 + stable_deviation_cost*10
        if plot_all || print
            push!(plots, plot_sim(sol, test_type))
        end
        if print
            initial_pid_params[control_method][pid_method] = pid_params

            println("objective summary:")
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
            println("\t Last state: ", sol.u[end])
        end
    end

    # Tf = min(max(times...) * 2, 200) # this can speed up the optimizing process, but is less stable
    
    if print || plot_all
        p = plot(plots..., layout=(length(test_types), 1), size=(800, length(plots)*400))
        if save
            savefig(p, name)
        else
            display(p)
        end
    end

    if !print && !plot_all
        if time() - last_time > 1.0
            update(progress_bar)
            last_time = time()
        end
    end
    
    return cost
end


function optimize()
    global initial_pid_params, pid_method, lower, upper, desired_water_level, progress_bar
    
    if gain_scheduling
        @assert pid_method == "PID"

        schedules = []
        percentage = 0

        println("\noptimizing using gain scheduling...")
        for (i, percentage) in enumerate(initial_pid_params[control_method]["gain_scheduling"]["levels"])
            progress_bar = ProgressBar(total=seconds)
            set_description(progress_bar, "optimizing round $i of $(length(initial_pid_params[control_method]["gain_scheduling"]["levels"]))... ")

            desired_water_level = percentage/100*0.63
            start_values = [initial_pid_params[control_method]["gain_scheduling"]["Kps"][i],
                            initial_pid_params[control_method]["gain_scheduling"]["Tis"][i],
                            initial_pid_params[control_method]["gain_scheduling"]["Tds"][i]]
            println("start values: ", start_values)
            if pid_method == "PID" && !optimize_t_i
                start_values = start_values[[1, 3]]
            end
            results = Optim.optimize(objective, start_values, SimulatedAnnealing(), Optim.Options(time_limit=seconds))
            objective(Optim.minimizer(results), true)
            initial_pid_params[control_method]["gain_scheduling"]["Kps"][i] = Optim.minimizer(results)[1]
            initial_pid_params[control_method]["gain_scheduling"]["Tis"][i] = optimize_t_i ? Optim.minimizer(results)[2] : initial_pid_params[control_method]["gain_scheduling"]["Tis"][i]
            initial_pid_params[control_method]["gain_scheduling"]["Tds"][i] = optimize_t_i ? Optim.minimizer(results)[3] : Optim.minimizer(results)[2]
            println()
        end
    else
        start_values = initial_pid_params[control_method][pid_method]
        println("\noptimizing...")
        println("initial pid params: ", initial_pid_params[control_method][pid_method])
        if pid_method == "PID" && !optimize_t_i
            start_values = initial_pid_params[control_method]["PID"][[1, 3]]
        end
        results = Optim.optimize(objective, start_values, SimulatedAnnealing(), Optim.Options(time_limit=seconds))
        objective(Optim.minimizer(results), true)
        println("optimized params: ", initial_pid_params[control_method][pid_method])
        if pid_method == "PID" && !optimize_t_i
            initial_pid_params[control_method]["PID"][[1, 3]] = Optim.minimizer(results)
        else
            initial_pid_params[control_method][pid_method] = Optim.minimizer(results)
        end
    end
end

if optimizing
    optimize()
    println("The simulation was ", round(Tf / calc_avg_time()), " times faster than realtime.")
else
    objective(initial_pid_params[control_method][pid_method], true)
end


YAML.write_file("params.yaml", initial_pid_params)

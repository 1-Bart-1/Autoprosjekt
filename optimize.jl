using Optim
include("simulator.jl")

Tf = 70.0
Ts = 0.1
desired_water_level = 0.315
control_method = "valve"
test_types = ["disturbance1", "disturbance2", "disturbance3", "no-disturbance"]
# test_types = ["disturbance1"]
delay = 0.05
overswing_percentage = 0.1
deviation_percentage = 0.01

function plot_sim(sol, title="Water Reservoir")
    
    # maxes = [maximum(abs.([u[i] for u in sol.u])) for i in 1:length(sol.u[1])]
    # plot_u = [[u[i] / maxes[i] for u in sol.u] for i in 1:length(sol.u[1])]
    # p = plot(sol.t, plot_u, title="Water Reservoir with variable outflow rate", xlabel="Time", ylabel="Water level")
    
    p = plot(sol, title=title, xlabel="Time", ylabel="Water level")
    display(p)
    savefig(p, "10% fault percentage - $title.png")
end

function objective(pid_params, print=false)
    cost = 0.0
    for test_type in test_types
        sol, p = simulate(pid_params, Tf, Ts, desired_water_level, control_method, test_type, delay)
        
        max_level = maximum([u[1] for u in sol.u])
        time_reached_level = Tf*2
        for (t, u) in zip(sol.t, sol.u)
            if t >= 1 && u[1] >= p.desired_water_level*0.99
                time_reached_level = t
                break
            end
        end
        
        overswing_cost = max(abs(max_level - p.desired_water_level), overswing_percentage*p.desired_water_level) / p.desired_water_level
        time_cost = time_reached_level / (Tf*2)
        stable_deviation = abs(sol.u[end][1] - p.desired_water_level)
        stable_deviation_cost = max(stable_deviation, deviation_percentage*p.desired_water_level) / p.desired_water_level
        cost += overswing_cost*10 + time_cost + stable_deviation_cost*10
        
        # plot_sim(sol)
        if print
            plot_sim(sol, test_type)
            println("Objective summary:")
            println("\t Test type: ", test_type)
            println("\t Max level: ", max_level)
            println("\t Desired level: ", p.desired_water_level)
            println("\t Overswing percentage: ", (max_level-p.desired_water_level)/p.desired_water_level*100, "%")
            println("\t Stable deviation percentage: ", (stable_deviation)/p.desired_water_level*100, "%")
            println("\t Time to reach level: ", time_reached_level)
            println("\t Pid params: ", pid_params)
            println("\t Cost: ", cost)
        end
    end
    
    return cost
end

function optimize()
    lower = [0, 0, 0]
    upper = [Inf, Inf, Inf]
    initial_pid_params = [15504.552881017757, 17.89357148908523, 143.40838751347013]

    results = Optim.optimize(objective, lower, upper, initial_pid_params, NelderMead(), Optim.Options(time_limit=100.0))
    println(summary(results))
    objective(Optim.minimizer(results), true)
end

optimize()
# objective([4.137311137212751, 10.74262363380598, 3.088050317422888], true)
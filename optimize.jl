using Optim
include("simulator.jl")

Tf = 40.0
Ts = 0.1
desired_water_level = 0.315
start_water_level = 0.315
control_method = "position"
test_type = "no-disturbance"
delay = 0.2

function plot_sim(sol)
    
    # maxes = [maximum(abs.([u[i] for u in sol.u])) for i in 1:length(sol.u[1])]
    # plot_u = [[u[i] / maxes[i] for u in sol.u] for i in 1:length(sol.u[1])]
    # p = plot(sol.t, plot_u, title="Water Reservoir with variable outflow rate", xlabel="Time", ylabel="Water level")
    
    p = plot(sol, title="Water Reservoir", xlabel="Time", ylabel="Water level")
    display(p)
    # savefig(p, "Realistic sim - with time delay.png")
end

function objective(pid_params, print=false)
    sol, p = simulate(pid_params, Tf, Ts, desired_water_level, control_method, test_type, delay)
    max_level = maximum([u[1] for u in sol.u])
    time_reached_level = Tf*2
    for (t, u) in zip(sol.t, sol.u)
        if u[1] >= p.desired_water_level*0.99
            time_reached_level = t
            break
        end
    end
    
    plot_sim(sol)
    
    cost = max(abs(max_level - p.desired_water_level), 0.01*p.desired_water_level) * time_reached_level
    
    if print
        println("Objective summary:")
        println("\t Max level: ", max_level)
        println("\t Desired level: ", p.desired_water_level)
        println("\t Overswing percentage: ", (max_level-p.desired_water_level)/p.desired_water_level*100, "%")
        println("\t Time to reach level: ", time_reached_level)
        println("\t Pid params: ", pid_params)
        println("\t Cost: ", cost)
    end
    return cost
end

function optimize()
    # simulate([1.15373308782321, 581.0078385651256, 21.137621880761632])

    lower = [0, 0, 0]
    upper = [Inf, Inf, Inf]
    initial_pid_params = [5.050970853758569, 60.74808584643459, 1.0497595666451431]

    # i = 100
    # minimum_cost = 1000000.0
    # while minimum_cost > 20.0
    #     local initial_pid_params = [float(i), 0.0, 0.0]
    #     local results = optimize(simulate, lower, upper, initial_pid_params, NelderMead(), Optim.Options(time_limit=10.0))
    #     global minimum_cost = Optim.minimum(results)
    #     println(summary(results))
    #     println(Optim.iterations(results))
    #     println(Optim.minimizer(results))
    #     println(Optim.minimum(results))
    #     global i += 10
    # end

    results = Optim.optimize(objective, lower, upper, initial_pid_params, NelderMead(), Optim.Options(time_limit=10.0))
    println(summary(results))
    # println(Optim.minimizer(results))
    # println(Optim.minimum(results))
    objective(Optim.minimizer(results), true)
end

# optimize()
objective([5.4031693504613525, 43.45363500025758, 1.4464488024488569], true)
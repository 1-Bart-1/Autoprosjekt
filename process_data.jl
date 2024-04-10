using Polynomials
using DataFrames
using CSV
using Plots

function get_polynomials()
    data = CSV.read("data/TankData.csv", DataFrame)

    time_disturbance1 = collect(skipmissing(data[!, :"time-disturbance1"]))
    level_disturbance1 = collect(skipmissing(data[!, :"level-disturbance1"])) / 100
    println(time_disturbance1)
    println(level_disturbance1)

    # Fit a quadratic polynomial to the points
    water_level = fit(time_disturbance1, level_disturbance1, 2) # 2 is the degree of the polynomial
    println("Disturbance1 speed at 0.5m: ", derivative(water_level)(0.5))

    
    # time_pol = range(0, stop=150, length=1000)
    # water_level_pol = water_level.(time_pol)
    # plot(time_pol, water_level_pol, title=string("Level: ", water_level))
    # p = scatter!(time_disturbance1, level_disturbance1, label="Original Points", markershape=:circle, markersize=2)
    # display(p)
    # savefig("data/water_level_over_time.png")
    return water_level
end

get_polynomials()
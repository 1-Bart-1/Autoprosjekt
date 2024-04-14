using Polynomials
using DataFrames
using CSV
using Plots

export disturbance_to_rate, get_polynomials

function get_disturbance(data, column_name, p)
    time_column = Symbol("time-$column_name")
    level_column = Symbol("level-$column_name")

    time = collect(skipmissing(data[!, time_column]))
    level = collect(skipmissing(data[!, level_column])) / 100
    scatter!(p, time, level, label=column_name, markershape=:circle, markersize=2)
    disturbance = fit(time, level, 2) # 2 is the degree of the polynomial
    return disturbance
end

function get_fill_rate(data, column_name, p, inverse=false)
    frequency_column = Symbol("$column_name")
    rate_column = Symbol("rate-$column_name")

    frequency = collect(skipmissing(data[!, frequency_column]))
    rate = collect(skipmissing(data[!, rate_column]))

    scatter!(p, frequency, rate, label=column_name, markershape=:circle, markersize=2)
    if !inverse
        fill_rate = fit(frequency, rate, 1)
    else
        fill_rate = fit(rate, frequency, 1)
    end
    return fill_rate
end

function disturbance_to_rate(disturbance, level)
    time_at_level = minimum(real(roots(disturbance - level)))
    flow_rate = derivative(disturbance)
    rate_at_level = flow_rate(time_at_level)
    return min(rate_at_level, 0.0) # rate is always negative
end

function plot_polynomials(polynomials, names, p, stop)
    for (poly, name) in zip(polynomials, names)
        time_pol = range(0, stop=stop, length=1000)
        water_level_pol = poly.(time_pol)
        plot!(p, time_pol, water_level_pol, label=name)
    end
    # savefig("data/water_level_over_time.png")
end

function get_polynomials()
    data = CSV.read("data/TankData.csv", DataFrame)
    p1 = plot()
    p2 = plot()
    p3 = plot()

    disturbance1 = get_disturbance(data, "disturbance1", p1)
    disturbance2 = get_disturbance(data, "disturbance2", p1)
    disturbance3 = get_disturbance(data, "disturbance3", p1)
    bigdisturbance = get_disturbance(data, "bigdisturbance", p1)


    frequency_fill_rate = get_fill_rate(data, "frequency", p2)
    valve_fill_rate = get_fill_rate(data, "valve", p3)
    frequency_to_rate = get_fill_rate(data, "frequency", p2, true)
    valve_to_rate = get_fill_rate(data, "valve", p3, true)

    # println("disturbance1 speed at 0.315m: ", disturbance_to_rate(disturbance1, 0.315))

    plot_polynomials([disturbance1, disturbance2, disturbance3, bigdisturbance], ["disturbance1", "disturbance2", "disturbance3", "bigdisturbance"], p1, 150)
    plot_polynomials([frequency_fill_rate], ["frequency_fill_rate"], p2, 50)
    plot_polynomials([valve_fill_rate], ["valve_fill_rate"], p3, 1)

    # p = plot(p1, p2, p3, layout = (3, 1))  # Combine the plots into a single plot with 3 subplots
    # xlabel!(p, "time")
    # display(p4)

    return disturbance1, disturbance2, disturbance3, bigdisturbance, frequency_fill_rate, valve_fill_rate, frequency_to_rate, valve_to_rate
end

get_polynomials();
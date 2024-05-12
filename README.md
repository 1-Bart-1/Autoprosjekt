# Water Tank + PLS Regulator Simulator

This project is a simulator for a water tank and a PLS (Proportional-Integral-Derivative) regulator. It is combined with an optimizing algorithm.

The simulator is designed to optimize a PID-controller. With the power of this simulator, you can optimize your PID-controller in less than 5 minutes.

This makes it an efficient tool for rapid prototyping and testing of PID controllers, saving you time and effort in the process.

# Simulator

The simulator is fast: it is more than 500 times faster than realtime.

# How to use

Install Julia using juliaup: https://github.com/JuliaLang/juliaup

Install needed packages:
In command line:
$julia

In julia:
julia> using Pkg
julia> Pkg.add(["Optim", "Statistics", "Plots", "ControlSystemsBase", "DelayDiffEq", "DiffEqCallbacks", "Polynomials", "DataFrames", "CSV", "Serialization", "ProgressMeter", "YAML"])

To run the optimization process:
julia> include("optimize.jl")

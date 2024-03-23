using WaterLily

function circle(n,m;Re=250,U=1)
    radius, center = m/8, m/2
    body = AutoBody((x,t)->√sum(abs2, x .- center) - radius)
    Simulation((n,m), (U,0), radius; ν=U*radius/Re, body)
end

circ = circle(3*2^6,2^7)
t_end = 10
sim_step!(circ,t_end)

using Plots
contour(circ.flow.p')

should add option for controlling valve opening speed or valve position with pid

Uff:
        disturbance1 speed at 0.315m: 0.00442010398903692
        measurement showing: 24.0
        24/0.00442 = 5430

        Uff_freq = 23.3363 + 510.508 * (measured_flow_rate / 5430)
        Uff_valve = 0.238858 + 13.1348 * (measured_flow_rate / 5430)

[4.466632691496241, 85.4723291594515, 1.0979373894449622]
the best pid variables for Realistic sim - position method.png
        desired_water_level = 0.315, # desired height
        control_method = "position", # Control position or speed

delay of 0.0 seconds and optimized:
Objective summary:
         Max level: 0.314926830013172
         Desired level: 0.315
         Overswing percentage: -0.023228567246981666%
         Time to reach level: 11.9
         Pid params: [5.4031693504613525, 43.45363500025758, 1.4464488024488569]
         Cost: 0.037485000000000004

delay of 0.2 seconds with same pid already had 4% overswing:
Objective summary:
         Max level: 0.3273767921646616
         Desired level: 0.315
         Overswing percentage: 3.929140369733837%
         Time to reach level: 10.9
         Pid params: [5.4031693504613525, 43.45363500025758, 1.4464488024488569]
         Cost: 0.13490703459481132


Solve time:
The avg solve time: 0.20024719829761897 seconds for Tf = 100.0 seconds. So it is 500 times faster than realtime
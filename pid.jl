using Plots

Up_values = []
Ui_values = []
Ud_values = []
outputs = []
time_values = []


mutable struct PIDState
    K_gain::Float32
    Ti::Float32
    Td::Float32
    derivationFilter::Float32
    mode_Tracking::Bool
    Uff::Float32
    Uman::Float32
    Unom::Float32
    outputSat::Float32
    Error::Float32
    lastError::Float32
    dInput::Float32
    lastInput::Float32
    lastOutput::Float32
    autoMode::Bool
    controlMode::Bool
    ITerm::Float32
    lastMode::Bool
    outMin::Float32
    outMax::Float32
    Ts::Float32
    Up::Float32
    Ui::Vector{Float32}
    Ud::Vector{Float32}
    Beta::Float32
    Alpha::Float32
    Sigma::Float32
    output::Float32
    Usat::Float32
    Tau_lead::Float32
    Tau_lag::Float32
    FF_Mode::Int
    internal::Vector{Float32}
    Kf::Float32
    UserInput::Float32
    ValveSpeed::Float32
    FreqSpeed::Float32
    lastSpeed::Float32
    accelTime::Float32
    valveAccel::Float32
    freqAccel::Float32
end

function PIDState()
    K_gain = 45.0690284349867
    Ti = 0.3126727304685277
    Td = 9.493077005765745
    derivationFilter = 10.
    mode_Tracking = true
    Uff = 0.
    Uman = 0.
    Unom = 0.
    outputSat = 0.
    Error = 0.
    lastError = 0.
    dInput = 0.
    lastInput = 0.
    lastOutput = 49.8
    autoMode = true
    controlMode = false
    ITerm = 0.
    lastMode = true
    outMin = 0.
    outMax = 100.
    Ts = 0.1
    Up = 0.
    Ui = [0., 0.]
    Ud = [0., 0.]
    Beta = 0.
    Alpha = 0.
    Sigma = 0.
    output = 49.8
    Usat = 0.
    Tau_lead = 0.
    Tau_lag = 0.
    FF_Mode = 0
    internal = [0., 0.]
    Kf = 0.0
    UserInput = 100.0
    ValveSpeed = 100.0/30.0
    FreqSpeed = 100.0/3.25
    lastSpeed = 0.0
    accelTime = 1.
    valveAccel = (ValveSpeed * 2.0) / accelTime
    freqAccel = (FreqSpeed * 2.0) / accelTime

    return PIDState(
        K_gain, Ti, Td, derivationFilter, mode_Tracking, Uff, Uman, Unom, outputSat, Error, lastError, dInput, lastInput, lastOutput, autoMode, controlMode, ITerm, lastMode, outMin, outMax, Ts, Up, Ui, Ud, Beta, Alpha, Sigma, output, Usat, Tau_lead, Tau_lag, FF_Mode, internal, Kf, UserInput, ValveSpeed, FreqSpeed, lastSpeed, accelTime, valveAccel, freqAccel
    )
end


function set_parameters!(state::PIDState, pid_params::Vector{Float32}, pid_method::String, Ts::Float32)
    state.autoMode = true
    state.Ts = Ts
    if pid_method == "none"
        state.autoMode = false
    elseif pid_method == "P"
        state.K_gain = pid_params[1]
        state.Ti = 0
        state.Td = 0
        state.Unom = pid_params[2]
        state.FF_Mode = 0
    elseif pid_method == "PI"
        state.K_gain = pid_params[1]
        state.Ti = pid_params[2]
        state.Td = 0
        state.Unom = 0.0
        state.FF_Mode = 0
    elseif pid_method == "PD"
        state.K_gain = pid_params[1]
        state.Ti = 0
        state.Td = pid_params[2]
        state.Unom = pid_params[3]
        state.FF_Mode = 0
    elseif pid_method == "PID"
        state.K_gain = pid_params[1]
        state.Ti = pid_params[2]
        state.Td = pid_params[3]
        state.Unom = 0.0
        state.FF_Mode = 0
    elseif pid_method == "LL"
        # state.K_gain = pid_params[1]
        # state.Ti = pid_params[2]
        # state.Td = pid_params[3]
        state.Tau_lead = pid_params[1]
        state.Tau_lag = pid_params[2]
        state.Kf = pid_params[3]
        state.FF_Mode = 1
    elseif pid_method == "FF"
        state.K_gain = pid_params[1]
        state.Ti = 0.0
        state.Td = pid_params[2]
        state.Unom = 0.0
        state.FF_Mode = 2
    end
end

function change_mode!(state::PIDState, mode::Bool)
    if mode
        state.autoMode = true
    else
        state.autoMode = false
    end
end

function change_control_mode!(state::PIDState, mode::Bool, Ui_start::Float32)
    if mode
        state.controlMode = true # frequency mode
    else
        state.controlMode = false # valve mode
    end
    start = Ui_start
    state.Ui[1] = start
    state.Ui[2] = start
    state.output = start
    state.lastOutput = start
end


"""
Setpoint: percentage
Input: percentage
FlowRate: measured value from the flow meter
"""
function pid(state::PIDState, Setpoint::Float32, Input::Float32, FlowRate::Float32)
    # println("Setpoint: ", Setpoint, ", Input: ", Input, ", FlowRate: ", FlowRate)
    if state.autoMode && !state.lastMode
        state.lastInput = Input
        state.Ui[1] = state.outputSat
    end

    if !state.autoMode && state.lastMode
        state.Uman = state.outputSat
    end

    if state.autoMode
        mode_P = (state.K_gain != 0.0 && state.Ti == 0.0 && state.Td == 0.0)
        mode_PI = (state.Ti != 0.0 && state.Td == 0.0)
        mode_PD = (state.Ti == 0.0 && state.Td != 0.0)
        mode_PID = (state.Ti != 0.0 && state.Td != 0.0)

        # P-leddet
        if mode_P || mode_PI || mode_PD || mode_PID
            state.Error = Setpoint - Input
            state.dInput = Input - state.lastInput
            state.Up = state.K_gain * state.Error
        end

        # I-leddet
        if mode_PI || mode_PID
            Alpha = state.Ts / state.Ti
            state.Ui[2] = state.Ui[1] + state.K_gain * Alpha * state.Error
            state.Ui[2] = clamp(state.Ui[2], state.outMin, state.outMax)
            state.Unom = 0.0
        else
            state.Ui[2] = 0.0
        end

        # D-leddet
        if mode_PD || mode_PID && state.lastInput != 0.0
            Beta = state.Td / (state.Td + state.derivationFilter)
            Sigma = state.K_gain * (state.Td / state.Ts) * (1.0 - Beta)
            state.Ud[2] = Beta * state.Ud[1] - Sigma * state.dInput
        else
            state.Ud[2] = 0.0
        end

        # lead_lag
        if state.FF_Mode == 1
            state.internal[2] = (1.0f0 - state.Ts / state.Tau_lag)*state.internal[1] + (state.Ts / state.Tau_lag)*FlowRate
            state.Uff = state.Kf * (1.0f0 - (state.Tau_lead / state.Tau_lag))*state.internal[1] + state.Kf * (state.Tau_lead / state.Tau_lag)*FlowRate
            state.internal[1] = state.internal[2]
        elseif state.FF_Mode == 2
            # real_rate = -0.00571194 + 1.48893e-5 * FlowRate # trenger på plsen!!
            real_rate = FlowRate
            # println("flowrate")
            if state.controlMode == true # frequency mode
                state.Uff = (23.3363 + 510.508 * real_rate)
                state.Uff = state.Uff / 50. * 100. # convert from Hz to percentage
            else
                state.Uff = (0.238858 + 13.1348 * real_rate) # valve mode
                state.Uff = state.Uff * 100. # convert from fraction to percentage
            end
            # println("Uff: ", state.Uff)
            # println(Input)
        else
            state.Uff = 0.0
        end

        # println("Up: ", state.Up, ", Ui[2]: ", state.Ui[2], ", Ud[2]: ", state.Ud[2], ", Unom: ", state.Unom, ", Uff: ", state.Uff)
        state.output = state.Up + state.Ui[2] + state.Ud[2] + state.Unom + state.Uff
        # state.output = state.Up + state.Uff

    end

    # mode_Manual
    if !state.autoMode
        state.Uman = state.UserInput
        state.output = state.Uman
    end
    
    
    # # clamping
    # if state.output > state.outMax
    #     state.Ui[2] -= state.output - state.outMax
    # elseif state.output < state.outMin
    #     state.Ui[2] += state.outMin - state.output
    # else
    #     state.output = state.output
    # end
    
    # mode_Tracking
    if state.controlMode == false
        speed = (state.output - state.lastOutput) / state.Ts

        if speed > state.ValveSpeed
            speed = state.ValveSpeed
        elseif speed < -state.ValveSpeed
            speed = -state.ValveSpeed
        end
        state.output = state.lastOutput + speed * state.Ts

        accel = (speed - state.lastSpeed) / state.Ts

        if accel > state.valveAccel
            speed = state.lastSpeed + state.valveAccel * state.Ts
        elseif accel < -state.valveAccel
            speed = state.lastSpeed - state.valveAccel * state.Ts
        end
        state.output = state.lastOutput + speed * state.Ts

        state.lastOutput = state.output
        state.lastSpeed = speed
    else
        speed = (state.output - state.lastOutput) / state.Ts

        if speed > state.FreqSpeed
            speed = state.FreqSpeed
        elseif speed < -state.FreqSpeed
            speed = -state.FreqSpeed
        end
        state.output = state.lastOutput + speed * state.Ts

        accel = (speed - state.lastSpeed) / state.Ts

        if accel > state.freqAccel
            speed = state.lastSpeed + state.freqAccel * state.Ts
        elseif accel < -state.freqAccel
            speed = state.lastSpeed - state.freqAccel * state.Ts
        end
        state.output = state.lastOutput + speed * state.Ts

        state.lastOutput = state.output
        state.lastSpeed = speed
    end

    state.Ui[1] = state.Ui[2]
    state.Ud[1] = state.Ud[2]
    state.lastInput = Input
    state.lastMode = state.autoMode

    # push!(Up_values, state.Up)
    # push!(Ui_values, state.Ui[2])
    # push!(Ud_values, state.Ud[2])
    # # push!(outputs, state.output)

    # display(plot([Up_values, Ui_values, Ud_values], label = ["Up Ui Ud"]))
    return state.output
end

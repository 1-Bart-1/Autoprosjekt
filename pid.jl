

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
    autoMode::Bool
    ITerm::Float32
    lastMode::Bool
    outMin::Float32
    outMax::Float32
    Ts::Float32
    Up::Float32
    Ui::Vector{Float32}
    Ud::Vector{Float32}
    Beta::Float32
    Tt::Float32
    Alpha::Float32
    Sigma::Float32
    gamma::Float32
    output::Float32
    Usat::Float32
end

function PIDState()
    K_gain = 1.
    Ti = 0.1
    Td = 0.1
    derivationFilter = 10.
    mode_Tracking = false
    Uff = 0.
    Uman = 0.
    Unom = 0.
    outputSat = 0.
    Error = 0.
    lastError = 0.
    dInput = 0.
    lastInput = 0.
    autoMode = true
    ITerm = 0.
    lastMode = false
    outMin = 0.
    outMax = 4082.
    Ts = 0.1
    Up = 0.
    Ui = [0., 0.]
    Ud = [0., 0.]
    Beta = 0.
    Tt = 0.
    Alpha = 0.
    Sigma = 0.
    gamma = 0.
    output = 0.
    Usat = 0.

    return PIDState(
        K_gain, Ti, Td, derivationFilter, mode_Tracking, Uff, Uman, Unom, outputSat, Error, lastError, dInput, lastInput, autoMode, ITerm, lastMode, outMin, outMax, Ts, Up, Ui, Ud, Beta, Tt, Alpha, Sigma, gamma, output, Usat
    )
end

function reset!(state::PIDState)
    state.K_gain = 1.
    state.Ti = 0.1
    state.Td = 0.1
    state.derivationFilter = 10.
    state.mode_Tracking = false
    state.Uff = 0.
    state.Uman = 0.
    state.outputSat = 0.
    state.Error = 0.
    state.lastError = 0.
    state.dInput = 0.
    state.lastInput = 0.
    state.autoMode = true
    state.ITerm = 0.
    state.lastMode = false
    state.outMin = 0.
    state.outMax = 4082.
    state.Ts = 0.1
    state.Up = 0.
    state.Ui = [0., 0.]
    state.Ud = [0., 0.]
    state.Beta = 0.
    state.Tt = 0.
    state.Alpha = 0.
    state.Sigma = 0.
    state.gamma = 0.
    state.output = 0.
    state.Usat = 0.
end

function set_parameters!(state::PIDState, pid_params::Vector{Float32}, pid_method::String)
    if pid_method == "P"
        state.K_gain = pid_params[1]
        state.Ti = 0
        state.Td = 0
        state.Unom = pid_params[2]
    elseif pid_method == "PI"
        state.K_gain = pid_params[1]
        state.Ti = pid_params[2]
        state.Td = 0
        state.Unom = 0.0
    elseif pid_method == "PD"
        state.K_gain = pid_params[1]
        state.Ti = 0
        state.Td = pid_params[2]
        state.Unom = pid_params[3]
    elseif pid_method == "PID"
        state.K_gain = pid_params[1]
        state.Ti = pid_params[2]
        state.Td = pid_params[3]
        state.Unom = 0.0
    end
end


# function lead_lag()
#     if FF_Mode == 1
#         internal[2] = (1.0f0 - Ts / tau_lag)*internal[1] + (Ts / tau_lag) * flowRate
#         Uff = (1.0f0 - (tau_lead / tau_lag))*internal[1] + (tau_lead / tau_lag)*flowRate
#         internal[1] = internal[2]
#     else
#         Uff = 0.0f0
#     end
#     return Uff
# end


function pid(state::PIDState, Setpoint::Float32, Input::Float32)
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
            Error = Setpoint - Input
            dInput = Input - state.lastInput
            Up = state.K_gain * Error
        end

        # I-leddet
        if mode_PI || mode_PID
            Alpha = state.Ts / state.Ti
            gamma = state.mode_Tracking ? state.Ts / state.Tt : 0.0
            state.Ui[2] = state.Ui[1] + state.K_gain * Alpha * Error + gamma * (state.outputSat - state.output)
            state.Ui[2] = clamp(state.Ui[2], state.outMin, state.outMax)
        else
            state.Ui[2] = 0.0
        end

        # D-leddet
        if mode_PD || mode_PID
            Beta = state.Td / (state.Td + state.derivationFilter)
            Sigma = state.K_gain * (state.Td / state.Ts) * (1.0 - Beta)
            state.Ud[2] = Beta * state.Ud[1] - Sigma * dInput
        else
            state.Ud[2] = 0.0
        end

        # println(state.K_gain)
        state.output = Up + state.Ui[2] + state.Ud[2] + state.Unom + state.Uff
        # println("Output: ", state.output)
    end

    if !state.autoMode
        state.Uman = state.userInput
        state.output = state.Uman
    end

    if state.output > state.outMax
        state.outputSat = state.outMax
        state.Ui[2] -= state.output - state.outMax
    elseif state.output < state.outMin
        state.outputSat = state.outMin
        state.Ui[2] += state.outMin - state.output
    else
        state.outputSat = state.output
    end

    state.Ui[1] = state.Ui[2]
    state.Ud[1] = state.Ud[2]
    state.lastInput = Input
    state.lastMode = state.autoMode

    return state.outputSat
end


# p=PIDState()
# pid(p, Float32(7.), Float32(6.))
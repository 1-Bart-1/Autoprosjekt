
module PID

export pid, set_parameters, reset

mutable struct PIDState
    K_gain::Float32
    Ti::Float32
    Td::Float32
    derivationFilter::Float32
    mode_Tracking::Bool
    Uff::Float32
    Uman::Float32
    outputSat::Float32
    Error::Float32
    lastError::Float32
    dInput::Float32
    lastInput::Float32
    autoMode::Bool
    ITerm::Float32
    lastAutoMode::Bool
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

function reset()
    K_gain = 1.
    Ti = 0.1
    Td = 0.1
    derivationFilter = 4082.
    mode_Tracking = false
    Uff = 0.
    Uman = 0.
    outputSat = 0.
    Error = 0.
    lastError = 0.
    dInput = 0.
    lastInput = 0.
    autoMode = true
    ITerm = 0.
    lastAutoMode = false
    outMin = 0.
    outMax = 0.
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
        K_gain, Ti, Td, derivationFilter, mode_Tracking, Uff, Uman, outputSat, Error, lastError, dInput, lastInput, autoMode, ITerm, lastAutoMode, outMin, outMax, Ts, Up, Ui, Ud, Beta, Tt, Alpha, Sigma, gamma, output, Usat
    )
end

# function reset()
#     global K_gain, Ti, Td, derivationFilter, mode_Tracking, Uff, Uman, outputSat, Error, lastError, dInput, lastInput, autoMode, ITerm, lastAutoMode, outMin, outMax, Ts, Up, Ui, Ud, Beta, Tt, Alpha, Sigma, gamma, output, Usat
#     K_gain = 1.
#     Ti = 0.1
#     Td = 0.1
#     derivationFilter = 10.
#     mode_Tracking = false
#     Uff = 0.
#     Uman = 0.

#     outputSat = 0.

#     Error = 0.
#     dInput = 0.

#     lastInput = 0.
#     autoMode = true
#     ITerm = 0.
#     lastAutoMode = false

#     outMin = 0.
#     outMax = 4082.
#     Ts = 0.1

#     Up = 0.
#     Ui = [0.0, 0.0]
#     Ud = [0.0, 0.0]
#     Beta = 0.
#     Tt = 0.
#     Alpha = 0.
#     Sigma = 0.
#     gamma = 0.

#     output = 0.
#     Usat = 0.
# end

function set_parameters(state::PIDState, pid_params::Vector{Float32}, pid_method::String)
    if pid_method == "P"
        state.K_gain = pid_params[1]
        state.Ti = 0
        state.Td = 0
    elseif pid_method == "PI"
        state.K_gain = pid_params[1]
        state.Ti = pid_params[2]
        state.Td = 0
    elseif pid_method == "PD"
        state.K_gain = pid_params[1]
        state.Ti = 0
        state.Td = pid_params[2]
    elseif pid_method == "PID"
        state.K_gain = pid_params[1]
        state.Ti = pid_params[2]
        state.Td = pid_params[3]
    end
end

function pid(state::PIDState, Setpoint::Float32, Input::Float32)
    mode_PD = (state.Ti == 0 && state.Td != 0)
    mode_PID = (state.Ti != 0 && state.Td != 0)
    mode_PI = (state.Ti != 0 && state.Td == 0)

    if state.autoMode & !state.lastAutoMode
        state.lastInput = Input
        state.Ui = [state.outputSat, 0]
    end

    if state.autoMode
        Error = Setpoint - Input
        dInput = Input - state.lastInput
        
        Up = state.K_gain * Error

        if mode_PI || mode_PID
            Alpha = state.Ts / state.Ti
            gamma = state.mode_Tracking ? state.Ts / state.Tt : 0

            state.Ui[2] = state.Ui[1] + state.K_gain * Alpha * Error + gamma * (state.outputSat - state.output)

            state.Ui[2] = clamp(state.Ui[2], state.outMin, state.outMax)
        else
            state.Ui[2] = 0
        end

        if mode_PD || mode_PID
            Beta = state.Td / (state.Td + state.derivationFilter)
            Sigma = state.K_gain * (state.Td / state.Ts) * (1 - Beta)

            state.Ud[2] = Beta * state.Ud[1] - Sigma * dInput
        else
            state.Ud[2] = 0
        end

        state.output = Up + state.Ui[2] + state.Ud[2] + state.Uff
    end

    if !state.autoMode
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
    state.lastAutoMode = state.autoMode

    return state.outputSat
end

end
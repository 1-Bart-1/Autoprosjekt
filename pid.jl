

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
    Tau_lead::Float32
    Tau_lag::Float32
    FF_Mode::Int
    internal::Vector{Float32}
    Kf::Float32
    UserInput::Float32
end

function PIDState()
    K_gain = 410163.3859666207
    Ti = 0.03598674140068368
    Td = 6.032244476391085
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
    Ts = 0.5
    Up = 0.
    Ui = [0., 0.]
    Ud = [0., 0.]
    Beta = 0.
    Tt = 0.0001
    Alpha = 0.
    Sigma = 0.
    gamma = 0.
    output = 0.
    Usat = 0.
    Tau_lead = 0.
    Tau_lag = 0.
    FF_Mode = 0
    internal = [0., 0.]
    Kf = 0.0
    UserInput = 0.3*4082.0

    return PIDState(
        K_gain, Ti, Td, derivationFilter, mode_Tracking, Uff, Uman, Unom, outputSat, Error, lastError, dInput, lastInput, autoMode, ITerm, lastMode, outMin, outMax, Ts, Up, Ui, Ud, Beta, Tt, Alpha, Sigma, gamma, output, Usat, Tau_lead, Tau_lag, FF_Mode, internal, Kf, UserInput
    )
end


function set_parameters!(state::PIDState, pid_params::Vector{Float32}, pid_method::String)
    state.autoMode = true
    if pid_method == "none"
        state.autoMode = false
    elseif pid_method == "P"
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
    elseif pid_method == "LL"
        # state.K_gain = pid_params[1]
        # state.Ti = pid_params[2]
        # state.Td = pid_params[3]
        state.Tau_lead = pid_params[1]
        state.Tau_lag = pid_params[2]
        state.Kf = pid_params[3]
        state.FF_Mode = 1
    end
end

function change_mode!(state::PIDState, mode::Bool)
    if mode
        state.autoMode = true
    else
        state.autoMode = false
    end
end



function pid(state::PIDState, Setpoint::Float32, Input::Float32, FlowRate::Float32)
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

        # lead_lag
        if state.FF_Mode == 1
            state.internal[2] = (1.0f0 - state.Ts / state.Tau_lag)*state.internal[1] + (state.Ts / state.Tau_lag)*FlowRate
            state.Uff = state.Kf * (1.0f0 - (state.Tau_lead / state.Tau_lag))*state.internal[1] + state.Kf * (state.Tau_lead / state.Tau_lag)*FlowRate
            state.internal[1] = state.internal[2]
            # println("flowrate", FlowRate)
            # println("Uff", state.Uff)
        else
            state.Uff = 0.0f0
        end
        
        state.output = Up + state.Ui[2] + state.Ud[2] + state.Unom + state.Uff
    end

    if !state.autoMode
        state.Uman = state.UserInput
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
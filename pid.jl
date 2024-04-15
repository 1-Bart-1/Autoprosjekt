
module PID

export pid, set_parameters, reset

K_gain = 1.
# K_hyper = [0.9, 1.0, 1.1]
# mode = [0, 1, 2] # 0=manuell, 1=auto, 2=hyper
Ti = 0.1
Td = 0.1
derivationFilter = 0.
mode_Tracking = false
Uff = 0.
Uman = 0.

outputSat = 0.

Error = 0.
dInput = 0.

lastInput = 0.
autoMode = true
ITerm = 0.
lastAutoMode = false

outMin = 0.
outMax = 4082.
Ts = 0.1

Up = 0.
Ui = [0.0, 0.0]
Ud = [0.0, 0.0]
Beta = 0.
Tt = 0.
Alpha = 0.
Sigma = 0.
gamma = 0.

output = 0.
Usat = 0.

function reset()
    global K_gain, Ti, Td, derivationFilter, mode_Tracking, Uff, Uman, outputSat, Error, lastError, dInput, lastInput, autoMode, ITerm, lastAutoMode, outMin, outMax, Ts, Up, Ui, Ud, Beta, Tt, Alpha, Sigma, gamma, output, Usat
    K_gain = 1.
    Ti = 0.1
    Td = 0.1
    derivationFilter = 0.
    mode_Tracking = false
    Uff = 0.
    Uman = 0.

    outputSat = 0.

    Error = 0.
    dInput = 0.

    lastInput = 0.
    autoMode = true
    ITerm = 0.
    lastAutoMode = false

    outMin = 0.
    outMax = 4082.
    Ts = 0.1

    Up = 0.
    Ui = [0.0, 0.0]
    Ud = [0.0, 0.0]
    Beta = 0.
    Tt = 0.
    Alpha = 0.
    Sigma = 0.
    gamma = 0.

    output = 0.
    Usat = 0.
end

function set_parameters(new_K_gain, new_Ti, new_Td)
    global K_gain, Ti, Td
    (K_gain, Ti, Td) = (new_K_gain, new_Ti, new_Td)
end

function pid(Setpoint, Input)
    global K_gain, Ki, Kd, Ti, Td, derivationFilter, mode_Tracking, Uff, Uman, outputSat, Error, lastError, dInput, lastInput, autoMode, ITerm, lastAutoMode, outMin, outMax, Ts, Up, Ui, Ud, Beta, Tt, Alpha, Sigma, gamma, output, Usat
    # Detect if we go from MAN -> AUTO and initialize values again if that is the case
    mode_PD = (Ti == 0 && Td != 0)
    mode_PID = (Ti != 0 && Td != 0)
    mode_PI = (Ti != 0 && Td == 0)

    if autoMode & !lastAutoMode
        lastInput = Input
        Ui = [outputSat, 0]
    end

    # Regulator parameters in position form
    if autoMode
        # PROPORTIONAL
        # Used in all AUTO modes
        # e = r - y
        Error = Setpoint - Input
        dInput = Input - lastInput
        
        # Calculate the contribution from P, I and D parts
        Up = K_gain * Error

        # INTEGRAL
        if mode_PI || mode_PID
            Alpha = Ts / Ti  # Discrete integral time

            # Tracking on / off
            gamma = mode_Tracking ? Ts / Tt : 0

            Ui[2] = Ui[1] + K_gain * Alpha * Error + gamma * (outputSat - output)

            # Limit the size of the I-part, anti-windup. Not necessary? it is theoretically fixed in output
            Ui[2] = clamp(Ui[2], outMin, outMax)
        else
            Ui[2] = 0
        end

        # DERIVATION
        if mode_PD || mode_PID
            Beta = Td / (Td + derivationFilter)
            Sigma = K_gain * (Td / Ts) * (1 - Beta)

            Ud[2] = Beta * Ud[1] - Sigma * dInput  # Beta -> 0 is pure backward difference
        else
            Ud[2] = 0
        end

        # Calculate output
        output = Up + Ui[2] + Ud[2] + Uff
    end

    if !autoMode
        output = Uman
    end

    # Limit the output to be in the range outMin to outMax.
    # The integral is corrected if the regulator tries to output values outside the limits, and should thus stop wind-up from the I-part
    if output > outMax
        outputSat = outMax
        Ui[2] -= output - outMax
    elseif output < outMin
        outputSat = outMin
        Ui[2] += outMin - output
    else
        outputSat = output
    end

    # For each scan, we move the value in scan [n] to [n-1]
    Ui[1] = Ui[2]
    Ud[1] = Ud[2]
    lastInput = Input
    lastAutoMode = autoMode

    return outputSat
end

end
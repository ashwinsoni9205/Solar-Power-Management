function [D1,D2,duty]  = control(Vo, IL)
    % Constants
    V_ref = 200;                % Reference voltage (400V)
    Kp_v = 0.09;                 % Proportional gain for voltage PI controller
    Ki_v = 40;               % Integral gain for voltage PI controller
    Kp_i = 0.02;                 % Proportional gain for current PI controller
    Ki_i = 50;              % Integral gain for current PI controller
    dead_time = 5e-6;           % Dead time in seconds (adjust as needed)
    Ts = 1e-5;                  % Sampling time in seconds

    % Persistent variables for integrators
    persistent error_v_int error_i_int;
    if isempty(error_v_int)
        error_v_int = 0;
        error_i_int = 0;
    end

    % Voltage Control Loop (Outer Loop)
    error_v = V_ref - Vo;       % Voltage error
    error_v_int = error_v_int + (error_v * Ts);  % Integrate the voltage error
    I_ref = (Kp_v * error_v) + (Ki_v * error_v_int);  % Output: Current reference
    
    % Temporary for innerloop tuning
    %I_ref = 7;

    % Current Control Loop (Inner Loop)
    error_i = I_ref - IL;       % Current error
    error_i_int = error_i_int + (error_i * Ts);  % Integrate the current error
    duty_cycle = (Kp_i * error_i) + (Ki_i * error_i_int);  % Output: Duty cycle

    duty = duty_cycle;
    
    % Clamp duty cycle to [0, 1]
    duty_cycle = max(0, min(0.95, duty_cycle));

    % Dead Time Logic
    if duty_cycle > 0.5
        D1 = duty_cycle - dead_time;   % Boost mode (Discharging battery)
        D2 = 0;                       % Disable buck mode
    else
        D1 = 0;                       % Disable boost mode
        D2 = duty_cycle - dead_time;   % Buck mode (Charging battery)
    end
    % Ensure duty cycles remain non-negative after applying dead time
    D1 = max(0, D1);
    D2 = max(0, D2);
end

function [D1,D2,duty_cycle]  = control(Vo, IL)
    % Constants
    V_ref = 200;                % Reference voltage (400V)
    Kp_v = 0.1;                 % Proportional gain for voltage PI controller
    Ki_v = 8;               % Integral gain for voltage PI controller
    Kp_i = 0.02;                 % Proportional gain for current PI controller
    Ki_i = 5;              % Integral gain for current PI controller
    dead_duty = 5e-6/1e-4;           % Dead time in seconds (adjust as needed)

    %% Corrected this Ts recently as per 10kHz sampling, need to tune the pi values again;
    Ts = 1e-4;                  % Sampling time in seconds

    % Persistent variables for integrators
   % Persistent variables for integrators
    persistent error_v_int error_i_int count I_ref_old first_time_entry windup;
    if isempty(error_v_int)
        error_v_int = 0;
        error_i_int = 0;
        count = 0;
        I_ref_old = 0;
        first_time_entry = 0;
        windup = 0;
    end
    
    if(first_time_entry == 0)
        first_time_entry = 1;
        error_v = V_ref - Vo;       % Voltage error
        error_v_int = error_v_int + (error_v * Ts * 1);  % Integrate the voltage error
        I_ref = (Kp_v * error_v) + (Ki_v * error_v_int);  % Output: Current reference
        I_ref_old = I_ref;
    end

    % Voltage Control Loop (Outer Loop)
    if(count == 50)
        error_v = V_ref - Vo;       % Voltage error
        error_v_int = error_v_int + (error_v * Ts * count);  % Integrate the voltage error
        I_ref = (Kp_v * error_v) + (Ki_v * error_v_int);  % Output: Current reference
        I_ref_old = I_ref;
        count = 0;
    else 
        I_ref = I_ref_old;
    end
    
    % Temporary for innerloop tuning
    %I_ref = 7;

    % Current Control Loop (Inner Loop)
    error_i = I_ref - IL + windup;       % Current error
    error_i_int = error_i_int + (error_i * Ts);  % Integrate the current error
    duty_cycle = (Kp_i * error_i) + (Ki_i * error_i_int);  % Output: Duty cycle
    
    % Clamp duty cycle to [0, 1]
    duty = max(0.05, min(0.95, duty_cycle));

    windup = duty - duty_cycle;

    % % Dead Time Logic DONT WRITE THIS IN C2000 CODE AS IT IS TO BE
    % HANDELED BY C2000 HARDWARE AND NOT BY OUR FUNCTION 
    
    D1 = duty - (dead_duty/2);
    D2 = duty + (dead_duty/2);

    count = count + 1;
    
end

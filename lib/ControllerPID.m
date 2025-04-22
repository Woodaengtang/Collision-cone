classdef ControllerPID < handle
    % ControllerPID: A simple PID controller

    properties
        dt              % Time step for integration (s)
        time            % Current simulation time (s)
        timeSim         % Total simulation duration (s)
        
        kp              % Proportional gain
        ki              % Integral gain
        kd              % Differential gain
        
        prevError       % Previous error (NaN until first update)
        intError        % Accumulated integral of error
        output          % Most recent controller output
        uMin            % Minimum input saturation
        uMax            % Maximum input saturation
        
        loggerTime      % Preallocated time log array
        loggerIndex     % Current write index for logging
        loggerError     % Logged error over time
        loggerOutput    % Logged controller output over time
    end

    methods
        function obj = ControllerPID(kp, ki, kd, dt, timeSim)
            % Constructor: set gains and sampling time, initialize state
            if (dt <= 0)
                error("Positive sampling time dt is required");
            elseif (timeSim <= 0)
                error("Positive simulation time sim_time is required");
            end
            obj.kp = kp;
            obj.ki = ki;
            obj.kd = kd;
            obj.dt = dt;
            obj.timeSim = timeSim;
            % Initialize error history to NaN so first derivative term is forced to zero
            obj.prevError = NaN;
            obj.intError  = 0;
            obj.output    = 0;
            obj.uMin = Nan;
            obj.uMax = Nan;
        end

        function reset(obj)
            % reset: clears integral and derivative history
            obj.prevError = NaN;
            obj.intError  = 0;
            obj.output    = 0;
        end

        function setOutputSaturation(obj, u_min, u_max)
            % set_input_saturation: setting the saturation of the controller output
            obj.uMin = u_min;
            obj.uMax = u_max;
        end

        function u = inputSaturation(obj, u)
            % input_saturation: clamping the output to [u_min,u_max]
            if ~isnan(obj.uMin)
                u = max(u, obj.uMin);
            end
            if ~isnan(obj.uMax)
                u = min(u, obj.uMax);   % element‑wise min
            end
        end

        function u = update(obj, reference, measurement)
            % update: compute PID output, forcing D=0 on first call
            %
            % Inputs:
            %   setpoint    - desired target value
            %   measurement - current measured value
            % Output:
            %   u           - control output

            err = reference - measurement;

            % Proportional term
            P = obj.kp.*err;

            % Integral term
            obj.intError = obj.intError + err.*obj.dt;
            I = obj.ki * obj.intError;

            % Derivative term: zero on first update to avoid kick
            if any(isnan(obj.prevError))    % Considering vector operation
                D = 0;
            else
                D = obj.kd.*(err - obj.prevError)./obj.dt;
            end

            unsat = P + I + D;
            obj.output = obj.inputSaturation(obj, unsat); % Combine terms
            u = obj.output;

            obj.prevError = err;    % Save error for next derivative calculation
        end
    end
end

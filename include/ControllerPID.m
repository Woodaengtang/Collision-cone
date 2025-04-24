classdef ControllerPID < handle
    % ControllerPID: A simple PID controller

    properties
        name

        dt              % Time step for integration (s)
        time            % Current simulation time (s)
        timeSim         % Total simulation duration (s)
        
        ioSize          % i/o size

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
        function obj = ControllerPID(name, gains, times, size)
            % Constructor: set gains and sampling time, initialize state
            if (times.dt <= 0)
                error("Positive sampling time dt is required");
            elseif (times.timeSim <= 0)
                error("Positive simulation time sim_time is required");
            end

            obj.name = name;

            obj.kp = gains.kp;
            obj.ki = gains.ki;
            obj.kd = gains.kd;

            obj.dt = times.dt;
            obj.time = 0;
            obj.timeSim = times.timeSim;
            
            obj.ioSize = size;
            
            obj.prevError = NaN([obj.ioSize, 1]); % Initialize error history to NaN so first derivative term is forced to zero
            obj.intError = zeros([obj.ioSize, 1]);
            obj.output = zeros([obj.ioSize, 1]);
            obj.uMin = NaN;
            obj.uMax = NaN;

            obj.loggerTime = obj.time : obj.dt : obj.timeSim;
            obj.loggerIndex = 1;
            obj.loggerError = NaN([obj.ioSize, length(obj.loggerTime)]);
            obj.loggerError(:,1) = zeros([obj.ioSize, 1]);
            obj.loggerOutput = NaN([obj.ioSize, length(obj.loggerTime)]);
            obj.loggerOutput(:,1) = zeros([obj.ioSize, 1]);
        end

        function obj = reset(obj)
            % reset: clears integral and derivative history
            obj.prevError = NaN([obj.ioSize, 1]);
            obj.intError  = zeros([obj.ioSize, 1]);
            obj.output    = zeros([obj.ioSize, 1]);
        end

        function obj = setOutputSaturation(obj, u_min, u_max)
            % set_input_saturation: setting the saturation of the controller output
            obj.uMin = u_min;
            obj.uMax = u_max;
        end

        function u = inputSaturation(obj, u)
            % input_saturation: clamping the output to [u_min,u_max]
            if and(~isnan(obj.uMin), ~isnan(obj.uMax))
                idx = 1;
                for i = 1:length(u)
                    if u(i) > obj.uMax
                        u(i) = obj.uMax;
                    elseif u(i) < obj.uMin
                        u(i) = obj.uMin;
                    end
                    idx = idx + 1;
                end
            end
        end

        function obj = ctrlLogger(obj)
            obj.loggerIndex = obj.loggerIndex + 1;
            if obj.loggerIndex <=  length(obj.loggerTime)
                obj.loggerError(:,obj.loggerIndex) = obj.prevError;
                obj.loggerOutput(:,obj.loggerIndex) = obj.output;
            else
                disp([obj.name, " controller logging finished"]);
            end
        end

        function output = update(obj, reference, measurement)
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
            obj.output = obj.inputSaturation(unsat); % Combine terms
            output = obj.output;

            obj.prevError = err;    % Save error for next derivative calculation
            obj.ctrlLogger();
        end
    end
end

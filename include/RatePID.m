classdef RatePID < ControllerPID
    
    properties
        current_rate
    end

    methods
        function obj = RatePID(name, gains, times, ioSize)
            obj = obj@ControllerPID(name, gains, times, ioSize);
            obj.current_rate = NaN([3,1]);
        end

        function obj = getRate(obj, current_rate)
            obj.current_rate = current_rate;
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
            if any(isnan(obj.current_rate))    % Considering vector operation
                D = 0;
            else
                D = -obj.kd.*obj.current_rate;
            end

            unsat = (P + I + D)*(pi/180);
            obj.output = obj.inputSaturation(unsat); % Combine terms
            u = obj.output;

            obj.prevError = err;    % Save error for next derivative calculation
            obj.ctrlLogger();
        end
    end
end


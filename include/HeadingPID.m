classdef HeadingPID < ControllerPID
    
    methods
        function obj = HeadingPID(name, gains, times, ioSize)
            obj = obj@ControllerPID(name, gains, times, ioSize);
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

            unsat = (P + I + D)*(pi/180);
            u = obj.inputSaturation(unsat); % Combine terms
            obj.output = u;

            obj.prevError = err;    % Save error for next derivative calculation
            obj.ctrlLogger();
        end
    end
end


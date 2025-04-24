classdef ThrustPID < ControllerPID
    properties
        hoverThrust
    end
    methods
        function obj = ThrustPID(name, gains, times, ioSize, hoverThrust)
            obj = obj@ControllerPID(name, gains, times, ioSize);
            obj.hoverThrust = hoverThrust;
        end

        function thrust = update(obj, reference, measurement)
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

            unsat = obj.hoverThrust + (P + I + D);
            obj.output = obj.inputSaturation(unsat); % Combine terms
            thrust = obj.output;

            obj.prevError = err;    % Save error for next derivative calculation
            obj.ctrlLogger();
        end
    end
end

classdef GuidancePID < ControllerPID

    properties
        current_att
    end
    
    methods
        function obj = GuidancePID(name, gains, times, ioSize)
            obj = obj@ControllerPID(name, gains, times, ioSize);
            obj.current_att = NaN([3,1]);
        end

        function obj = getAtt(obj, current_att)
            obj.current_att = current_att;
        end
        
        function u = update(obj, reference, measurement)
            % update: compute PID output, forcing D=0 on first call
            %
            % Inputs:
            %   setpoint    - desired target value
            %   measurement - current measured value
            % Output:
            %   u           - control output
            bRi = eul2rotm([obj.current_att(3), obj.current_att(2), obj.current_att(1)], 'ZYX');
            err = bRi*(reference - measurement);
            err_x = -err(2);
            err_y = err(1);
            err = [err_x; err_y; err(3)];

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
            obj.output = obj.inputSaturation(unsat); % Combine terms
            u = obj.output;

            obj.prevError = err;    % Save error for next derivative calculation
            obj.ctrlLogger();
        end
    end
end


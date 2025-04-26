function [state, measured] = kineticKalman(motion, time_step)
    % From sensor, NED position is obtained
    % Using kinetic Kalman Filter, Estimate intruder's state
    
    persistent x    % Initial state
    persistent P    % Initial Covariance matrix
    persistent A    % State-Space Eqn A
    persistent C    % State-Space Eqn C
    persistent Q    % Process noise Covariance matrix
    persistent R    % Measurement noise Covariance matrix
    persistent dt   % Time step
    persistent y    % for measured values
    persistent isFirst
    
    if isempty(isFirst)
        isFirst = 1;
        dt = time_step;
        A = eye(9) +...
            dt * [0, 0, 0, 1, 0, 0, 0, 0, 0;...
                  0, 0, 0, 0, 1, 0, 0, 0, 0;...
                  0, 0, 0, 0, 0, 1, 0, 0, 0;...
                  0, 0, 0, 0, 0, 0, 1, 0, 0;...
                  0, 0, 0, 0, 0, 0, 0, 1, 0;...
                  0, 0, 0, 0, 0, 0, 0, 0, 1;...
                  0, 0, 0, 0, 0, 0, 0, 0, 0;...
                  0, 0, 0, 0, 0, 0, 0, 0, 0;...
                  0, 0, 0, 0, 0, 0, 0, 0, 0] + ...
             0.5*dt^2 * [0, 0, 0, 0, 0, 0, 1, 0, 0;...
                         0, 0, 0, 0, 0, 0, 0, 1, 0;...
                         0, 0, 0, 0, 0, 0, 0, 0, 1;...
                         0, 0, 0, 0, 0, 0, 0, 0, 0;...
                         0, 0, 0, 0, 0, 0, 0, 0, 0;...
                         0, 0, 0, 0, 0, 0, 0, 0, 0;...
                         0, 0, 0, 0, 0, 0, 0, 0, 0;...
                         0, 0, 0, 0, 0, 0, 0, 0, 0;...
                         0, 0, 0, 0, 0, 0, 0, 0, 0];
        C = [1, 0, 0, dt, 0, 0, (dt^2)/2, 0, 0;...
             0, 1, 0, 0, dt, 0, 0, (dt^2)/2, 0;...
             0, 0, 1, 0, 0, dt, 0, 0, (dt^2)/2];

        diagP = [1, 1, 1, 1, 1, 1, 1, 1, 1];
        diagQ = 0.1*ones([1, 9]);
        diagR = 0.03*ones([1, 3]);
        P = diag(diagP);
        Q = diag(diagQ);
        R = diag(diagR);
        
        % x = [motion(1), motion(2), motion(3),...
        %     motion(4), motion(5), motion(6), 0, 0, 0]';
        x = [motion(1), motion(2), motion(3),...
            0, 0, 0, 0, 0, 0]';
        y = [];
    end
    
    x_ = A * x;
    P_ = A * P * A' + Q;

    K = (P_ * C') / (C * P_ * C' + R);
    sensed = Sensor(motion);
    x = x_ + K * (sensed - C * x_);
    P = P_ - K * C * P_;
    
    state = x;

    % figure of Measured values
    y = [y, sensed];
    measured = y;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y_ = Sensor(GroundTruth)
    y_ = [GroundTruth(1) + 0.015*randn, GroundTruth(2) + 0.015*randn, GroundTruth(3) + 0.015*randn]';
end
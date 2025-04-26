classdef MultiCopter < handle
    % MultiCopter: A class to simulate the dynamics of a quadcopter.
    % It operates in the FLU coordinate frame.

    properties
        g                   % Gravity acceleration (m/s^2)
        dt                  % Time step for integration (s)
        time                % Current simulation time (s)
        timeSim             % Total simulation duration (s)

        mass                % Mass of the quadcopter (kg)
        armLength           % Distance from center to rotor (m)
        inertia             % Inertia matrix (3×3) of the vehicle

        pos                 % Position vector [x; y; z] in FLU frame (m)
        posF                % Forward component of position (x)
        posL                % Left component of position (y)
        posU                % Up component of position (z)
        posDot              % Time derivative of position (velocity) [m/s]

        vel                 % Velocity vector [vx; vy; vz] in FLU frame (m/s)
        velF                % Forward velocity component (vx)
        velL                % Left velocity component (vy)
        velU                % Up velocity component (vz)
        velDot              % Time derivative of velocity (acceleration) [m/s^2]

        quat                % Attitude quaternion [e0; ex; ey; ez]
        e0                  % Quaternion scalar part
        ex                  % Quaternion x‑component
        ey                  % Quaternion y‑component
        ez                  % Quaternion z‑component
        quatDot             % Time derivative of quaternion

        att                 % Euler angles [roll; pitch; yaw] (rad)
        rol                 % Roll angle φ (rad)
        pit                 % Pitch angle θ (rad)
        yaw                 % Yaw angle ψ (rad)

        omg                 % Body angular rate vector [p; q; r] (rad/s)
        p                   % Roll rate p (rad/s)
        q                   % Pitch rate q (rad/s)
        r                   % Yaw rate r (rad/s)
        omgDot              % Time derivative of angular rate (rad/s^2)

        u                   % Control input vector [T; Mx; My; Mz]
        thrust              % Total thrust command T (N)
        moment              % Moment commands [Mx; My; Mz] (N·m)

        inducedDragCoeff    % Induced drag coefficient matrix (3×3)

        loggerTime          % Preallocated time log array
        loggerIndex         % Current write index for logging
        loggerPos           % Logged position over time
        loggerVel           % Logged velocity over time
        loggerQuat          % Logged quaternion over time
        loggerAtt           % Logged Euler angles over time
        loggerOmg           % Logged angular rates over time
        loggerInput         % Logged control inputs over time
    end


    methods
        function obj = MultiCopter(initCond, initInput, params, simTime)

            obj.g = 9.81;
            obj.time = 0;
            obj.dt = 0.001;
            obj.timeSim = simTime;

            obj.mass = params.Mass;
            obj.armLength = params.armLength;
            obj.inertia = diag([params.Ixx, params.Iyy, params.Izz]);

            obj.pos = initCond.pos;
            [obj.posF, obj.posL, obj.posU] = deal(obj.pos(1), obj.pos(2), obj.pos(3));
            
            obj.vel = initCond.vel;
            [obj.velF, obj.velL, obj.velU] = deal(obj.vel(1), obj.vel(2), obj.vel(3));

            obj.quat = initCond.quat;
            [obj.e0, obj.ex, obj.ey, obj.ez] = deal(obj.quat(1), obj.quat(2), obj.quat(3), obj.quat(4));

            obj.att = obj.quatToEuler(obj.quat);
            [obj.rol, obj.pit, obj.yaw] = deal(obj.att(1), obj.att(2), obj.att(3));

            obj.omg = initCond.omg;
            [obj.p, obj.q, obj.r] = deal(obj.omg(1), obj.omg(2), obj.omg(3));

            obj.posDot = zeros([3, 1]);
            obj.velDot = zeros([3, 1]);
            obj.quatDot = zeros([4, 1]);
            obj.omgDot = zeros([3, 1]);

            obj.u = initInput;
            [obj.thrust, obj.moment] = deal(obj.u(1), obj.u(2:end));
            obj.inducedDragCoeff = zeros([3, 3]);  % No induced drag assumption

            obj.loggerTime = 0 : obj.dt : obj.timeSim;
            obj.loggerIndex = 1;
            obj.loggerPos = NaN([3, length(obj.loggerTime)]);
            obj.loggerVel = NaN([3, length(obj.loggerTime)]);
            obj.loggerQuat = NaN([4, length(obj.loggerTime)]);
            obj.loggerAtt = NaN([3, length(obj.loggerTime)]);
            obj.loggerOmg = NaN([3, length(obj.loggerTime)]);
            obj.loggerInput = NaN([4, length(obj.loggerTime)]);
            obj.loggerPos(:, obj.loggerIndex) = obj.pos;
            obj.loggerVel(:, obj.loggerIndex) = obj.vel;
            obj.loggerQuat(:, obj.loggerIndex) = obj.quat;
            obj.loggerAtt(:, obj.loggerIndex) = obj.quatToEuler(obj.quat);
            obj.loggerOmg(:, obj.loggerIndex) = obj.omg;
            obj.loggerInput(:, obj.loggerIndex) = obj.u;
        end
        
        function euler = quatToEuler(~, quat)
            n = norm(quat);
            if n == 0
                error("Zero‑norm quaternion");
            end
            quat = quat./n;

            e_0 = quat(1);
            e_x = quat(2);
            e_y = quat(3);
            e_z = quat(4);
            
            phi  = atan2(2*(e_0*e_x + e_y*e_z), e_0^2 + e_z^2 - e_x^2 - e_y^2);

            s = 2*(e_0*e_y - e_x*e_z);
            s = max(-1,min(1,s));
            % clamp to [-1,1]
            tht = asin(s);

            psi   = atan2(2*(e_0*e_z + e_x*e_y),  e_0^2 + e_x^2 - e_y^2 - e_z^2);

            euler = [phi; tht; psi];
        end
        function state = getState(obj)
            state = [obj.pos', obj.vel', obj.att', obj.omg']';
        end
 
        function Ri2b = inertial2Body(obj)
            obj.att = obj.quatToEuler(obj.quat);
            [obj.rol, obj.pit, obj.yaw] = deal(obj.att(1), obj.att(2), obj.att(3));
            [sphi, cphi] = deal(sin(obj.rol), cos(obj.rol));
            [stht, ctht] = deal(sin(obj.pit), cos(obj.pit));
            [spsi, cpsi] = deal(sin(obj.yaw), cos(obj.yaw));

            Ri2b = [ctht*cpsi, ctht*spsi, -stht;...
                    sphi*stht*cpsi - cphi*spsi, sphi*stht*spsi + cphi*cpsi, sphi*ctht;...
                    cphi*stht*cpsi + sphi*spsi, cphi*stht*spsi - sphi*cpsi, cphi*ctht];
        end
        
        function posDot = funcPosDot(~, vel)
            posDot = vel;
        end

        function velDot = funcVelDot(obj, vel, thrust)
            ed = [0; 0; -1];
            Ri2b = obj.inertial2Body();
            velDot = obj.g*ed - Ri2b' * obj.inducedDragCoeff * Ri2b * vel - (thrust/obj.mass)*(Ri2b'*ed);
        end

        function quatDot = funcQuatDot(~, quat, omega)
            
            [pOmg, qOmg, rOmg] = deal(omega(1), omega(2), omega(3));
            Omega = [0, -pOmg, -qOmg, -rOmg;...
                     pOmg, 0, rOmg, -qOmg;...
                     qOmg, -rOmg, 0, pOmg;...
                     rOmg, qOmg, -pOmg, 0];
            quatDot = Omega*quat./2;
        end

        function omgDot = funcOmegaDot(~, J, omega, moment)
            omgDot = J\(-cross(omega, J*omega) + moment);
        end

        function obj = equationOfMotion(obj)
            obj.posDot = obj.funcPosDot(obj.vel);
            obj.velDot = obj.funcVelDot(obj.vel, obj.thrust);
            obj.quatDot = obj.funcQuatDot(obj.quat, obj.omg);
            obj.omgDot = obj.funcOmegaDot(obj.inertia, obj.omg, obj.moment);
        end

        function obj = rk4Step(obj)
            h = obj.dt;               % time‑step shorthand

            %------------- k1 ------------------------------------------------
            k1_pos   = obj.funcPosDot(obj.vel);
            k1_vel   = obj.funcVelDot(obj.vel, obj.thrust);
            k1_quat  = obj.funcQuatDot(obj.quat, obj.omg);
            k1_omega = obj.funcOmegaDot(obj.inertia, obj.omg, obj.moment);

            %------------- k2 ------------------------------------------------
            % pos2 = obj.pos + h*k1_pos/2;
            vel2 = obj.vel + h*k1_vel./2;
            quat2 = obj.quat + h*k1_quat./2;
            quat2 = quat2./norm(quat2);
            omg2 = obj.omg + h*k1_omega./2;

            k2_pos   = obj.funcPosDot(vel2);
            k2_vel   = obj.funcVelDot(vel2, obj.thrust);
            k2_quat  = obj.funcQuatDot(quat2, omg2);
            k2_omega = obj.funcOmegaDot(obj.inertia, omg2, obj.moment);

            %------------- k3 ------------------------------------------------
            % pos3 = obj.pos + h*k2_pos/2;
            vel3 = obj.vel + h*k2_vel./2;
            quat3 = obj.quat + h*k2_quat./2;
            quat3 = quat3./norm(quat3);
            omg3 = obj.omg + h*k2_omega./2;

            k3_pos = obj.funcPosDot(vel3);
            k3_vel = obj.funcVelDot(vel3, obj.thrust);
            k3_quat = obj.funcQuatDot(quat3, omg3);
            k3_omega = obj.funcOmegaDot(obj.inertia, omg3, obj.moment);

            %------------- k4 ------------------------------------------------
            % pos4 = obj.pos + h*k3_pos;
            vel4 = obj.vel + h*k3_vel;
            quat4 = obj.quat + h*k3_quat;
            quat4 = quat4./norm(quat4);
            omg4 = obj.omg + h*k3_omega;

            k4_pos = obj.funcPosDot(vel4);
            k4_vel = obj.funcVelDot(vel4, obj.thrust);
            k4_quat = obj.funcQuatDot(quat4, omg4);
            k4_omega = obj.funcOmegaDot(obj.inertia, omg4, obj.moment);

            %------------- state update -------------------------------------
            obj.pos   = obj.pos   + (h/6) * (k1_pos   + 2*k2_pos   + 2*k3_pos   + k4_pos  );
            obj.vel   = obj.vel   + (h/6) * (k1_vel   + 2*k2_vel   + 2*k3_vel   + k4_vel  );
            obj.quat  = obj.quat  + (h/6) * (k1_quat  + 2*k2_quat  + 2*k3_quat  + k4_quat );
            obj.omg   = obj.omg   + (h/6) * (k1_omega + 2*k2_omega + 2*k3_omega + k4_omega);
            
            % Numerical error accumulation prevention by quaternion normalization
            obj.quat = obj.quat / norm(obj.quat);
            obj.att = obj.quatToEuler(obj.quat);
            while obj.att(3) > pi
                obj.att(3) = obj.att(3) - 2*pi;
            end
            while obj.att(3) < -pi
                obj.att(3) = obj.att(3) + 2*pi;
            end
            [obj.posF, obj.posL, obj.posU] = deal(obj.pos(1), obj.pos(2), obj.pos(3));
            [obj.velF, obj.velL, obj.velU] = deal(obj.vel(1), obj.vel(2), obj.vel(3));
            [obj.e0, obj.ex, obj.ey, obj.ez] = deal(obj.quat(1), obj.quat(2), obj.quat(3), obj.quat(4));
            [obj.rol, obj.pit, obj.yaw] = deal(obj.att(1), obj.att(2), obj.att(3));
            [obj.p, obj.q, obj.r] = deal(obj.omg(1), obj.omg(2), obj.omg(3));
        end

        function obj = stateLogger(obj)
            obj.loggerPos(:, obj.loggerIndex) = obj.pos;
            obj.loggerVel(:, obj.loggerIndex) = obj.vel;
            obj.loggerQuat(:, obj.loggerIndex) = obj.quat;
            obj.loggerAtt(:, obj.loggerIndex) = obj.quatToEuler(obj.quat);
            obj.loggerOmg(:, obj.loggerIndex) = obj.omg;
            obj.loggerInput(:, obj.loggerIndex) = obj.u;
            obj.loggerIndex = obj.loggerIndex + 1;
        end
            
        function obj = applyControlStep(obj)
            [obj.thrust, obj.moment] = deal(obj.u(1), obj.u(2:end));
            obj.equationOfMotion();
            obj.rk4Step();
            obj.time = obj.time + obj.dt;
            obj.stateLogger();
        end
    end
end

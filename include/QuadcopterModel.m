classdef QuadcopterModel < handle
  % QuadcopterModel: A class to simulate the dynamics and control of a quadcopter.
  % This class includes state propagation using Euler integration as well as PID
  % controllers for various aspects such as attitude, angular rates, and velocity.
  % It operates in the NED coordinate frame.

  properties  % Member Variables
      g                       % Gravitational constant (9.81 m/s^2)
      time                    % Simulation time (seconds)
      dt                      % Time step for numerical integration

      m                       % Mass of the quadcopter (kg)
      armlenght               % Arm length (distance from the center to each motor) (m)
      Inertia                 % Inertia matrix of the quadcopter (kg·m^2)

      x                       % Full state vector: [x; y; z; dx; dy; dz; phi; theta; psi; p; q; r]
      r                       % Position vector extracted from x ([x; y; z])
      dr                      % Velocity vector extracted from x ([dx; dy; dz])
      euler                   % Euler angles extracted from x ([phi (roll); theta (pitch); psi (yaw)])
      w                       % Angular velocities extracted from x ([p; q; r])

      dx                      % Time derivative of the state vector (used for integration)

      u                       % Control input vector: [T; M1; M2; M3] (thrust and moments)
      T                       % Total thrust (scalar)
      M                       % Moments vector: [M1; M2; M3]

      % Desired and error variables for angular rate control
      p_des                   % Desired roll rate
      p_err                   % Current roll rate error
      p_err_prev              % Previous roll rate error
      p_err_sum               % Sum (integral) of roll rate error

      q_des                   % Desired pitch rate
      q_err                   % Current pitch rate error
      q_err_prev              % Previous pitch rate error
      q_err_sum               % Sum (integral) of pitch rate error

      r_des                   % Desired yaw rate
      r_err                   % Current yaw rate error
      r_err_prev              % Previous yaw rate error
      r_err_sum               % Sum (integral) of yaw rate error

      % Desired and error variables for Euler angle control
      phi_des                 % Desired roll angle (phi)
      phi_err                 % Roll angle error
      phi_err_prev            % Previous roll angle error
      phi_err_sum             % Integral (sum) of roll angle error

      theta_des               % Desired pitch angle (theta)
      theta_err               % Pitch angle error
      theta_err_prev          % Previous pitch angle error
      theta_err_sum           % Integral (sum) of pitch angle error

      psi_des                 % Desired yaw angle (psi)
      psi_err                 % Yaw angle error
      psi_err_prev            % Previous yaw angle error
      psi_err_sum             % Integral (sum) of yaw angle error

      % Desired and error variables for velocity control
      xdot_des                % Desired x velocity
      xdot_err                % x velocity error
      xdot_err_prev           % Previous x velocity error
      xdot_err_sum            % Integral (sum) of x velocity error

      ydot_des                % Desired y velocity
      ydot_err                % y velocity error
      ydot_err_prev           % Previous y velocity error
      ydot_err_sum            % Integral (sum) of y velocity error

      zdot_des                % Desired z velocity
      zdot_err                % z velocity error
      zdot_err_prev           % Previous z velocity error
      zdot_err_sum            % Integral (sum) of z velocity error

      % PID control gains for angular rate control (p, q, r)
      P_p, I_p, D_p           % PID gains for roll rate (p)
      P_q, I_q, D_q           % PID gains for pitch rate (q)
      P_r, I_r, D_r           % PID gains for yaw rate (r)

      % PID control gains for Euler angle control (phi, theta, psi)
      P_phi, I_phi, D_phi      % PID gains for roll angle (phi)
      P_theta, I_theta, D_theta% PID gains for pitch angle (theta)
      P_psi, I_psi, D_psi      % PID gains for yaw angle (psi)

      % PID control gains for velocity control (xdot, ydot, zdot)
      P_xdot, I_xdot, D_xdot   % PID gains for x velocity
      P_ydot, I_ydot, D_ydot   % PID gains for y velocity
      P_zdot, I_zdot, D_zdot   % PID gains for z velocity
  end

  methods  % Member Methods
      function obj = QuadcopterModel(initStates, initInputs)
          % Constructor: Initializes the quadcopter model with the given states and control inputs.
          % The physical parameters are set using a containers.Map.

          % Create a map of physical parameters for the quadcopter
          params = containers.Map({'Mass', 'armLength', 'Ixx', 'Iyy', 'Izz'}, ...
              {2, 0.25, 0.021667, 0.021667, 0.04});

          obj.g = 9.81;          % Gravitational constant (m/s^2)
          obj.time = 0.0;        % Initial simulation time
          obj.dt = 0.001;        % Time step for simulation (s)

          % Set physical parameters
          obj.m = params('Mass');
          obj.armlenght = params('armLength');
          obj.Inertia = diag([params('Ixx'), params('Iyy'), params('Izz')]);  % Inertia matrix

          % Initialize state vector and extract sub-components
          obj.x = initStates;      % Full state vector
          obj.r = obj.x(1:3);      % Position vector (x, y, z)
          obj.dr = obj.x(4:6);     % Velocity vector (dx, dy, dz)
          obj.euler = obj.x(7:9);  % Euler angles (phi, theta, psi)
          obj.w = obj.x(10:12);    % Angular velocities (p, q, r)
          obj.dx = zeros(12, 1);   % Initialize state derivative vector

          % Set initial control inputs and extract thrust and moments
          obj.u = initInputs;      % Control input vector [T; M1; M2; M3]
          obj.T = obj.u(1);        % Total thrust
          obj.M = obj.u(2:4);      % Moments (M1, M2, M3)

          % Initialize desired states and error accumulators (all set to zero).
          obj.p_des = 0.0; obj.p_err = 0.0; obj.p_err_sum = 0.0;
          obj.q_des = 0.0; obj.q_err = 0.0; obj.q_err_sum = 0.0;
          obj.r_des = 0.0; obj.r_err = 0.0; obj.r_err_sum = 0.0;
          obj.phi_des = 0.0; obj.phi_err = 0.0; obj.phi_err_prev = 0.0; obj.phi_err_sum = 0.0;
          obj.theta_des = 0.0; obj.theta_err = 0.0; obj.theta_err_prev = 0.0; obj.theta_err_sum = 0.0;
          obj.psi_des = 0.0; obj.psi_err = 0.0; obj.psi_err_prev = 0.0; obj.psi_err_sum = 0.0;
          obj.xdot_des = 0.0; obj.xdot_err = 0.0; obj.xdot_err_prev = 0.0; obj.xdot_err_sum = 0.0;
          obj.ydot_des = 0.0; obj.ydot_err = 0.0; obj.ydot_err_prev = 0.0; obj.ydot_err_sum = 0.0;
          obj.zdot_des = 0.0; obj.zdot_err = 0.0; obj.zdot_err_prev = 0.0; obj.zdot_err_sum = 0.0;

          % Set PID gains using a containers.Map for clarity.
          gains = containers.Map(...
              {'P_p', 'I_p', 'D_p', ...
               'P_q', 'I_q', 'D_q', ...
               'P_r', 'I_r', 'D_r', ...
               'P_phi', 'I_phi', 'D_phi', ...
               'P_theta', 'I_theta', 'D_theta', ...
               'P_psi', 'I_psi', 'D_psi', ...
               'P_xdot', 'I_xdot', 'D_xdot', ...
               'P_ydot', 'I_ydot', 'D_ydot', ...
               'P_zdot', 'I_zdot', 'D_zdot'}, ...
              {10, 0.2, 0.1, ...
               10, 0.2, 0.1, ...
               10, 0.2, 0.1, ...
               10, 0.0, 0.00005, ...
               10, 0.0, 0.00005, ...
               10, 0.0, 0.00005, ...
               25, 0.001, 0.005, ...
               25, 0.001, 0.005, ...
               25, 0.001, 0.005});

          % Initialize the PID gains for angular rates, Euler angles, and velocities.
          obj.P_p = gains('P_p'); obj.I_p = gains('I_p'); obj.D_p = gains('D_p');
          obj.P_q = gains('P_q'); obj.I_q = gains('I_q'); obj.D_q = gains('D_q');
          obj.P_r = gains('P_r'); obj.I_r = gains('I_r'); obj.D_r = gains('D_r');

          obj.P_phi = gains('P_phi'); obj.I_phi = gains('I_phi'); obj.D_phi = gains('D_phi');
          obj.P_theta = gains('P_theta'); obj.I_theta = gains('I_theta'); obj.D_theta = gains('D_theta');
          obj.P_psi = gains('P_psi'); obj.I_psi = gains('I_psi'); obj.D_psi = gains('D_psi');

          obj.P_xdot = gains('P_xdot'); obj.I_xdot = gains('I_xdot'); obj.D_xdot = gains('D_xdot');
          obj.P_ydot = gains('P_ydot'); obj.I_ydot = gains('I_ydot'); obj.D_ydot = gains('D_ydot');
          obj.P_zdot = gains('P_zdot'); obj.I_zdot = gains('I_zdot'); obj.D_zdot = gains('D_zdot');
      end

      function state = getState(obj)
          % getState: Returns the current state vector of the quadcopter.
          % Output:
          %   state - The current state vector [x; y; z; dx; dy; dz; phi; theta; psi; p; q; r]
          state = obj.x;
      end

      function command = getCommand(obj)
          % getCommand: Constructs and returns the desired control command vector.
          % The command includes desired velocities, Euler angles, and angular rates.
          % Output:
          %   command - A 9x1 vector: [xdot_des; ydot_des; zdot_des; phi_des; theta_des; psi_des; p_des; q_des; r_des]
          command = [obj.xdot_des; obj.ydot_des; obj.zdot_des; ...
                     obj.phi_des; obj.theta_des; obj.psi_des; ...
                     obj.p_des; obj.q_des; obj.r_des];
      end

      function input = getControlInput(obj)
          % getControlInput: Returns the current control input vector.
          % Output:
          %   input - A 4x1 vector [T; M1; M2; M3]
          input = obj.u;
      end

      function obj = EvalEOM(obj)
          % EvalEOM: Evaluates the Equations of Motion (EOM) for the quadcopter.
          %
          % This method computes the derivatives of the state vector based on the
          % current state and control inputs. It uses a rotation matrix to transform
          % the thrust from the body frame to the inertial frame.

          % Extract current Euler angles
          phi = obj.euler(1);    % Roll angle
          theta = obj.euler(2);  % Pitch angle
          psi = obj.euler(3);    % Yaw angle

          % Precompute trigonometric values for efficiency
          cos_phi = cos(phi);     sin_phi = sin(phi);
          cos_theta = cos(theta); sin_theta = sin(theta);
          cos_psi = cos(psi);     sin_psi = sin(psi);

          % Construct the rotation matrix from the body frame to the inertial frame
          % The rotation matrix is the transpose of the body-to-inertial conversion matrix.
          R = [cos_theta * cos_psi, sin_phi * sin_theta * cos_psi - cos_phi * sin_psi, cos_phi * sin_theta * cos_psi + sin_phi * sin_psi;
               cos_theta * sin_psi, sin_phi * sin_theta * sin_psi + cos_phi * cos_psi, cos_phi * sin_theta * sin_psi - sin_phi * cos_psi;
               -sin_theta,          sin_phi * cos_theta,                             cos_phi * cos_theta];

          % Update the state derivatives based on the current velocities and forces.
          obj.dx(1:3) = obj.dr;  % Position derivative (equals current velocity)

          % Compute the inertial thrust force vector.
          % Note: The thrust is applied in the body z-axis (negative in inertial frame).
          thrust_force = R * [0; 0; -obj.T];
          % Update velocity derivatives: include gravity plus the effect of thrust.
          obj.dx(4:6) = [0; 0; obj.g] + thrust_force ./ obj.m;

          % Calculate Euler angle derivatives using the transformation matrix for angular velocities.
          tan_theta = tan(theta);
          sec_theta = sec(theta);
          obj.dx(7:9) = [1, sin_phi * tan_theta, cos_phi * tan_theta;
                         0, cos_phi,            -sin_phi;
                         0, sin_phi * sec_theta, cos_phi * sec_theta] * obj.w;

          % Update angular velocity derivatives using Euler's equations:
          % The angular acceleration is computed as the inverse inertia matrix times
          % (applied moments minus the gyroscopic effect).
          obj.dx(10:12) = obj.Inertia \ (obj.M - cross(obj.w, obj.Inertia * obj.w));
      end

      function obj = UpdateState(obj)
          % UpdateState: Updates the quadcopter's state by integrating the state derivatives.
          %
          % This method increments the simulation time and uses Euler integration to update
          % the state vector. It also updates the extracted state components.

          obj.time = obj.time + obj.dt;     % Increment simulation time
          obj.EvalEOM();                     % Evaluate the equations of motion
          obj.x = obj.x + obj.dx * obj.dt;    % Update the state vector using Euler integration

          % Update state components from the full state vector
          obj.r = obj.x(1:3);                % Update position
          obj.dr = obj.x(4:6);               % Update velocity

          % Ensure the yaw angle remains within the range [-pi, pi] for consistency.
          if obj.x(9) * (180/pi) > 180
              obj.x(9) = obj.x(9) - 2*pi;
          elseif obj.x(9) * (180/pi) < -180
              obj.x(9) = obj.x(9) + 2*pi;
          end
          obj.euler = obj.x(7:9);            % Update Euler angles
          obj.w = obj.x(10:12);              % Update angular velocities
      end

      function obj = angleSaturation(obj)
          % angleSaturation: Limits the desired Euler angles to a specified maximum value.
          %
          % This method saturates the desired roll and pitch angles to a maximum of 30 degrees.
          % It also wraps the desired yaw angle to within [-pi, pi].

          sat_angle = deg2rad(30);  % Maximum allowable angle (30 degrees in radians)

          % Saturate desired pitch (theta)
          if obj.theta_des > sat_angle
              obj.theta_des = sat_angle;
          elseif obj.theta_des < -sat_angle
              obj.theta_des = -sat_angle;
          end

          % Saturate desired roll (phi)
          if obj.phi_des > sat_angle
              obj.phi_des = sat_angle;
          elseif obj.phi_des < -sat_angle
              obj.phi_des = -sat_angle;
          end

          % Wrap desired yaw (psi) to within [-pi, pi]
          while obj.psi_des > pi
              obj.psi_des = obj.psi_des - 2 * pi;
          end
          while obj.psi_des < -pi
              obj.psi_des = obj.psi_des + 2 * pi;
          end
      end

      function obj = rateSaturation(obj)
          % rateSaturation: Saturates the desired angular rates (roll, pitch, yaw) to a maximum value.
          %
          % This method ensures that the desired angular rates do not exceed 60 degrees per second.

          sat_rate = deg2rad(60);  % Maximum angular rate (60 deg/s in radians per second)

          % Saturate desired roll rate (p)
          if obj.p_des > sat_rate
              obj.p_des = sat_rate;
          elseif obj.p_des < -sat_rate
              obj.p_des = -sat_rate;
          end

          % Saturate desired pitch rate (q)
          if obj.q_des > sat_rate
              obj.q_des = sat_rate;
          elseif obj.q_des < -sat_rate
              obj.q_des = -sat_rate;
          end

          % Saturate desired yaw rate (r)
          if obj.r_des > sat_rate
              obj.r_des = sat_rate;
          elseif obj.r_des < -sat_rate
              obj.r_des = -sat_rate;
          end
      end

      function obj = rateControl(obj)
          % rateControl: Implements a PID controller for angular rate control.
          %
          % This method computes control commands for roll, pitch, and yaw rates using
          % the current errors (difference between desired and actual rates) and updates the control input.

          % Compute roll rate error and update control input for roll rate.
          obj.p_err = obj.p_des - obj.w(1);
          obj.u(2) = obj.P_p * obj.p_err + obj.I_p * obj.p_err_sum - obj.D_p * obj.w(1);
          obj.p_err_sum = obj.p_err_sum + obj.p_err * obj.dt;

          % Compute pitch rate error and update control input for pitch rate.
          obj.q_err = obj.q_des - obj.w(2);
          obj.u(3) = obj.P_q * obj.q_err + obj.I_q * obj.q_err_sum - obj.D_q * obj.w(2);
          obj.q_err_sum = obj.q_err_sum + obj.q_err * obj.dt;

          % Compute yaw rate error and update control input for yaw rate.
          obj.r_err = obj.r_des - obj.w(3);
          obj.u(4) = obj.P_r * obj.r_err + obj.I_r * obj.r_err_sum - obj.D_r * obj.w(3);
          obj.r_err_sum = obj.r_err_sum + obj.r_err * obj.dt;

          % Update moments vector with the computed control inputs.
          obj.M = obj.u(2:4);
      end

      function obj = attitudeControl(obj)
          % attitudeControl: Implements a PID controller for attitude (Euler angle) control.
          %
          % This method calculates the desired angular rates from the errors between desired and
          % current Euler angles (roll, pitch, yaw) and then updates these errors.

          % Use an extended time step for attitude control (4 times the simulation dt)
          att_dt = obj.dt * 4;

          % Roll control: update desired roll rate based on roll error.
          obj.phi_err = obj.phi_des - obj.euler(1);
          obj.p_des = obj.P_phi * obj.phi_err + obj.I_phi * obj.phi_err_sum + obj.D_phi * ((obj.phi_err - obj.phi_err_prev) / att_dt);
          obj.phi_err_prev = obj.phi_err;
          obj.phi_err_sum = obj.phi_err_sum + obj.phi_err * att_dt;

          % Pitch control: update desired pitch rate based on pitch error.
          obj.theta_err = obj.theta_des - obj.euler(2);
          obj.q_des = obj.P_theta * obj.theta_err + obj.I_theta * obj.theta_err_sum + obj.D_theta * ((obj.theta_err - obj.theta_err_prev) / att_dt);
          obj.theta_err_prev = obj.theta_err;
          obj.theta_err_sum = obj.theta_err_sum + obj.theta_err * att_dt;

          % Yaw control: compute desired yaw from velocity direction and update desired yaw rate.
          obj.psi_des = atan2(obj.ydot_des, obj.xdot_des);
          obj.psi_err = obj.psi_des - obj.euler(3);
          obj.r_des = obj.P_psi * obj.psi_err + obj.I_psi * obj.psi_err_sum + obj.D_psi * ((obj.psi_err - obj.psi_err_prev) / att_dt);
          obj.psi_err_prev = obj.psi_err;
          obj.psi_err_sum = obj.psi_err_sum + obj.psi_err * att_dt;

          % Saturate the desired angular rates to avoid excessive commands.
          obj.rateSaturation();
      end

      function obj = guidanceControl(obj, refSig)
          % guidanceControl: Implements a PID controller for position (velocity) control.
          %
          % This method takes a reference velocity signal (refSig) and calculates the error
          % between desired and current velocities. It then computes desired Euler angles and
          % thrust commands to steer the UAV along the desired trajectory.

          % Compute the rotation matrix (body-to-inertial transformation) based on Euler angles.
          bRi = eul2rotm([-obj.euler(3), -obj.euler(2), -obj.euler(1)], 'ZYX');
          guid_dt = obj.dt * 20;  % Time step for guidance control (scaled factor)

          % Set desired velocities from the reference signal.
          obj.xdot_des = refSig(1);
          obj.ydot_des = refSig(2);
          obj.zdot_des = refSig(3);

          % Calculate velocity errors in the inertial frame.
          obj.xdot_err = obj.xdot_des - obj.dr(1);
          obj.ydot_err = obj.ydot_des - obj.dr(2);
          obj.zdot_err = obj.zdot_des - obj.dr(3);

          % Transform the velocity errors into the body frame.
          bRi = bRi * [obj.xdot_err; obj.ydot_err; obj.zdot_err];
          obj.xdot_err = bRi(1);
          obj.ydot_err = bRi(2);
          obj.zdot_err = bRi(3);

          % Compute desired pitch (theta) based on x velocity error.
          obj.theta_des = -(obj.P_xdot * obj.xdot_err + obj.I_xdot * obj.xdot_err_sum + obj.D_xdot * ((obj.xdot_err - obj.xdot_err_prev) / guid_dt)) * (pi/180);
          obj.xdot_err_prev = obj.xdot_err;
          obj.xdot_err_sum = obj.xdot_err * obj.dt * 20 + obj.xdot_err_sum;

          % Compute desired roll (phi) based on y velocity error.
          obj.phi_des = (obj.P_ydot * obj.ydot_err + obj.I_ydot * obj.ydot_err_sum + obj.D_ydot * ((obj.ydot_err - obj.ydot_err_prev) / guid_dt)) * (pi/180);
          obj.ydot_err_prev = obj.ydot_err;
          obj.ydot_err_sum = obj.ydot_err * obj.dt * 20 + obj.ydot_err_sum;

          % Compute desired thrust to regulate the z velocity.
          obj.u(1) = obj.m * obj.g - (obj.P_zdot * obj.zdot_err + obj.I_zdot * obj.zdot_err_sum + obj.D_zdot * ((obj.zdot_err - obj.zdot_err_prev) / obj.dt));
          obj.zdot_err_prev = obj.zdot_err;
          obj.zdot_err_sum = obj.zdot_err_sum + obj.zdot_err * obj.dt * 20;

          % Ensure thrust is non-negative.
          if obj.u(1) < 0
              obj.u(1) = 0;
          end
          % Update total thrust.
          obj.T = obj.u(1);

          % Saturate Euler angles to prevent excessive commands.
          obj.angleSaturation();
      end
  end
end

close all; clear all; clc;
addpath("include\");
addpath("lib\");

%% Initialize simulation settings
% Define the initial state vector: [pn, pe, pd, vn, ve, vd, phi, theta, psi, p, q, r]
drone_initStates = [0, 0, -10,...       % Position: pn, pe, pd
                    0, 3, 0,...         % Velocity: vn, ve, vd
                    0, 0, 1.5708,...    % Euler angles: phi, theta, psi
                    0, 0, 0]';          % Angular rates: p, q, r
drone_initInputs = [19.62, 0, 0, 0]';   % Define the initial control inputs: [T, M1, M2, M3] (Thrust and Moments)

% Create a QuadcopterModel object with the initial states and inputs
egoUAV = QuadcopterModel(drone_initStates, drone_initInputs);

% Set the simulation time step and initial time
ts = egoUAV.dt;
time = 0;                       % Initialize simulation time to zero
egoGoalpoint = [0, 50, -10]';   % Define the goal (target) position for the quadcopter
egoSpeed = 3;                   % Desired speed for the quadcopter (m/s)

%% Intruder Initialization
% Define the initial state for the intruder (position, velocity, acceleration)
Int.Position = [-22, 35, -10]';
Int.Velocity = [2.304, -1.2, 0.0]';
Int.Acceleration = [0, 0, 0]';

%% Variable Initialization for Logging and Control
R_safe = 3;                 % Set the safety radius (3 m)
d_thres = 0.5;              % Set the threshold distance for waypoint arrival
aiming_point = [];          % Aiming (target) point
prev_waypoint = drone_initStates(1:3);  % Initialize previous waypoint with initial position
next_waypoint = egoGoalpoint;           % Initially set next waypoint as goal point
egoCPA = [];                % Ego's CPA (Closest Point of Approach)
intruderCPA = [];           % Intruder's CPA
ego_state_log = [];         % Log for full quadcopter state (position, velocity, Euler angles, angular rates)
ego_command_log = [];       % Log for command output (desired control values)
intruder_state_log = [];    % Log for intruder's state
intruder_estimate_log = []; % Log for estimated intruder state (e.g., via Kalman filter)
ellipse_log = [];           % Log for collision threat ellipse data
egoCPA_log = [];            % Log for egoCPA values
aiming_point_log = [];      % Log for aiming points
control_input_log = [];     % Log for control inputs
CPA_data = [];              % Array to store CPA data over several iterations
effectiveCPA = [];          % Effective CPA value
idx_log = [];               % Log of index flags or state conditions
index = false;              % Flag for a certain condition status

first_detect = true;        % Flag to indicate the first detection

collision_threat_log = [];  % Log for collision threat values
collision_threat = zeros([3, 1]);  % Collision threat vector initialized to zero
collision_std_log = [];     % Log for collision standard deviation data
collision_std = zeros([3, 1]);

point_arrived = false;      % Flag indicating if the waypoint has been reached
spanAngle = pi/36;          % Angular span for generating candidate cones
varphi = spanAngle : spanAngle : 2*pi;  % Array of candidate angles for aiming
escapeGainKe = 0.8;         % Weighting factor for escape command

% Initialize reference signal for guidance (desired velocity)
refVel          = [0; 3; 0];    % Initial reference velocity input (3 m/s in y-direction)
refVelUpdate    = false;
prev_refSig     = refVel;       % Store previous reference signal for smoothing
refAtt          = [];
refAttUpdate    = false;
sig_k           = 0.985;        % Smoothing gain (low-pass filtering effect)
data_size       = 20;           % Maximum size of stored CPA data

candidates_log = zeros([3, 72, 15000]); % Log for candidate aiming directions (3 x 72 x max sample size)

% Set up modular control cycle indices
successive_idx  = 0;    % Loop iteration counter for closed-loop control
guidance_ = 20;         % Guidance control module period (executed at 50Hz)
attitude_ = 4;          % Attitude control module period (executed at 250Hz)

while and(time <= 60, norm(egoUAV.r - egoGoalpoint) > d_thres)
    % Update Intruder's State (Constant Velocity Assumption)
    Int.Position = Int.Position + ts * Int.Velocity + (ts^2 * Int.Acceleration)/2;
    Int.Velocity = Int.Velocity + ts * Int.Acceleration;
    intruderMotion = [Int.Position; Int.Velocity; Int.Acceleration];
    
    % Reset collision metrics for this iteration
    collision_std = zeros([3, 1]);
    collision_threat = zeros([3, 1]);
    index = false;

    if mod(successive_idx, guidance_) == 0
        refVel = egoSpeed*(egoGoalpoint - egoUAV.r)./norm(egoGoalpoint - egoUAV.r);
        egoUAV.guidanceControl(refVel);
        isDangerous = false;
        
        if norm(egoUAV.r - Int.Position) < 20
            [EstIntruderMotion, measured] = kinetic_kalman(intruderMotion, guidance_*ts);
            [rMin, tMin, radInfo, velInfo, flag] = isCollision(egoUAV.x, EstIntruderMotion, R_safe);
            if flag
                flag = false;
                refAttUpdate = true;
                refDelta = pi/3;
                refGamma = pi/4;
                gainK = 7;
                aA = refAcc(egoUAV.x, EstIntruderMotion, radInfo, velInfo, gainK, R_safe, refDelta, refGamma);
            else
                refAttUpdate = false;
            end
        end
    end

    if mod(successive_idx, attitude_) == 0
        if flag
            egoUAV.phi_des = asin(aA(1)/(egoUAV.T/egoUAV.m));
            egoUAV.theta_des = acos(aA(2)*(egoUAV.m/(egoUAV.T*cos(egoUAV.phi_des))));
            egoUAV.psi_des = pi/2;
            egoUAV.angleSaturation();
        end
        egoUAV.attitudeControl();
    end
    egoUAV.rateControl();
    egoUAV.UpdateState();
end













% [initCond, initInput, params, simTime] = initSimul();
% egoUAV = MultiCopter(initCond, initInput, params, simTime);
% 
% [simulTime, velTimes, attTimes, omgTimes] = initRatesPID();
% thrustGain = initGainsPID([25; 0.001; 0.005]);
% velGainPID = initGainsPID([25; 0.001; 0.005]);    % m/s in FLU coordinate
% attGainPID = initGainsPID([10; 0; 0.00005]);      % Euler angle
% omgGainPID = initGainsPID([10; 0.2; 0.1]);
% rateSaturation = deg2rad(60);
% attSaturation = deg2rad(30);
% 
% [ioThrust, ioHeading] = deal(1, 1);
% ioAtt = 2;
% ioSize = 3;
% 
% maxThrust = initInput(1)*2;
% minThrust = 0;
% thrustCtrl = ThrustPID("thrustCtrl", thrustGain, velTimes, ioThrust, initInput(1));
% thrustCtrl.setOutputSaturation(minThrust, maxThrust);
% 
% headingCtrl = HeadingPID("headingCtrl", attGainPID, velTimes, ioHeading);
% headingCtrl.setOutputSaturation(-rateSaturation, rateSaturation);
% 
% velCtrl = GuidancePID("velCtrl", velGainPID, velTimes, ioSize);
% velCtrl.setOutputSaturation(-attSaturation, attSaturation);
% 
% attCtrl = ControllerPID("attCtrl", attGainPID, attTimes, ioSize);
% attCtrl.setOutputSaturation(-rateSaturation, rateSaturation);
% 
% omgCtrl = RatePID("omgCtrl", omgGainPID, omgTimes, ioSize);
% 
% time = 0;
% 
% % loop indexes
% loopIdx = 0;
% loopVel = omgTimes.rate/velTimes.rate;
% loopAtt = omgTimes.rate/attTimes.rate;
% 
% distThreshold = 0.5;    % distance threshold
% 
% goalPoint = [50, 0, 10];
% 
% % Initial reference command
% ref = struct( ...
%     "velocity", [2; 2; 0],...
%     "attitude", zeros([3,1]),...
%     "omega", zeros([3,1]));
% 
% %% Main simulation loops
% while and(time <= simulTime, norm(goalPoint-egoUAV.pos) > distThreshold)
% 
%     if mod(loopIdx, loopVel) == 0
%         egoUAV.u(1) = thrustCtrl.update(ref.velocity(3), egoUAV.vel(3));
%         velCtrl.getAtt(egoUAV.att);
%         ref.attitude = velCtrl.update(ref.velocity, egoUAV.vel);
%         ref.attitude(3) = headingCtrl.update(atan2(ref.velocity(2), ref.velocity(1)), egoUAV.yaw);
%     end
% 
%     if mod(loopIdx, loopAtt) == 0
%         ref.omega = attCtrl.update(ref.attitude, egoUAV.att);
%     end
% 
%     omgCtrl.getRate(egoUAV.omg);
%     egoUAV.moment = omgCtrl.update(ref.omega, egoUAV.omg);
%     [egoUAV.u(2), egoUAV.u(3), egoUAV.u(4)] = deal(egoUAV.moment(1), egoUAV.moment(2), egoUAV.moment(3));
%     egoUAV.applyControlStep();
%     loopIdx = loopIdx + 1;
%     time = time + egoUAV.dt;
% end
% %% Save logs to files
% folderPath = fullfile(pwd, 'log', 'simul');
% if ~exist(folderPath, 'dir')
%     mkdir(folderPath);
% end
% save(fullfile(folderPath, "SimComparison.mat"), "attCtrl",...
%     "egoUAV", "headingCtrl", "omgCtrl", "thrustCtrl", "velCtrl");

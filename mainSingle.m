close all; clear all; clc;
addpath("include");
addpath("lib");

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

ego_state_log = [];         % Log for full quadcopter state (position, velocity, Euler angles, angular rates)
ego_command_log = [];       % Log for command output (desired control values)
intruder_state_log = [];    % Log for intruder's state
control_input_log = [];     % Log for control inputs
rMin_log = [];
rMin = NaN;
aA = NaN([3, 1]);
aA_log = [];
err = NaN;
err_log = [];

% Initialize reference signal for guidance (desired velocity)
refVel = [0; 3; 0];    % Initial reference velocity input (3 m/s in y-direction)
prev_refVel = refVel;       % Store previous reference signal for smoothing
refAtt = [];
sig_k = 0.985;        % Smoothing gain (low-pass filtering effect)
gainK = NaN;

% Set up modular control cycle indices
successive_idx  = 0;    % Loop iteration counter for closed-loop control
guidance_ = 20;         % Guidance control module period (executed at 50Hz)
attitude_ = 4;          % Attitude control module period (executed at 250Hz)

while and(time <= 60, norm(egoUAV.r - egoGoalpoint) > d_thres)
    % Update Intruder's State (Constant Velocity Assumption)
    Int.Position = Int.Position + ts * Int.Velocity + (ts^2 * Int.Acceleration)/2;
    Int.Velocity = Int.Velocity + ts * Int.Acceleration;
    intruderMotion = [Int.Position; Int.Velocity; Int.Acceleration];
    
    if mod(successive_idx, guidance_) == 0
        refVel = egoSpeed*(egoGoalpoint - egoUAV.r)./norm(egoGoalpoint - egoUAV.r);
        egoUAV.guidanceControl(refVel);        
        if norm(egoUAV.r - Int.Position) < 20
            [EstIntruderMotion, measured] = kineticKalman(intruderMotion, guidance_*ts);
            [rMin, tMin, radInfo, velInfo, flag] = isCollision(egoUAV.x, EstIntruderMotion, R_safe);
            if flag
                refDelta = pi/3;
                refGamma = pi/4;
                err = errFcn(egoUAV.x, EstIntruderMotion, velInfo, R_safe);
                if isnan(gainK)
                    epsilon = err - 0.1;
                    gainK = (1/tMin)*log(err/epsilon);
                end
                aA = refAcc(egoUAV.x, EstIntruderMotion, radInfo, velInfo, gainK, R_safe, refDelta, refGamma);
            else
                gainK = NaN;
            end
        end
    end

    if mod(successive_idx, attitude_) == 0
        if flag
            temp = aA(1)/(egoUAV.T/egoUAV.m);
            temp = max(-1,min(1,temp));
            egoUAV.phi_des = asin(temp);
            temp = aA(2)*(egoUAV.m/(egoUAV.T*cos(egoUAV.phi_des)));
            temp = max(-1,min(1,temp));
            egoUAV.theta_des = acos(temp);
            egoUAV.psi_des = pi/2;
            egoUAV.angleSaturation();
        end
        egoUAV.attitudeControl();
    end
    egoUAV.rateControl();
    egoUAV.UpdateState();
    successive_idx = successive_idx + 1;
    time = time + egoUAV.dt;

    refVel = prev_refVel*sig_k + (1 - sig_k)*refVel;

    ego_state_log = [ego_state_log, egoUAV.x];
    intruder_state_log = [intruder_state_log, intruderMotion];
    ego_command_log = [ego_command_log, egoUAV.getCommand()];
    control_input_log = [control_input_log, egoUAV.getControlInput()];
    rMin_log = [rMin_log, rMin];
    aA_log = [aA_log, aA];
    err_log = [err_log, err];
end

folderPath = fullfile(pwd, 'log', 'single');

% Create output folder if it does not exist
if ~exist(folderPath, 'dir')
    mkdir(folderPath);
end

% Save all main log variables to a MAT file using grouped save commands
save(fullfile(folderPath, "SimSingle.mat"), "ego_state_log", ...
    "intruder_state_log", "ego_command_log", "control_input_log", ...
    "rMin_log", "aA_log", "err_log");


clc; close all; clear all;
rng(0);
addpath("lib");
addpath("include");
load("cases/IntruderRandomPoints.mat");
load("cases/RandomCollisionPoints.mat");
load("cases/IntruderVeloicty.mat");

egoGoalpoint = [0, 50, -10]';
egoSpeed = 3;
R_safe = 3;
ke = 0.8;
minMissDist = [];
trajectoryDeviation = [];
maxDistPath = [];
arrivalTime = [];

d_thres = 0.5;

sig_k = 0.985;
guidance_       = 20;               % guidance index for modular operation 50Hz
attitude_       = 4;                % attitude index for modular operation 250Hz

dq = parallel.pool.DataQueue;
afterEach(dq, @nUpdate);

function nUpdate(~)
persistent progress N
if isempty(progress)
    progress = 0;
    N = 500;
end
progress = progress + 1;
fprintf("Progress: %d / %d (%.1f%%)\n", ...
    progress, N, 100*progress/N);
end

parfor i = 1 : length(IntruderRandomPoints)
    successive_idx  = 0;        % Index for successive closed loop
    ego_state_log = [];         % Log for full quadcopter state (position, velocity, Euler angles, angular rates)
    intruder_state_log = [];    % Log for intruder's state
    rMin = NaN;
    aA = NaN([3, 1]);
    err = NaN;
    CPA_data = [];
    % Quadcopter Initialization
    drone_initStates = [0, 0, -10,...           %   x     y    z
        0, 3, 0,...             %  dx    dy   dz
        0, 0, 1.5708,...        % phi theta  psi
        0, 0, 0]';              %   p     q    r
    drone_initInputs = [19.62, 0, 0, 0]';       % u1, u2, u3, u4 {T, M1, M2, M3}
    egoUAV = QuadcopterModel(drone_initStates, drone_initInputs);
    ts = egoUAV.dt;
    time = 0;

    refVel = [0; 3; 0];
    prev_refVel = refVel;
    refAtt = [];

    intPos = IntruderRandomPoints(:, i);
    intVel = IntruderVeloicty(:, i);
    intAcc = [0; 0; 0];

    ref_update = false;
    isDangerous = false;

    flag = false;
    isFirst = true;
    while and(time <= 60, norm(egoUAV.r - egoGoalpoint) > d_thres)
        % Update Intruder's State (Constant Velocity Assumption)
        intPos = intPos + ts * intVel + (ts^2 * intAcc)/2;
        intVel = intVel + ts * intAcc;
        intruderMotion = [intPos; intVel; intAcc];

        if mod(successive_idx, guidance_) == 0
            refVel = egoSpeed*(egoGoalpoint - egoUAV.r)./norm(egoGoalpoint - egoUAV.r);
            egoUAV.guidanceControl(refVel);
            if norm(egoUAV.r - intPos) < 20
                [EstIntruderMotion, measured] = kineticKalman(intruderMotion, guidance_*ts);
                [isDangerous, t_CPA, egoCPA, intCPA] = isCollision(egoUAV.x, EstIntruderMotion, R_safe);
                if size(CPA_data, 2) > data_size - 1
                    CPA_data(:, 1:end-1) = CPA_data(:, 2:end);
                    CPA_data(:, end) = intCPA;
                else
                    CPA_data = [CPA_data, intCPA];
                end
            end

            if isDangerous
                [scale, rotation] = getScaleData(CPA_data, R_safe);
                [C1, C2] = getFoci(scale, rotation, intCPA, R_safe);
                [rMin, tMin, rI, vI, rM, flag] = isCollisionSpheroid(egoUAV.x, EstIntruderMotion, R_safe, C1, C2);
                err = errFcnSpheroid(vI, scale, R_safe, rM);
                if isnan(gainK)
                    epsilon = err - 0.1;
                    gainK = 1.001 * (1 / tMin) * log(err / epsilon);
                    refDelta = pi/3;
                    refGamma = pi/4;
                end
                aA = refAccSpheroid(err, gainK, rI, vI, rM, refDelta, refGamma);
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
                flag = false;
            end
            egoUAV.attitudeControl();
        end
        egoUAV.rateControl();
        egoUAV.UpdateState();
        successive_idx = successive_idx + 1;
        time = time + egoUAV.dt;

        refVel = prev_refVel*sig_k + (1 - sig_k)*refVel;

        ego_state_log = [ego_state_log, egoUAV.x];
    end
    miss_dist = [];
    traj_deviation = [];
    dist_from_path = [];
    for idx = 1 : length(ego_state_log)
        miss_dist = [miss_dist, norm(ego_state_log(1:3, idx) - intruder_state_log(1:3, idx))];
        traj_deviation = [traj_deviation, (ego_state_log(4:6, idx) - ego_state_log(4:6, 1)).*0.001];
        dist_from_path = [dist_from_path, norm((ego_state_log(1:3, idx)-[0; 0; -10]) - (ego_state_log(1:3, idx)-[0; 0; -10]).*[0; 1; 0])];
    end
    data_deviation = zeros([1, length(traj_deviation)]);

    for idx = 1:length(data_deviation)
        dev_sum = zeros([3, 1]);
        dev_sum(1) = sum(traj_deviation(1,1:idx));
        dev_sum(2) = sum(traj_deviation(2,1:idx));
        dev_sum(3) = sum(traj_deviation(3,1:idx));
        data_deviation(idx) = norm(dev_sum);
    end

    minMissDist = [minMissDist, min(miss_dist)];
    trajectoryDeviation = [trajectoryDeviation, max(data_deviation)];
    maxDistPath = [maxDistPath, max(dist_from_path)];
    arrivalTime = [arrivalTime, time];
    send(dq, i);
end
%%
fig_min_miss_dist = figure(1);
cases = 1:1:500;
scatter(cases, minMissDist);
hold on; grid on;
plot(cases, 3*ones([1, length(cases)]), 'r--');
xlabel("cases");
ylabel("Min miss distance(m)");

fig_max_traj_dev = figure(2);
scatter(cases, trajectoryDeviation);
hold on; grid on;
xlabel("cases");
ylabel("Max trajectory deviation(m)");
%%
folderPath = fullfile(pwd, 'log', 'Monte-Carlo');

% Create output folder if it does not exist
if ~exist(folderPath, 'dir')
    mkdir(folderPath);
end

save(fullfile(folderPath, "ResultMC.mat"), "arrivalTime", "minMissDist", "trajectoryDeviation", "maxDistPath");

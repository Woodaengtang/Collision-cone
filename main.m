close all; clear all; clc;
addpath("include\");
addpath("lib\");

%% Initialize simulation settings
[initCond, initInput, params, simTime] = initSimul();
egoUAV = MultiCopter(initCond, initInput, params, simTime);

[simulTime, velTimes, attTimes, omgTimes] = initRatesPID();
thrustGain = initGainsPID([25; 0.001; 0.005]);
velGainPID = initGainsPID([25; 0.001; 0.005]);    % m/s in FLU coordinate
attGainPID = initGainsPID([10; 0; 0.00005]);      % Euler angle
omgGainPID = initGainsPID([10; 0.2; 0.1]);
rateSaturation = deg2rad(60);
attSaturation = deg2rad(30);

[ioThrust, ioHeading] = deal(1, 1);
ioAtt = 2;
ioSize = 3;

maxThrust = initInput(1)*2;
minThrust = 0;
thrustCtrl = ThrustPID("thrustCtrl", thrustGain, velTimes, ioThrust, initInput(1));
thrustCtrl.setOutputSaturation(minThrust, maxThrust);

headingCtrl = HeadingPID("headingCtrl", attGainPID, velTimes, ioHeading);
headingCtrl.setOutputSaturation(-rateSaturation, rateSaturation);

velCtrl = GuidancePID("velCtrl", velGainPID, velTimes, ioSize);
velCtrl.setOutputSaturation(-attSaturation, attSaturation);

attCtrl = ControllerPID("attCtrl", attGainPID, attTimes, ioSize);
attCtrl.setOutputSaturation(-rateSaturation, rateSaturation);

omgCtrl = RatePID("omgCtrl", omgGainPID, omgTimes, ioSize);

time = 0;

% loop indexes
loopIdx = 0;
loopVel = omgTimes.rate/velTimes.rate;
loopAtt = omgTimes.rate/attTimes.rate;

distThreshold = 0.5;    % distance threshold

goalPoint = [50, 0, 10];

% Initial reference command
ref = struct( ...
    "velocity", [2; 2; 0],...
    "attitude", zeros([3,1]),...
    "omega", zeros([3,1]));

%% Main simulation loops
while and(time <= simulTime, norm(goalPoint-egoUAV.pos) > distThreshold)

    if mod(loopIdx, loopVel) == 0
        egoUAV.u(1) = thrustCtrl.update(ref.velocity(3), egoUAV.vel(3));
        velCtrl.getAtt(egoUAV.att);
        ref.attitude = velCtrl.update(ref.velocity, egoUAV.vel);
        ref.attitude(3) = headingCtrl.update(atan2(ref.velocity(2), ref.velocity(1)), egoUAV.yaw);
    end

    if mod(loopIdx, loopAtt) == 0
        ref.omega = attCtrl.update(ref.attitude, egoUAV.att);
    end

    omgCtrl.getRate(egoUAV.omg);
    egoUAV.moment = omgCtrl.update(ref.omega, egoUAV.omg);
    [egoUAV.u(2), egoUAV.u(3), egoUAV.u(4)] = deal(egoUAV.moment(1), egoUAV.moment(2), egoUAV.moment(3));
    egoUAV.applyControlStep();
    loopIdx = loopIdx + 1;
    time = time + egoUAV.dt;
end
%% Save logs to files
folderPath = fullfile(pwd, 'log', 'simul');
if ~exist(folderPath, 'dir')
    mkdir(folderPath);
end
save(fullfile(folderPath, "SimComparison.mat"), "attCtrl",...
    "egoUAV", "headingCtrl", "omgCtrl", "thrustCtrl", "velCtrl");

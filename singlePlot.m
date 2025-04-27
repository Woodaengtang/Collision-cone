clc; close all; clear all;

%% Index
% Set the path to the "single" folder inside "log"
folderPath = fullfile(pwd, "log", "single");

% Get a list of all .mat files in the "single" folder
matFiles = dir(fullfile(folderPath, "*.mat"));

% Loop through each .mat file and load them into the workspace
for i = 1:length(matFiles)
    fileName = matFiles(i).name;
    filePath = fullfile(folderPath, fileName);
    load(filePath);
end

addpath("lib");

R_safe = 3;
scenario_interval = 20;
nonzero = 3140;
datalength = 20;
snapshot_linewidth = 2;

miss_dist = zeros([1, length(ego_state_log)]);
traj_deviation = zeros([3, length(ego_state_log)]);
traj_int = 0;
idx_init = 1;

for i = 1 : length(ego_state_log)
    miss_dist(i) = norm(ego_state_log(1:3, i) - intruder_state_log(1:3, i));
    traj_deviation(:,i) = (ego_state_log(4:6, i) - ego_state_log(4:6, 1)).*0.001;
end

data_deviation = zeros([1, length(ego_state_log)]);
dev_sum = zeros([3, 1]);
for i = 1:length(data_deviation)
    dev_sum(1) = sum(traj_deviation(1,1:i));
    dev_sum(2) = sum(traj_deviation(2,1:i));
    dev_sum(3) = sum(traj_deviation(3,1:i));
    data_deviation(i) = norm(dev_sum);
end


[~, idx] = min(miss_dist);


%% plot data
input_linewidth = 1.5;
output_linewidth = 2;
idx_linewidth = 0.8;
time = (1:length(ego_state_log))*0.001;
rad_to_deg = 180/pi;
deg_to_rad = 1/rad_to_deg;

plot_size = [630, 700];

figure();
sgtitle("Velocity Response");
subplot(3, 1, 1);
EN_out = plot(time, ego_state_log(4, :), "LineWidth", output_linewidth);
hold on
grid on
VN_ref = plot(time, ego_command_log(1, :), "r", "LineWidth", input_linewidth);
plot_idx_lines(idx, time, idx_linewidth);
ylabel("Vn(m/s)");
ylim([min(ego_state_log(4, :))-0.1, max(ego_state_log(4, :))+0.1]);
legend([VN_ref, EN_out], {"Command", "Response"});
set(gca, "FontSize", 10);

subplot(3, 1, 2);
VE_out = plot(time, ego_state_log(5, :), "LineWidth", output_linewidth);
hold on
grid on
EN_ref = plot(time, ego_command_log(2, :), "r", "LineWidth", input_linewidth);
plot_idx_lines(idx, time, idx_linewidth);
ylabel("Ve(m/s)");
ylim([min(ego_state_log(5, :))-0.1, max(ego_state_log(5, :))+0.1]);
set(gca, "FontSize", 10);

subplot(3, 1, 3);
DN_out = plot(time, ego_state_log(6, :), "LineWidth", output_linewidth);
hold on
grid on
DN_ref = plot(time, ego_command_log(3, :), "r", "LineWidth", input_linewidth);
plot_idx_lines(idx, time, idx_linewidth);
ylabel("Vd(m/s)");
xlabel("Time(s)");
ylim([min(ego_state_log(6, :))-0.1, max(ego_state_log(6, :))+0.1]);
set(gca, "FontSize", 10);
set(gca, "YDir", "reverse");

figure();
sgtitle("Attitude Response");
subplot(3, 1, 1);
phi_out = plot(time, rad_to_deg*ego_state_log(7, :), "LineWidth", output_linewidth);
hold on
grid on
phi_ref = plot(time, rad_to_deg*ego_command_log(4, :), "r", "LineWidth", input_linewidth);
plot_idx_lines(idx, time, idx_linewidth);
ylabel("\phi(deg)");
ylim([rad_to_deg*min(ego_command_log(4, :))-0.1, rad_to_deg*max(ego_command_log(4, :))+0.1]);
legend([phi_ref, phi_out], {"Command", "Response"});
set(gca, "FontSize", 10);

subplot(3, 1, 2);
theta_out = plot(time, rad_to_deg*ego_state_log(8, :), "LineWidth", output_linewidth);
hold on
grid on
theta_ref = plot(time, rad_to_deg*ego_command_log(5, :), "r", "LineWidth", input_linewidth);
plot_idx_lines(idx, time, idx_linewidth);
ylabel("\theta(deg)");
ylim([rad_to_deg*min(ego_command_log(5, :))-0.1, rad_to_deg*max(ego_command_log(5, :))+0.1]);
set(gca, "FontSize", 10);

subplot(3, 1, 3);
psi_out = plot(time, rad_to_deg*ego_state_log(9, :), "LineWidth", output_linewidth);
hold on
grid on
psi_ref = plot(time, rad_to_deg*ego_command_log(6, :), "r", "LineWidth", input_linewidth);
plot_idx_lines(idx, time, idx_linewidth);
ylabel("\psi(deg)");
xlabel("Time(s)");
ylim([rad_to_deg*min(ego_command_log(6, :))-0.1, rad_to_deg*max(ego_command_log(6, :))+0.1]);
set(gca, "FontSize", 10);

figure();
sgtitle("Angular rate Response");
subplot(3, 1, 1);
p_out = plot(time, rad_to_deg*ego_state_log(10, :), "LineWidth", output_linewidth);
hold on
grid on
p_ref = plot(time, rad_to_deg*ego_command_log(7, :), "r", "LineWidth", input_linewidth);
plot_idx_lines(idx, time, idx_linewidth);
ylabel("p(deg/s)");
ylim([rad_to_deg*min(ego_command_log(7, :))-0.1, rad_to_deg*max(ego_command_log(7, :))+0.1]);
legend([p_ref, p_out], {"Command", "Response"});
set(gca, "FontSize", 10);

subplot(3, 1, 2);
q_out = plot(time, rad_to_deg*ego_state_log(11, :), "LineWidth", output_linewidth);
hold on
grid on
q_ref = plot(time, rad_to_deg*ego_command_log(8, :), "r", "LineWidth", input_linewidth);
plot_idx_lines(idx, time, idx_linewidth);
ylabel("q(deg/s)");
ylim([rad_to_deg*min(ego_command_log(8, :))-0.1, rad_to_deg*max(ego_command_log(8, :))+0.1]);
set(gca, "FontSize", 10);

subplot(3, 1, 3);
r_out = plot(time, rad_to_deg*ego_state_log(12, :), "LineWidth", output_linewidth);
hold on
grid on
r_ref = plot(time, rad_to_deg*ego_command_log(9, :), "r", "LineWidth", input_linewidth);
plot_idx_lines(idx, time, idx_linewidth);
ylabel("r(deg/s)");
ylim([rad_to_deg*min(ego_command_log(9, :))-0.1, rad_to_deg*max(ego_command_log(9, :))+0.1]);
xlabel("Time(s)");
set(gca, "FontSize", 10);

%%%% Min-Miss Distance / Trajectory Deviation / Control Input %%%%

figure();
missDistance = plot(time, miss_dist, "LineWidth", output_linewidth);
hold on
safeRadius = plot(time, ones([1, length(ego_state_log)])*3, "r--", "LineWidth", input_linewidth);
plot_idx_lines(idx, time, idx_linewidth);
[val, i] = min(abs(miss_dist(1:int32(length(miss_dist)/2))-20));
scatter(i/1000, miss_dist(i), "ko");
xlabel("time(s)"); ylabel("Miss Distance(m)");
legend([missDistance, safeRadius], {"Miss Distance", "Safe Radius"});
title("Miss Distance between UAV and Intruder");
ylim([0, 23]);
grid on

figure();
plot(time, data_deviation, "LineWidth", output_linewidth);
xlabel("time(s)"); ylabel("Trajectory Deviation(m)");
title("Trajectory Deviation of UAV");
grid on
hold on
plot_idx_lines(idx, time, idx_linewidth);

figure();
sgtitle("Control Input of UAV");
subplot(4, 1, 1);
plot(time, control_input_log(1,:), "LineWidth", output_linewidth);
hold on
plot_idx_lines(idx, time, idx_linewidth);
ylabel("Thrust(N)");
grid on

subplot(4, 1, 2);
plot(time, control_input_log(2,:), "LineWidth", output_linewidth);
ylabel("M_i");
hold on
plot_idx_lines(idx, time, idx_linewidth);
grid on

subplot(4, 1, 3);
plot(time, control_input_log(3,:), "LineWidth", output_linewidth);
hold on
plot_idx_lines(idx, time, idx_linewidth);
ylabel("M_j");
grid on

subplot(4, 1, 4);
plot(time, control_input_log(4,:), "LineWidth", output_linewidth);
hold on
plot_idx_lines(idx, time, idx_linewidth);
xlabel("time(s)"); ylabel("M_k");
grid on

figure();
plot(time, -err_log, "LineWidth", output_linewidth);
grid on
hold on
plot(time, zeros([1,length(time)]), "r--" ,"LineWidth", input_linewidth);
xlabel("time(s)"); ylabel("y(t)"); title("Plot of Collision condition");
xlim([0, 25]);


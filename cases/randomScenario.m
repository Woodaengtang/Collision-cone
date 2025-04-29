clc; close all; clear all;
rng(0);  % Set random seed for reproducibility

% Add parent and "lib" folder paths to access helper functions
addpath('../lib/');

%% Define the line segment and simulation parameters
% Define two points that form the line segment (start and end).
point1 = [0, 0, -10];     % Start point in 3D (NED coordinate system)
point2 = [0, 50, -10];    % End point in 3D

ego_speed = 3;            % UAV (ego) speed (m/s)
int_speed_thres = 3.5;    % Intruder speed threshold (m/s)

%% Generate random points within a cylindrical space
num_case = 500;                   % Number of random cases/points to generate
num_points_cylinder = num_case;   % Number of points in the cylinder
num_points_sphere = num_case;     % Number of points in the sphere

len_thres = 2.5;            % Radius of the cylinder

% Generate random angles for cylindrical coordinates (0 to 2*pi)
theta = 2 * pi * rand(num_points_cylinder, 1);
% Generate random radii: scaled random values between 0.2*radius_cylinder and 1*radius_cylinder
r = len_thres * (0.8 * rand(num_points_cylinder, 1) + 0.2);
% Generate random y-coordinates along the cylinder height (centered between 12.5 and 37.5)
y_cylinder = 25 * rand(num_points_cylinder, 1) + 12.5;

% Convert cylindrical coordinates to Cartesian coordinates.
% x and z are computed from r and theta while y remains as given.
x_cylinder = r .* sin(theta);      % X-coordinate (using sine component)
z_cylinder = r .* cos(theta) - 10;   % Z-coordinate adjusted by -10 to match the vertical offset

% Assemble the cylinder points into a 3xN matrix.
RandomCollisionPoints = [x_cylinder'; y_cylinder'; z_cylinder'];

%% Generate random points within a spherical space
% Define the sphere's center and radius
center_sphere = [0, 25, -10];    % Center of the sphere in 3D space
radius_sphere = 30;              % Sphere radius

% Define an anonymous function to generate sphere points:
% It returns random radius (with cubic root to ensure uniform distribution),
% a random angle (phi, from 0 to 2*pi) and a costheta value between -1 and 1.
generate_sphere_points = @(n) deal(radius_sphere * rand(n, 1).^(1/3), 2 * pi * rand(n, 1), 2 * rand(n, 1) - 1);

% Initialize arrays to store sphere points and intruder velocities.
x_sphere = [];
y_sphere = [];
z_sphere = [];
IntruderVeloicty = [];

% Index counter for sphere points accepted
idx = 1;
% Keep generating points until we have the desired number.
while length(x_sphere) < num_points_sphere
    % Generate one random point in spherical coordinates
    [r_sphere, phi, costheta] = generate_sphere_points(1);
    theta = acos(costheta);  % Convert cosine value to polar angle
    
    % Convert spherical coordinates to Cartesian coordinates,
    % and add the sphere center offset.
    x_temp = r_sphere * sin(theta) * cos(phi) + center_sphere(1);
    y_temp = r_sphere * sin(theta) * sin(phi) + center_sphere(2);
    z_temp = r_sphere * cos(theta) + center_sphere(3);
    
    % Filter out points that are within an "extended cylinder" defined by thresholds:
    % Compute distances from the start and end points of the line segment.
    dist_from_start = norm([x_temp, y_temp, z_temp] - point1);
    is_outside_sphere = dist_from_start > 20;
    dist_from_end = norm([x_temp, y_temp, z_temp] - point2);
    is_outside_goal = dist_from_end > 10;
    
    % If the point is sufficiently distant from both endpoints...
    if and(is_outside_sphere, is_outside_goal)
        % Obtain additional spatial points using a helper function get_points.
        % This returns two points: p_on_nom (nominal CPA point for ego) and p_on_int (CPA for intruder).
        [p_on_nom, p_on_int] = get_points(point1', point2', [x_temp; y_temp; z_temp], RandomCollisionPoints(:, idx));
        % Compute the time to CPA along the nominal path using y-component (assumed velocity along y)
        if norm(p_on_nom - p_on_int) > len_thres
            continue;
        end
        t_cpa = p_on_nom(2) / ego_speed;
        % Compute the intruder's velocity (difference between collision point and generated point)
        int_vel = (RandomCollisionPoints(:, idx) - [x_temp; y_temp; z_temp]) / t_cpa;
        % If the computed intruder velocity exceeds the threshold, skip this point.
        if norm(int_vel) > int_speed_thres
            continue;
        end
        
        % If the point is valid, append its coordinates to the sphere arrays.
        x_sphere = [x_sphere; x_temp];
        y_sphere = [y_sphere; y_temp];
        z_sphere = [z_sphere; z_temp];
        % Append the computed intruder velocity to IntruderVeloicty.
        IntruderVeloicty = [IntruderVeloicty, int_vel];
        idx = idx + 1;
    else
        % If the point does not satisfy the criteria, continue to the next iteration.
        continue;
    end
end

% Truncate any extra points to have exactly num_points_sphere points.
x_sphere = x_sphere(1:num_points_sphere);
y_sphere = y_sphere(1:num_points_sphere);
z_sphere = z_sphere(1:num_points_sphere);

%% Plotting the Results
figure;

% Plot the line segment (from point1 to point2) in blue.
plot3([point1(1), point2(1)], [point1(2), point2(2)], [point1(3), point2(3)], 'b-', 'LineWidth', 2);
hold on;

% Plot the random points generated in the cylindrical volume in red dots.
scatter3(x_cylinder, y_cylinder, z_cylinder, 'r.');

% Plot the random points generated in the spherical volume in green 'x' markers.
scatter3(x_sphere, y_sphere, z_sphere, 'gx');

% Label the axes and add a title.
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Random Points in Cylindrical and Spherical Spaces');

grid on;  % Enable grid
hold off;

%% Save the generated random points and intruder velocities
% Save RandomCollisionPoints (from cylinder) as a MAT file.
save("RandomCollisionPoints.mat", "RandomCollisionPoints");

% Assemble intruder random points from sphere points into a 3xN matrix.
IntruderRandomPoints = [x_sphere'; y_sphere'; z_sphere'];
save("IntruderRandomPoints.mat", "IntruderRandomPoints");

% Save IntruderVeloicty as a MAT file.
save("IntruderVeloicty.mat", "IntruderVeloicty");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper Function: rand_condition
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function bool = rand_condition(init_p, goal_p, init_r, goal_r)
    % rand_condition: Determines whether the point along a path meets a specific condition.
    %
    % It uses the helper function get_points to compute two candidate points and then
    % evaluates if the nominal candidate is beyond the intruder's point.
    
    [p_on_nom, p_on_int] = get_points(init_p, goal_p, init_r, goal_r);
    int_len = norm(goal_r - init_r);
    int_dir = (goal_r - init_r) ./ int_len;
    
    if dot((p_on_nom - init_r), int_dir) > int_len
        bool = true;
    else
        bool = false;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper Function: get_points
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [p_on_nom, p_on_int] = get_points(init_p, goal_p, init_r, goal_r)
    % get_points: Computes candidate points along two linear segments.
    %
    % Inputs:
    %   init_p - 3x1 starting point of the first segment.
    %   goal_p - 3x1 ending point of the first segment.
    %   init_r - 3x1 starting point of the second segment.
    %   goal_r - 3x1 ending point of the second segment.
    %
    % Outputs:
    %   p_on_nom - 3x1 point computed along the first segment.
    %   p_on_int - 3x1 point computed along the second segment.
    
    % Compute the difference vector between init_r and init_p.
    d = init_r - init_p;
    % Normalize the vector from init_p to goal_p.
    d1 = (goal_p - init_p) ./ norm(goal_p - init_p);
    % Normalize the vector from init_r to goal_r.
    d2 = (goal_r - init_r) ./ norm(goal_r - init_r);

    % Setup the linear system based on dot products:
    % A, B, C, D, and E are computed as inner products of these vectors.
    A = dot(d1, d2);
    B = dot(d1, d1);
    C = dot(d2, d2);
    D = dot(d1, d);
    E = dot(d2, d);
    
    % Formulate and solve the 2x2 linear system to compute parameters.
    invM = [A, -B; C, -A];
    vecM = -[D; E];
    params = invM \ vecM;
    
    % Compute candidate points along each segment using the obtained parameters.
    p_on_nom = params(2) * d1 + init_p;
    p_on_int = params(1) * d2 + init_r;
end

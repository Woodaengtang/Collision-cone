function [isDangerous, t_CPA, egoCPA, intruderCPA] = isCollision(ego_motion, int_motion, R_safe)
    % isCollision: Determines whether a collision danger exists between the ego UAV
    % and an intruder by computing their Closest Point of Approach (CPA).
    %
    % Inputs:
    %   ego_motion   - A vector representing the full state of the ego UAV
    %                  ([position; velocity] with position and velocity each being 3x1).
    %   ego_goal     - A 3x1 vector representing the goal (destination) of the ego UAV.
    %   int_motion   - A vector representing the intruder's state
    %                  ([position; velocity] with position and velocity each being 3x1).
    %   R_safe       - A scalar specifying the safety radius (threshold distance).
    %
    % Outputs:
    %   isDangerous  - A logical flag, true if a collision danger is detected.
    %   t_CPA        - The computed time at which the Closest Point of Approach occurs.
    %   egoCPA       - The position (3x1 vector) of the ego UAV at CPA.
    %   intruderCPA  - The position (3x1 vector) of the intruder at CPA.

    
    % Extract position and velocity components from the ego UAV state vector.
    ego_pos = ego_motion(1:3);  % Current position of the ego UAV
    ego_vel = ego_motion(4:6);  % Current velocity of the ego UAV
    % (A version for virtual velocity aligned to the goal is commented out)

    % Extract position and velocity components from the intruder's state vector.
    int_pos = int_motion(1:3);  % Current position of the intruder
    int_vel = int_motion(4:6);  % Current velocity of the intruder

    % Compute the relative position and relative velocity between the ego UAV and the intruder.
    rel_Pos = (ego_pos - int_pos);   % Difference in positions (vector from intruder to ego)
    rel_Vel = (ego_vel - int_vel);   % Difference in velocities
    
    % Calculate the time to Closest Point of Approach (t_CPA). This gives the time at which the distance between the two objects is minimal.
    t_CPA = -(rel_Pos' * rel_Vel) / (rel_Vel' * rel_Vel);
    
    % Compute the CPA (Closest Point of Approach) positions for both the ego UAV and the intruder.
    egoCPA      = ego_pos + ego_vel * t_CPA;      % Position of ego UAV at t_CPA
    intruderCPA = int_pos + int_vel * t_CPA;        % Position of intruder at t_CPA
    
    % Determine if a collision is dangerous:
    % 1. The distance between the UAV and the intruder at CPA must be less than or equal to R_safe.
    % 2. The computed t_CPA must be non-negative (future or current CPA).
    if and(norm(rel_Pos + rel_Vel * t_CPA) <= R_safe, t_CPA >= 0)
        isDangerous = true;
    else
        isDangerous = false;
    end

end

function [rMin, tMin, rI, vI, flag] = isCollision(ego_motion, int_motion, R_safe)
    % give flag if UAV is in collision threat or not

    NED2FLU = [1, 0, 0;...
               0, -1, 0;...
               0, 0, -1];
    egoPos = NED2FLU*ego_motion(1:3);
    egoVel = NED2FLU*ego_motion(4:6);
    intPos = NED2FLU*int_motion(1:3);
    intVel = NED2FLU*int_motion(4:6);

    % Get radian info
    rI = getSphericCoord(egoPos, egoVel, intPos, intVel);
    
    relPos = intPos - egoPos;
    relVel = intVel - egoVel;

    V_A = norm(egoVel);
    V_B = norm(intVel);
    
    thetaVel = V_B*cos(rI.eta)*sin(rI.mu - rI.theta) - V_A*cos(rI.alpha)*sin(rI.beta - rI.theta);
    phiVel = V_B*(-cos(rI.eta)*sin(rI.phi)*cos(rI.mu - rI.theta) + sin(rI.eta)*cos(rI.phi)) - V_A*(-cos(rI.alpha)*sin(rI.phi)*cos(rI.beta - rI.theta) + sin(rI.alpha)*cos(rI.phi));
    rVel = V_B*(cos(rI.eta)*cos(rI.phi)*cos(rI.mu - rI.theta) + sin(rI.eta)*sin(rI.phi)) - V_A*(cos(rI.alpha)*cos(rI.phi)*cos(rI.beta - rI.theta) + sin(rI.alpha)*cos(rI.phi));
    
    % Velocity information is defined in FLU spherical coordinate
    vI = struct(...
        "thetaVel", thetaVel,...
        "phiVel", phiVel,...
        "rVel", rVel);

    rMin2 = norm(relPos)^2*((thetaVel^2 + phiVel^2)/(relVel'*relVel));
    rMin = sqrt(rMin2);
    tMin = -(norm(relPos)*rVel)/(relVel'*relVel);

    if rMin2 < R_safe^2
        flag = true;    % in collision threat
    else
        flag = false;   % not in collision threat
    end
    
end


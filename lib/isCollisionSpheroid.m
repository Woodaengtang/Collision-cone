function [rMin, tMin, rI, vI, flag] = isCollisionSpheroid(ego_motion, int_motion, rMajor, C1, C2)
    % give flag if UAV is in collision threat or not

    NED2FLU = [1, 0, 0;...
               0, -1, 0;...
               0, 0, -1];
    egoPos = NED2FLU*ego_motion(1:3);
    egoVel = NED2FLU*ego_motion(4:6);
    intVel = NED2FLU*int_motion(4:6);
    c1Pos = NED2FLU*C1;
    c2Pos = NED2FLU*C2;
    relC1 = c1Pos - egoPos;
    relC2 = c2Pos - egoPos;

    % Get radian info
    [rI1, tFI1] = getSphericCoord(egoPos, egoVel, relC1, intVel);
    [rI2, tFI2] = getSphericCoord(egoPos, egoVel, relC2, intVel);
    rI.c1 = rI1;
    rI.c2 = rI2;

    relVel = intVel - egoVel;

    V_A = norm(egoVel);
    V_B = norm(intVel);
    
    thetaVel1 = V_B*tFI1.cosEta*tFI1.sinMuTheta - V_A*tFI1.cosAlpha*tFI1.sinBetaTheta;
    phiVel1 = V_B*(-tFI1.cosEta*tFI1.sinPhi*tFI1.cosMuTheta + tFI1.sinEta*tFI1.cosPhi) - V_A*(-tFI1.cosAlpha*tFI1.sinPhi*tFI1.cosBetaTheta + tFI1.sinAlpha*tFI1.cosPhi);
    rVel1 = V_B*(tFI1.cosEta*tFI1.cosPhi*tFI1.cosMuTheta + tFI1.sinEta*tFI1.sinPhi) - V_A*(tFI1.cosAlpha*tFI1.cosPhi*tFI1.cosBetaTheta + tFI1.sinAlpha*tFI1.cosPhi);

    thetaVel2 = V_B*tFI2.cosEta*tFI2.sinMuTheta - V_A*tFI2.cosAlpha*tFI2.sinBetaTheta;
    phiVel2 = V_B*(-tFI2.cosEta*tFI2.sinPhi*tFI2.cosMuTheta + tFI2.sinEta*tFI2.cosPhi) - V_A*(-tFI2.cosAlpha*tFI2.sinPhi*tFI2.cosBetaTheta + tFI2.sinAlpha*tFI2.cosPhi);
    rVel2 = V_B*(tFI2.cosEta*tFI2.cosPhi*tFI2.cosMuTheta + tFI2.sinEta*tFI2.sinPhi) - V_A*(tFI2.cosAlpha*tFI2.cosPhi*tFI2.cosBetaTheta + tFI2.sinAlpha*tFI2.cosPhi);
    
    % Velocity information is defined in FLU spherical coordinate
    vI1 = struct(...
        "thetaVel", thetaVel1,...
        "phiVel", phiVel1,...
        "rVel", rVel1);
    vI2 = struct(...
        "thetaVel", thetaVel2,...
        "phiVel", phiVel2,...
        "rVel", rVel2);

    vI.c1 = vI1;
    vI.c2 = vI2;

    rm1 = sqrt((relC1'*relC1)*(thetaVel1^2 + phiVel1^2)/(relVel^2));
    rm2 = sqrt((relC2'*relC2)*(thetaVel2^2 + phiVel2^2)/(relVel^2));
    rMin = rm1 + rm2;

    tm1 = -(rm1*norm(relVel))/(relVel'*relVel);
    tm2 = -(rm2*norm(relVel))/(relVel'*relVel);
    tMin = (rm1*tm2 + rm2*tm1)/(rm1 + rm2);
    
    if (rm1 + rm2)^2 <= (2*rMajor)^2
        flag = true;    % in collision threat
    else
        flag = false;   % not in collision threat
    end
    
end


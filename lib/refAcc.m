function  aA = refAcc(ego_motion, int_motion, radInfo, velInfo, gainK, R_safe, delta, gamma)
    NED2FLU = [1, 0, 0;...
               0, -1, 0;...
               0, 0, -1];
    egoPos = NED2FLU*ego_motion(1:3);
    egoVel = NED2FLU*ego_motion(4:6);
    intPos = NED2FLU*int_motion(1:3);
    intVel = NED2FLU*int_motion(4:6);
    relPos = intPos - egoPos;
    relVel = intVel - egoVel;
    A = -0.5*gainK*(relVel^2)/(relPos^2);
    B = (relPos^2)*(velInfo.thetaVel^2 + velInfo.phiVel^2) - R_safe^2*(relVel^2);
    C = -velInfo.thetaVel*(velInfo.rVel^2)*cos(gamma)*sin(delta - radInfo.theta) + velInfo.phiVel*(velInfo.rVel^2)*(cos(gamma)*sin(radInfo.phi)*cos(delta - radInfo.theta) - sin(gamma)*cos(radInfo.phi)) + velInfo.rVel*(relVel^2 - velInfo.rVel^2)*(cos(gamma)*cos(radInfo.phi)*cos(delta - radInfo.theta) + sin(gamma)*sin(radInfo.phi));

    scale = A*B/C;

    aA = NED2FLU'*[cos(gamma)*cos(delta);...
                   cos(gamma)*sin(delta);...
                   sin(delta)]*scale;
    
end
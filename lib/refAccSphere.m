function  aA = refAccSphere(ego_motion, int_motion, radInfo, velInfo, gainK, R_safe, delta, gamma)
    NED2FLU = [1, 0, 0;...
               0, -1, 0;...
               0, 0, -1];
    egoPos = NED2FLU*ego_motion(1:3);
    egoVel = NED2FLU*ego_motion(4:6);
    intPos = NED2FLU*int_motion(1:3);
    intVel = NED2FLU*int_motion(4:6);
    relPos = intPos - egoPos;
    relVel = intVel - egoVel;
    sinDelta = sin(delta);
    cosDelta = cos(delta);
    sinGamma = sin(gamma);
    cosGamma = cos(gamma);
    sinPhi = sin(radInfo.phi);
    cosDeltaTheta = cos(delta - radInfo.theta);
    A = -0.5*gainK*(relVel'*relVel)/(relPos'*relPos);
    B = (relPos'*relPos)*(velInfo.thetaVel^2 + velInfo.phiVel^2) - R_safe^2*(relVel'*relVel);
    C = -velInfo.thetaVel*(velInfo.rVel^2)*cosGamma*sin(delta - radInfo.theta) + velInfo.phiVel*(velInfo.rVel^2)*(cosGamma*sinPhi*cosDeltaTheta - sinGamma*cos(radInfo.phi)) + velInfo.rVel*(relVel'*relVel - velInfo.rVel^2)*(cosGamma*cos(radInfo.phi)*cosDeltaTheta + sinGamma*sinPhi);

    scale = A*B/C;

    aA = NED2FLU'*[cosGamma*cosDelta;...
                   cosGamma*sinDelta;...
                   sinDelta]*scale;
end
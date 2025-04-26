function err = errFcn(ego_motion, int_motion, velInfo, R_safe)
    NED2FLU = [1, 0, 0;...
               0, -1, 0;...
               0, 0, -1];
    egoPos = NED2FLU*ego_motion(1:3);
    intPos = NED2FLU*int_motion(1:3);
    relPos = intPos - egoPos;

    a = velInfo.thetaVel^2 + velInfo.phiVel^2;
    b = (a + velInfo.rVel^2);
    
    err = -((relPos'*relPos)*a/b - R_safe^2);
end


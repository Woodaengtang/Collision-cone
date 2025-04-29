function  aA = refAccSpheroid(y, gainK, rI, vI, rM, delta, gamma)
    % Equation (54)
    sinDelta = sin(delta);
    cosDelta = cos(delta);
    sinGamma = sin(gamma);
    cosGamma = cos(gamma);
    sinPhi1 = sin(rI.c1.phi);
    cosPhi1 = cos(rI.c1.phi);
    sinPhi2 = sin(rI.c2.phi);
    cosPhi2 = cos(rI.c2.phi);
    cosDeltaTheta1 = cos(delta - rI.c1.theta);
    cosDeltaTheta2 = cos(delta - rI.c2.theta);

    temp1 = -y * gainK;
    temp2 = -cosGamma * (pYpVTheta1(vI.c1.phiVel, vI.c2.phiVel, vI.c1.rVel, vI.c2.rVel, vI.c1.thetaVel, vI.c2.thetaVel, rM.rm1, rM.rm2) * sin(delta - rI.c1.theta) + pYpVTheta2(vI.c1.phiVel, vI.c2.phiVel, vI.c1.rVel, vI.c2.rVel, vI.c1.thetaVel, vI.c2.thetaVel, rM.rm1, rM.rm2) * sin(delta - rI.c2.theta));
    temp3 = pYpVPhi1(vI.c1.phiVel, vI.c2.phiVel, vI.c1.rVel, vI.c2.rVel, vI.c1.thetaVel, vI.c2.thetaVel, rM.rm1, rM.rm2) * (cosGamma * sinPhi1 * cosDeltaTheta1 - sinGamma * cosPhi1);
    temp4 = pYpVPhi2(vI.c1.phiVel, vI.c2.phiVel, vI.c1.rVel, vI.c2.rVel, vI.c1.thetaVel, vI.c2.thetaVel, rM.rm1, rM.rm2) * (cosGamma * sinPhi2 * cosDeltaTheta2 - sinGamma * cosPhi2);
    temp5 = pYpVR1(vI.c1.phiVel, vI.c2.phiVel, vI.c1.rVel, vI.c2.rVel, vI.c1.thetaVel, vI.c2.thetaVel, rM.rm1, rM.rm2) * (cosGamma * cosPhi1 * cosDeltaTheta1 + sinGamma * sinPhi1);
    temp6 = pYpVR2(vI.c1.phiVel, vI.c2.phiVel, vI.c1.rVel, vI.c2.rVel, vI.c1.thetaVel, vI.c2.thetaVel, rM.rm1, rM.rm2) * (cosGamma * cosPhi2 * cosDeltaTheta2 + sinGamma * sinPhi2);

    Acc = temp1 / (temp2 + temp3 + temp4 - temp5 - temp6);
    NED2FLU = [1, 0, 0;...
               0, -1, 0;...
               0, 0, -1];
    aA = NED2FLU'*[cosGamma*cosDelta;...
                   cosGamma*sinDelta;...
                   sinDelta]*Acc;
end
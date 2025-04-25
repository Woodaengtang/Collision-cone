function radInfo = getSphericCoord(egoPos, egoVel, intPos, intVel)
    % Get spherical coordinate expression about positions and velocities
    
    relPos = intPos - egoPos;

    alpha = asin(egoVel(3)/norm(egoVel));
    beta = atan2(egoVel(2), egoVel(1));
    eta = asin(intVel(3)/norm(intVel));
    mu = atan2(intVel(2), intVel(1));
    
    theta = atan2(relPos(2), relPos(1));
    phi = asin(relPos(3)/norm(relPos));

    radInfo = struct(...
        "alpha", alpha,...
        "beta", beta,...
        "eta", eta,...
        "mu", mu,...
        "theta", theta,...
        "phi", phi);
end
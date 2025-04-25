function [delta, gamma] = accAngle(egoMotion, r0, varphi)
    % Using grid spanning angle varphi and virtual sphere, aiming point
    % candidate based acceleration angle determination
    
    egoPos = egoMotion(1:3);
    egoVel = egoMotion(4:6);
    dirVel = egoVel./norm(egoVel);

    cndPoint = [];
    minPoint = [];

    for cnddt = varphi
        
        dirCnd = (cnddt - egoPos)./norm(cnddt - egoPos);


    end
end
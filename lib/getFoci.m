function [C1, C2, a] = getFoci(scale, rotation, intCPA, R_safe)
    
    a = R_safe*scale(1);
    b = R_safe*scale(2);
    c = sqrt(a^2 - b^2);
    
    c1 = [c; 0; 0];
    c2 = [-c; 0; 0];
    C1 = rotation'*c1 + intCPA;
    C2 = rotation'*c2 + intCPA;
end
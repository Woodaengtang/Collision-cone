function euler = quat2euler(quaternion)
    quaternion = quaternion(:);
    n = norm(quaternion);
    if n == 0
        error("Zero‑norm quaternion");
    end
    quaternion = quaternion/n;
    
    e0 = quaternion(1); 
    ex = quaternion(2); 
    ey = quaternion(3); 
    ez = quaternion(4);
    
    rol  = atan2(2*(e0*ex + ey*ez), e0^2 + ez^2 - ex^2 - ey^2);
    
    s = 2*(e0*ey - ex*ez);
    s = max(-1,min(1,s));
    % clamp to [-1,1]
    pit = asin(s);
    
    yaw   = atan2(2*(e0*ez + ex*ey),  e0^2 + ex^2 - ey^2 - ez^2);
    
    euler = [rol, pit, yaw]';
end

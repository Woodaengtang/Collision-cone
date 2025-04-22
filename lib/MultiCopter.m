classdef MultiCopter < handle
    % MultiCopter: A class to simulate the dynamics of a quadcopter.
    % It operates in the FLU coordinate frame.

    properties
        g
        dt
        time
        log_time
        sim_time

        m
        arm_len
        inertia

        pos
        pos_d
        pf
        pl
        pu

        vel
        vel_d
        vf
        vl
        vu

        quat
        quat_d
        e0
        ex
        ey
        ez

        att
        rol
        pit
        yaw

        omg
        omg_d
        p
        q
        r

        u
        T
        Mx
        My
        Mz
    end

    methods
        function obj = MultiCopter(initCond, sim_time)

            params = containers.Map({"Mass", "arm_len", "Ixx", "Iyy", "Izz"},...
                {2, 0.25, 0.021667, 0.021667, 0.04});

            obj.g = 9.81;
            obj.time = 0;
            obj.dt = 0.001;
            obj.sim_time = sim_time;
            obj.log_time = 0:dt:sim_time;

            obj.m = params("Mass");
            obj.arm_len = params("arm_len");
            obj.inertia = diag([params("Ixx"), params("Iyy"), params("Izz")]);

            obj.pf = initCond.pos(1);
            obj.pl = initCond.pos(2);
            obj.pu = initCond.pos(3);
            obj.pos = [obj.pf; obj.pl; obj.pu];

            obj.vf = initCond.vel(1);
            obj.vl = initCond.vel(2);
            obj.vu = initCond.vel(3);
            obj.vel = [obj.vf; obj.vl; obj.vu];

            obj.e0 = initCond.quat(1);
            obj.ex = initCond.quat(2);
            obj.ey = initCond.quat(3);
            obj.ez = initCond.quat(4);
            obj.quat = [obj.e0, obj.ex, obj.ey, obj.ez];

            [obj.rol, obj.pit, obj.yaw] = quat2euler(initCond.q);
            obj.att = [obj.rol; obj.pit; obj.yaw];

            obj.p = initCond.omega(1);
            obj.q = initCond.omega(2);
            obj.r = initCond.omega(3);
            obj.omg = [obj.p; obj.q; obj.r];

            obj.pos_d = NaN([3, 1]);
            obj.vel_d = NaN([3, 1]);
            obj.quat_d = NaN([4, 1]);
            obj.omg_d = NaN([3, 1]);
            
            obj.T = NaN;
            obj.Mx = NaN;
            obj.My = NaN;
            obj.Mz = NaN;
            obj.u = [obj.T; obj.Mx; obj.My; obj.Mz];
        end

        function state = get_state(obj)
            state = [obj.pos', obj.vel', obj.att', obj.omg']';
        end
        
        function input = get_inputs(obj)
            obj.u = [obj.T; obj.Mx; obj.My; obj.Mz];
            input = obj.u;
        end

        function Ri2b = inertial2body(obj)
            euler = quat2euler(obj.quat);
            phi = euler(1);     % roll
            theta = euler(2);   % pitch
            psi = euler(3);     % yaw

            sinphi = sin(phi);
            cosphi = cos(phi);
            sintht = sin(theta);
            costht = cos(theta);
            sinpsi = sin(psi);
            cospsi = cos(psi);

            Ri2b = 
        end
        
        function obj = eom(obj)
            
        end
    end
end

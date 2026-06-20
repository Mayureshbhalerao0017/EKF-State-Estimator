function p = vehicleParams()


    
    p.R  = 0.335;        % wheel radius [m]
    p.Iw = 1.65;        % wheel inertia [kg m^2] (same for all wheels here)
    p.Cx = 550000;      % longitudinal tire stiffness (Fx = Cx * sr)

    
    p.C_alpha_f = 217724;   % front axle lateral stiffness [N/rad]
    p.C_alpha_r = 275020;   % rear axle lateral stiffness [N/rad]

   
    p.Lf = 1.62;        % distance CoM -> front axle [m]
    p.Lr = 1.53;        % distance CoM -> rear axle [m]
    p.w  = 3.15;        % track width [m]

    % Vehicle mass / inertiap.m  = 798;        % mass [kg] (example, tune for your car)
    p.Iz = 2450;        % yaw moment of inertia [kg m^2] (example)


end

function x_next = vehicleProcessModel(x, u, dt, p)
    % Unpack State
    vx = x(1);
    vy = x(2);
    r  = x(3);
    ax = x(4);
    ay = x(5);
    sr = x(6:9);   % [srFL; srFR; srRL; srRR]
    
    % Unpack Inputs
    tRL   = u(1);      % Torque Rear Left
    tRR   = u(2);      % Torque Rear Right
    delta = u(3);      % Steering Angle [rad]
    
    % --- SAFETY 1: CLAMP INPUTS ---
    % Prevent steering singularities (max 45 degrees)
    delta = max(min(delta, 0.78), -0.78); 
    
    % 1. Velocity Dynamics (Kinematic)
    % --------------------------------
    % CRITICAL FIX: Ensure signs match coordinate system (SAE J670)
    vx_dot = ax + r * vy;
    vy_dot = ay - r * vx;
    
    % CRITICAL FIX: Standard Euler Integration uses PLUS (+)
    vx_next = vx + dt * vx_dot;
    vy_next = vy + dt * vy_dot; 
    
    % 2. Tire Force Calculation
    % --------------------------------
    Lf = p.Lf; Lr = p.Lr;
    Cx = p.Cx; Caf = p.C_alpha_f; Car = p.C_alpha_r;
    Iz = p.Iz;
    
    % --- SAFETY 2: LOW SPEED PROTECTION ---
    % If moving too slow, physics are unstable. Clamp effective velocity.
    vx_eff = max(vx, 1.0); % Treat anything < 1m/s as 1m/s for division
    
    beta    = atan2(vy, vx_eff);
    alpha_f = delta - beta - (Lf * r / vx_eff);
    alpha_r =- beta + (Lr * r / vx_eff);
    
    % LINEAR TIRE MODEL WITH SATURATION (Friction Circle)
    % This prevents forces from becoming infinite if alpha is large
    max_Fy = 4000; % Approx 4000N limit (adjust based on weight)
    Fy_FL = max(min(-Caf * alpha_f, max_Fy), -max_Fy);
    Fy_FR = max(min(-Caf * alpha_f, max_Fy), -max_Fy);
    Fy_RL = max(min(-Car * alpha_r, max_Fy), -max_Fy);
    Fy_RR = max(min(-Car * alpha_r, max_Fy), -max_Fy);
    
    Fx_FL = Cx * sr(1);
    Fx_FR = Cx * sr(2);
    Fx_RL = Cx * sr(3);
    Fx_RR = Cx * sr(4);
    
    % 3. Yaw Dynamics
    
    w = p.w;
    % Moment arms
    pos_FL = [ Lf;  w/2]; pos_FR = [ Lf; -w/2];
    pos_RL = [-Lr;  w/2]; pos_RR = [-Lr; -w/2];
    
    % Rotate Front Forces to Body Frame (Cos/Sin projection)
    s = sin(delta); c = cos(delta);
    Fx_FL_B = Fx_FL*c - Fy_FL*s; Fy_FL_B = Fx_FL*s + Fy_FL*c;
    Fx_FR_B = Fx_FR*c - Fy_FR*s; Fy_FR_B = Fx_FR*s + Fy_FR*c;
    
    % Sum Moments
    Mz = (pos_FL(1)*Fy_FL_B - pos_FL(2)*Fx_FL_B) + ...
         (pos_FR(1)*Fy_FR_B - pos_FR(2)*Fx_FR_B) + ...
         (pos_RL(1)*Fy_RL   - pos_RL(2)*Fx_RL)   + ...
         (pos_RR(1)*Fy_RR   - pos_RR(2)*Fx_RR);
         
    r_dot  = Mz / Iz;
    r_next = r + dt * r_dot;
    
    % 4. Acceleration States 
    
    ax_next = ax; 
    ay_next = ay; 
    
    % 5. Slip Ratio Dynamics (AMZ Style)
    
    tM = [0; 0; tRL; tRR]; % Motor torques
    Rw = p.R; Iw = p.Iw;
    
    % Calculate wheel velocities projected on wheel heading
    % (Simplified for debugging stability)
    vwx = [vx; vx; vx; vx]; 
    vwx = max(vwx, 0.5); % Hard floor to prevent division by zero
    
    % AMZ Dynamics
    % wdot = (Torque - Fx*R) / I_wheel
    wdot = (tM - Rw * Cx .* sr) ./ Iw;
    
    % Slip derivative
    % This term: (Rw * Cx ./ (Iw .* vwx)) is the DAMPING. 
    % If vwx is small, this term explodes. clamped vwx above.
    srdot = (Rw .* wdot) ./ vwx ...
            - sr .* (ax ./ vwx) ...
            - sr .* (Rw * Cx ./ (Iw .* vwx));
            
    sr_next = sr + dt * srdot;
    
    %  SAFETY 3: STATE SATURATION
    % Physically, a car cannot go Mach 10. If it tries, clamp it.
    
    vx_next = max(min(vx_next, 100), -20);  % Max 360 km/h
    vy_next = max(min(vy_next, 20), -20);   % Max drift
    sr_next = max(min(sr_next, 1.0), -1.0); % Slip max 100%
    
    x_next = [vx_next; vy_next; r_next; ax_next; ay_next; sr_next];
end
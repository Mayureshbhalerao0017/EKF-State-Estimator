function y = vehicleMeasurementModel(x, u, p)

    vx = x(1);
    vy = x(2);
    r  = x(3);
    ax = x(4);
    ay = x(5);
    sr = x(6:9);   % [srFL; srFR; srRL; srRR]

    
    delta = u(3);

    
    r_meas  = r;   % yaw rate
    ax_meas = ax;  % longitudinal acceleration
    ay_meas = ay;  % lateral acceleration

    
     
    Lf = p.Lf;
    Lr = p.Lr;
    w  = p.w;

    
    p_FL = [ Lf;  w/2];
    p_FR = [ Lf; -w/2];
    p_RL = [-Lr;  w/2];
    p_RR = [-Lr; -w/2];

    
    v_c_FL = [vx - r * p_FL(2);
              vy + r * p_FL(1)];
    v_c_FR = [vx - r * p_FR(2);
              vy + r * p_FR(1)];
    v_c_RL = [vx - r * p_RL(2);
              vy + r * p_RL(1)];
    v_c_RR = [vx - r * p_RR(2);
              vy + r * p_RR(1)];

    
    eF = [cos(delta); sin(delta)];  % front wheels steer
    eR = [1; 0];                    % rear wheels fixed, along x

    % Longitudinal velocities: projection onto heading
    vwx_FL = eF' * v_c_FL;
    vwx_FR = eF' * v_c_FR;
    vwx_RL = eR' * v_c_RL;
    vwx_RR = eR' * v_c_RR;

    vwx = [vwx_FL; vwx_FR; vwx_RL; vwx_RR];
    
    vwx = max(vwx, 0.01);
    Rw  = p.R;

    % wheel angular speeds (noiseless model)
    omega = (vwx .* (1 + sr)) / Rw;          % omega_i = vwx_i (1+sr_i)/R

    y = [r_meas;
         ax_meas;
         ay_meas;
         omega];
end


function state_estimation()


    clear; clc;

    % --- Configuration and Initialization ---
    p  = vehicleParams();
    dt = 0.01;          % Time step (100 Hz)
    N  = 1000;          % Number of simulation time steps

    % Initial state estimate [vx; vy; r; ax; ay; srFL; srFR; srRL; srRR]
    x_hat = zeros(9,1);         % initial state estimate
    P     = eye(9) * 0.1;       % initial covariance

    % Process noise covariance Q
    Q = diag([ ...
        0.1^2, 0.1^2, ...   % vx, vy
        0.05^2, ...         % r
        0.3^2, 0.3^2, ...   % ax, ay
        0.2^2, 0.2^2, 0.2^2, 0.2^2]);  % slip ratios

    % Measurement noise covariance R (y = [r; ax; ay; 4 wheel speeds])
    R = diag([ ...
        0.01^2, ...   % yaw rate
        0.2^2,  ...   % ax
        0.2^2,  ...   % ay
        0.05^2, 0.05^2, 0.05^2, 0.05^2]);  % wheel speeds

    % Data logging initialization
    x_log = zeros(9, N);
    u_log = zeros(3, N);    % Inputs: [tRL; tRR; delta]
    y_log = zeros(7, N);    % Measurements: [r; ax; ay; omegaFL; omegaFR; omegaRL; omegaRR]

    % --- EKF Main Loop ---
    for k = 1:N
        u_k = u_log(:,k);
        y_k = y_log(:,k);

        % 1. Prediction Step
        x_pred = vehicleProcessModel(x_hat, u_k, dt, p);
        F      = jacobianF(x_hat, u_k, dt, p);
        G      = eye(9);              % Process noise input matrix
        P_pred = F * P * F.' + G * Q * G.';

        % 2. Measurement Update Step
        y_pred = vehicleMeasurementModel(x_pred, u_k, p);
        H      = jacobianH(x_pred, u_k, p);

        innov = y_k - y_pred;         % Innovation (measurement residual)
        S     = H * P_pred * H.' + R; % Innovation covariance
        K     = P_pred * H.' / S;     % Kalman gain

        x_hat = x_pred + K * innov;   % Updated state estimate
        P     = (eye(9) - K * H) * P_pred;  % Updated covariance

        x_log(:,k) = x_hat;
    end
    
    disp('EKF simulation complete.');
end

% --- Supporting Functions ---

function p = vehicleParams()
    % Vehicle and tire parameters
    p.R  = 0.335;            % Wheel radius [m]
    p.Iw = 1.65;             % Wheel inertia [kg m^2]
    p.Cx = 550000;           % Longitudinal tire stiffness
    p.C_alpha_f = 217724;    % Front axle lateral stiffness [N/rad]
    p.C_alpha_r = 275020;    % Rear axle lateral stiffness [N/rad]
    p.Lf = 1.62;             % CoM to front axle [m]
    p.Lr = 1.53;             % CoM to rear axle [m]
    p.w  = 3.15;             % Track width [m]
    p.Iz = 2450;             % Yaw moment of inertia [kg m^2]
end

function x_next = vehicleProcessModel(x, u, dt, p)
    % Physics-based process model for vehicle dynamics
    vx = x(1); vy = x(2); r = x(3); ax = x(4); ay = x(5); sr = x(6:9);
    tRL = u(1); tRR = u(2); delta = u(3);
    
    delta = max(min(delta, 0.78), -0.78); % Safety clamp
    
    % Velocity dynamics
    vx_dot = ax + r * vy;
    vy_dot = ay - r * vx;
    vx_next = vx + dt * vx_dot;
    vy_next = vy + dt * vy_dot;
    
    % Tire forces
    vx_eff = max(vx, 1.0); % Low speed protection
    beta    = atan2(vy, vx_eff);
    alpha_f = delta - beta - (p.Lf * r / vx_eff);
    alpha_r = -beta + (p.Lr * r / vx_eff);
    
    max_Fy = 4000;
    Fy_FL = max(min(-p.C_alpha_f * alpha_f, max_Fy), -max_Fy);
    Fy_FR = Fy_FL;
    Fy_RL = max(min(-p.C_alpha_r * alpha_r, max_Fy), -max_Fy);
    Fy_RR = Fy_RL;
    
    Fx = p.Cx * sr; %
    
    % Yaw dynamics
    s = sin(delta); c = cos(delta);
    Fx_FL_B = Fx(1)*c - Fy_FL*s; Fy_FL_B = Fx(1)*s + Fy_FL*c;
    Fx_FR_B = Fx(2)*c - Fy_FR*s; Fy_FR_B = Fx(2)*s + Fy_FR*c;
    
    Mz = (p.Lf*Fy_FL_B - (p.w/2)*Fx_FL_B) + (p.Lf*Fy_FR_B - (-p.w/2)*Fx_FR_B) + ...
         (-p.Lr*Fy_RL - (p.w/2)*Fx(3)) + (-p.Lr*Fy_RR - (-p.w/2)*Fx(4));
         
    r_dot  = Mz / p.Iz;
    r_next = r + dt * r_dot;
    
    % Slip ratio dynamics (AMZ style)
    tM = [0; 0; tRL; tRR];
    vwx_vec = max([vx; vx; vx; vx], 0.5);
    wdot = (tM - p.R * p.Cx .* sr) ./ p.Iw;
    srdot = (p.R .* wdot) ./ vwx_vec - sr .* (ax ./ vwx_vec) - sr .* (p.R * p.Cx ./ (p.Iw .* vwx_vec));
    sr_next = sr + dt * srdot;
    
    % State saturation safety
    vx_next = max(min(vx_next, 100), -20);
    sr_next = max(min(sr_next, 1.0), -1.0);
    x_next = [vx_next; vy_next; r_next; ax; ay; sr_next];
end

function y = vehicleMeasurementModel(x, u, p)
    % Maps internal states to expected sensor measurements
    vx = x(1); vy = x(2); r = x(3); ax = x(4); ay = x(5); sr = x(6:9);
    delta = u(3);

    % Direct sensor mappings
    r_meas = r; ax_meas = ax; ay_meas = ay;

    % Wheel velocities calculation
    p_wheel = [p.Lf, p.Lf, -p.Lr, -p.Lr; p.w/2, -p.w/2, p.w/2, -p.w/2];
    v_c = [vx - r * p_wheel(2,:); vy + r * p_wheel(1,:)];

    % Wheel headings
    eF = [cos(delta); sin(delta)]; eR = [1; 0];
    vwx = [eF' * v_c(:,1); eF' * v_c(:,2); eR' * v_c(:,3); eR' * v_c(:,4)];
    vwx = max(vwx, 0.01);

    % Convert to wheel angular speeds
    omega = (vwx .* (1 + sr)) / p.R;
    y = [r_meas; ax_meas; ay_meas; omega];
end

function F = jacobianF(x, u, dt, p)
    % Numerical computation of the state transition Jacobian
    n = numel(x); F = zeros(n);
    fx = vehicleProcessModel(x, u, dt, p);
    eps = 1e-4;
    for j = 1:n
        dx = zeros(n,1); dx(j) = eps;
        fx2 = vehicleProcessModel(x + dx, u, dt, p);
        F(:,j) = (fx2 - fx) / eps;
    end
end

function H = jacobianH(x, u, p)
    % Numerical computation of the measurement Jacobian
    n = numel(x); 
    y = vehicleMeasurementModel(x, u, p);
    m = numel(y); H = zeros(m, n);
    eps = 1e-4;
    for j = 1:n
        dx = zeros(n,1); dx(j) = eps;
        y2 = vehicleMeasurementModel(x + dx, u, p);
        H(:,j) = (y2 - y) / eps;
    end
end
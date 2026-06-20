function runVehicleEKF_RWD()


    clear; clc;

    % parameters and timing 
    p  = vehicleParams();
    dt = 0.01;          % 100 Hz
    N  = 1000;          % number of time steps

    % initial state and covariance 
    x_hat = zeros(9,1);         % initial state estimate
    P     = eye(9) * 0.1;       % initial covariance

    % process noise Q 
    Q = diag([ ...
        0.1^2, 0.1^2, ...   % vx, vy
        0.05^2, ...         % r
        0.3^2, 0.3^2, ...   % ax, ay
        0.2^2, 0.2^2, 0.2^2, 0.2^2]);  % slip ratios

    % measurement noise R (y = [r; ax; ay; 4 wheel speeds]) 
    R = diag([ ...
        0.01^2, ...   % yaw rate
        0.2^2,  ...   % ax
        0.2^2,  ...   % ay
        0.05^2, 0.05^2, 0.05^2, 0.05^2]);  % wheel speeds

    % storage for logging 
    x_log = zeros(9, N);

    
    % RWD input: [tRL; tRR; delta]
    u_log = zeros(3, N);
    y_log = zeros(7, N);    % [r; ax; ay; omegaFL; omegaFR; omegaRL; omegaRR]

    % For now everything = 0 (just to check code runs)
    % In practice: fill u_log and y_log from your data.

    % EKF loop
    for k = 1:N

        u_k = u_log(:,k);
        y_k = y_log(:,k);

        % PREDICTION 
        x_pred = vehicleProcessModel(x_hat, u_k, dt, p);
        F      = jacobianF(x_hat, u_k, dt, p);
        G      = eye(9);              % process noise input matrix

        P_pred = F * P * F.' + G * Q * G.';

        % MEASUREMENT UPDATE
        y_pred = vehicleMeasurementModel(x_pred, u_k, p);
        H      = jacobianH(x_pred, u_k, p);

        innov = y_k - y_pred;         % innovation (measurement residual)
        S     = H * P_pred * H.' + R; % innovation covariance
        K     = P_pred * H.' / S;     % Kalman gain

        x_hat = x_pred + K * innov;   % updated state estimate
        P     = (eye(9) - K * H) * P_pred;  % updated covariance

        x_log(:,k) = x_hat;
    end


end

function [X_out, P_out, signal_est, innovation] = SineWaveEKF2(X, P_prev, ...
    ts, ... % sample time
    z)  % measurement

    % Split state vector, X[k-1], into individual variables        
    theta = X(1);
    omega = X(2);
    A = X(3);
    
    % Setup covariance matrices
    Q_proc = diag([0.001, 0.1, 0.1]); % process noise
    R_meas = 0.01; % measurement noise    
    
    %% Prediction step  
    X_apriori = zeros(3,1);
    
    X_apriori(1) = X(1) + ts * omega;
    X_apriori(2) = X(2); % constant prediction
    X_apriori(3) = X(3); % constant prediction    
    
    % Determine model Jacobian (F)   -  OBS. This is not supposed to use/depend on apriori states!
    F_prev = eye(3);    
    F_prev(1,2) = ts;
    
    % Calculate apriori covariance of estimate error
    P_apriori = F_prev * P_prev * F_prev' + Q_proc;     

    %% Update/correction step
    theta_apriori = X_apriori(1);
    omega_apriori = X_apriori(2);
    A_apriori = X_apriori(3);
    
    % Form measurement and Jacobian matrix    
    z_hat = A_apriori * sin(theta_apriori);
    H = zeros(1,3);
    H(1,1) = A_apriori * cos(theta_apriori);
    H(1,3) = sin(theta_apriori);
        
    % Calculate Kalman gain
    S = H * P_apriori * H' + R_meas;        
    K = P_apriori * H' / S;  % P_apriori * H' * inv(S)        
 
    X_aposteriori = X_apriori + K * (z - z_hat);    
    P_aposteriori = (eye(3) - K*H) * P_apriori;     
    
    % If the amplitude estimate goes negative it is because we are 180
    % degree off-phase. Correct this in a brute-force way.
    if (X_aposteriori(3) < 0)
        X_aposteriori(1) = X_aposteriori(1) + pi;
        X_aposteriori(3) = -X_aposteriori(3);
    end
    
    %% Set output    
    X_out = X_aposteriori;
    P_out = P_aposteriori;  
    signal_est = X_aposteriori(3) * sin(X_aposteriori(1));
    innovation = (z - z_hat);
end
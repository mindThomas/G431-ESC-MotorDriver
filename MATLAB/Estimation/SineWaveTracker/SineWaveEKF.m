function [X_out, P_out, innovation] = SineWaveEKF(X, P_prev, ...
    ts, ... % sample time
    z)  % measurement

    % Split state vector, X[k-1], into individual variables        
    omega = X(1); % signal frequency
    cos_theta = X(2); % A*cos(theta)
    neg_sin_theta = X(3); % dA*cos(theta)/dtheta = -A*sin(theta)
    
    % Setup covariance matrices
    Q_proc = diag([0.1, 0.01, 0.01]); % process noise
    R_meas = 0.01; % measurement noise    
    
    %% Prediction step  
    X_apriori = zeros(3,1);
     
    % Construct prediction matrix
    Ad = [ cos(omega*ts),  sin(omega*ts);
          -sin(omega*ts),  cos(omega*ts)];
    
    X_apriori(1) = omega; % constant prediction
    X_apriori(2) = cos(omega*ts) * X(2) + sin(omega*ts) * X(3);
    X_apriori(3) = -sin(omega*ts) * X(2) + cos(omega*ts) * X(3);
    
    dcos_domega = -ts * sin(omega*ts); % dcos/domega
    dsin_domega = ts * cos(omega*ts); % dsin/domega
    
    % Determine model Jacobian (F)   -  OBS. This is not supposed to use/depend on apriori states!
    F_prev = zeros(3,3);
    F_prev(1,1) = 1; % domega_apriori / domega_prev
    F_prev(2,1) = dcos_domega * X(2) + dsin_domega * X(3); % dcos_theta_apriori / domega_prev
    F_prev(3,1) = -dsin_domega * X(2) + dcos_domega * X(3); % dneg_sin_theta_apriori / domega_prev
    
    F_prev(2,2) = cos(omega*ts); % dcos_theta_apriori / dcos_theta_prev
    F_prev(2,3) = sin(omega*ts); % dcos_theta_apriori / dneg_sin_theta_prev
    
    F_prev(3,2) = -sin(omega*ts); % dneg_sin_theta_apriori / dcos_theta_prev
    F_prev(3,3) = cos(omega*ts); % dneg_sin_theta_apriori / dneg_sin_theta_prev
    
    % Calculate apriori covariance of estimate error
    P_apriori = F_prev * P_prev * F_prev' + Q_proc;     

    %% Update/correction step
    % Form measurement and Jacobian matrix    
    z_hat = X_apriori(2);
    H = [0, 1, 0];
        
    % Calculate Kalman gain
    S = H * P_apriori * H' + R_meas;        
    K = P_apriori * H' / S;  % P_apriori * H' * inv(S)        
 
    X_aposteriori = X_apriori + K * (z - z_hat);    
    P_aposteriori = (eye(3) - K*H) * P_apriori;     
     
    X_aposteriori(1) = max(X_aposteriori(1), 0.1);
    
    %% Set output    
    X_out = X_aposteriori;
    P_out = P_aposteriori;  
    innovation = (z - z_hat);
end
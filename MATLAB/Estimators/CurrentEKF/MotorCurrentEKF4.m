function [X_out, P_out, i_avg] = MotorCurrentEKF4(X, P, ...
    R, L, Ke, ... % constants/parameters
    frequency, duty, Vin, dtheta_internal, ... % inputs (u)
    i_ON_MID, i_OFF_MID)  % measurements (z)

    % Compute general parameters
    dt = 1/frequency;
    tON = duty * dt;
    tOFF = dt - tON;
    
    % Split state vector, X[k-1], into individual variables        
    i0_prev = X(1); % bottom current posterior estimate from previous timestep
    
    % Setup covariance matrices
    Q_proc = 0.001; % process noise
    R_meas = 0.1; % measurement noise
    
    %% Update step
    % Form measurement and Jacobian matrix
    z = zeros(2, 1);
    z_hat = zeros(2, 1);
    H = zeros(2, 1);
    
    Vmot = Vin - Ke*dtheta_internal;
    
    % Sampled during ON-period
    t_sample = duty/2 * dt;
    iON_MID_hat = (i0_prev - Vmot/R) * exp(-R/L*t_sample) + Vmot/R;    
    diON_di0_prev = exp(-R/L*t_sample);
    z(1,1) = i_ON_MID;
    z_hat(1,1) = iON_MID_hat;    
    H(1,:) = diON_di0_prev;
    
    % Sampled during OFF-period
    t_sample = (1-(1-duty)/2) * dt - tON;
    i_max = (i0_prev - Vmot/R)*exp(-R/L*tON) + Vmot/R;
    di_max_di0_prev = exp(-R/L*tON);
    iOFF_MID_hat = i_max * exp(-R/L*t_sample);            
    diOFF_di_max = exp(-R/L*t_sample);                            
    diOFF_di0_prev = diOFF_di_max * di_max_di0_prev;
    
    z(2,1) = i_OFF_MID;
    z_hat(2,1) = iOFF_MID_hat;    
    H(2,:) = diOFF_di0_prev;   
    
    % Calculate Kalman gain
    S = H * P * H' + R_meas*eye(2);        
    K = P * H' / S;  % P_apriori * H' * inv(S)        
 
    X = X + K * (z - z_hat);    
    P = (eye(1) - K*H) * P;      
    
    %% Prediction step  
    % Predict the correct i0 current to the next timestep
    i0_aposteriori = X;
    i_max = (i0_aposteriori - Vmot/R)*exp(-R/L*tON) + Vmot/R;
    i0_next = i_max * exp(-R/L*tOFF);
    
    i_avg = 1/dt * (Vmot/R*tON + L/R*i0_aposteriori*(1-exp(-R/L*dt)) + L/R*Vmot/R*exp(-R/L*dt) - L/R*Vmot/R*exp(-R/L*tOFF));
    
    X = i0_next;

    % Determine model Jacobian (F)   -  OBS. This is not supposed to use/depend on apriori states!
    F = exp(-R/L*dt); % di0_next / di0_aposteriori 
    
    % Calculate apriori covariance of estimate error
    P = F * P * F' + Q_proc;        
     
    %% Set output
    X_out = X;
    P_out = P;
end
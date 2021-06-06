function [X_out, P_out, i_avg] = MotorCurrentEKF5(X, P, ...
    L, Ke, ... % constants/parameters
    frequency, duty, Vin, dtheta_internal, ... % inputs (u)
    i_ON_MID, i_OFF_MID)  % measurements (z)

    % Compute general parameters
    dt = 1/frequency;
    tON = duty * dt;
    tOFF = dt - tON;
    
    % Split state vector, X[k-1], into individual variables        
    i0_prev = X(1); % bottom current posterior estimate from previous timestep
    R = X(2);
    
    % Setup covariance matrices
    Q_proc = diag([0.001, 0.001]); % process noise
    R_meas = 0.1; % measurement noise
    
    %% Update step
    % Form measurement and Jacobian matrix
    z = zeros(2, 1);
    z_hat = zeros(2, 1);
    H = zeros(2, 2);
    
    Vmot = Vin - Ke*dtheta_internal;
    
    % Sampled during ON-period
    t_sample = duty/2 * dt;
    iON_MID_hat = (i0_prev - Vmot/R) * exp(-R/L*t_sample) + Vmot/R;    
    diON_di0_prev = exp(-R/L*t_sample);
    diON_dR = Vmot/R^2*exp(-R/L*t_sample) - t_sample/L*(i0_prev - Vmot/R)*exp(-R/L*t_sample) - Vmot/R^2;
    z(1,1) = i_ON_MID;
    z_hat(1,1) = iON_MID_hat;    
    H(1,:) = [diON_di0_prev, diON_dR];
    
    % Sampled during OFF-period
    t_sample = (1-(1-duty)/2) * dt - tON;
    i_max = (i0_prev - Vmot/R)*exp(-R/L*tON) + Vmot/R;
    di_max_di0_prev = exp(-R/L*tON);
    di_max_dR = Vmot/R^2*exp(-R/L*tON) - tON/L*(i0_prev - Vmot/R)*exp(-R/L*tON) - Vmot/R^2;
    iOFF_MID_hat = i_max * exp(-R/L*t_sample);            
    diOFF_di_max = exp(-R/L*t_sample);                            
    diOFF_di0_prev = diOFF_di_max * di_max_di0_prev;
    diOFF_dR = di_max_dR * exp(-R/L*t_sample) - t_sample/L * i_max * exp(-R/L*t_sample);
    
    z(2,1) = i_OFF_MID;
    z_hat(2,1) = iOFF_MID_hat;    
    H(2,:) = [diOFF_di0_prev, diOFF_dR];   
    
    % Calculate Kalman gain
    S = H * P * H' + R_meas*eye(2);        
    K = P * H' / S;  % P_apriori * H' * inv(S)        
 
    X = X + K * (z - z_hat);    
    P = (eye(2) - K*H) * P;      
    
    %% Prediction step  
    % Predict the correct i0 current to the next timestep
    i0_aposteriori = X(1);
    i_max = (i0_aposteriori - Vmot/R)*exp(-R/L*tON) + Vmot/R;
    di_max_dR = Vmot/R^2*exp(-R/L*tON) - tON/L*(i0_aposteriori - Vmot/R)*exp(-R/L*tON) - Vmot/R^2;
    i0_next = i_max * exp(-R/L*tOFF);
    
    i_avg = 1/dt * (Vmot/R*tON + L/R*i0_aposteriori*(1-exp(-R/L*dt)) + L/R*Vmot/R*exp(-R/L*dt) - L/R*Vmot/R*exp(-R/L*tOFF));
    
    X(1) = i0_next;

    % Determine model Jacobian (F)   -  OBS. This is not supposed to use/depend on apriori states!
    F = zeros(2,2);
    F(1,1) = exp(-R/L*dt); % di0_next / di0_aposteriori 
    F(1,2) = di_max_dR * exp(-R/L*tOFF) - tOFF/L * i_max * exp(-R/L*tOFF); % di0_next / dR
    F(2,1) = 0; % dR / di0_aposteriori
    F(2,2) = 1; % dR / dR
    
    % Calculate apriori covariance of estimate error
    P = F * P * F' + Q_proc;        
     
    %% Set output
    X_out = X;
    P_out = P;
end
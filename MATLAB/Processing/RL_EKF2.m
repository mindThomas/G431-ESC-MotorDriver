function [X_out, P_out, i_avg] = RL_EKF2(X, P, ...
    Ke, ... % constants/parameters
    frequency, duty, Vin, omega, ... % inputs (u)
    i_sample1, location_sample1, ...
    i_sample2, location_sample2)  % measurements (z)

    % Compute general parameters
    dt = 1/frequency;
    tON = duty * dt;
    tOFF = dt - tON;
    
    % Split state vector, X[k-1], into individual variables        
    i0_prev = X(1); % bottom current posterior estimate from previous timestep
    R = X(2);
    L = X(3);
    
    % Setup covariance matrices
    Q_proc = diag([0, 0.000001, 0.000000001]); % process noise
    R_meas = 0.03; % measurement noise
    
    %% Update step
    % Form measurement and Jacobian matrix
    z = zeros(2, 1);
    z_hat = zeros(2, 1);
    H = zeros(2, 3);    
        
    if (location_sample1 > 0 && location_sample1 <= duty)
        % Sampled during ON-period
        t_sample = location_sample1 * dt;
        i_sample1_hat = (i0_prev - (Vin-Ke*omega)/R) * exp(-R/L*t_sample) + (Vin-Ke*omega)/R;        
        di_sample1_di0_prev = exp(-R/L*t_sample);
        di_sample1_dR = (Vin-Ke*omega)/R^2*exp(-R/L*t_sample) - t_sample/L*(i0_prev - (Vin-Ke*omega)/R)*exp(-R/L*t_sample) - (Vin-Ke*omega)/R^2;
        di_sample1_dL = R/L^2*t_sample * (i0_prev - (Vin-Ke*omega)/R) * exp(-R/L*t_sample);

        z(1,1) = i_sample1;
        z_hat(1,1) = i_sample1_hat;    
        H(1,:) = [di_sample1_di0_prev, di_sample1_dR, di_sample1_dL];
    else
        disp('Error: location_sample1 > duty which is current not handled');
    end
    
    if (location_sample2 > 0 && location_sample2 > duty)
        % Sampled during OFF-period        
        i_max_hat = (i0_prev - (Vin-Ke*omega)/R) * exp(-R/L*tON) + (Vin-Ke*omega)/R;        
        di_max_di0_prev = exp(-R/L*tON);
        di_max_dR = (Vin-Ke*omega)/R^2*exp(-R/L*tON) - tON/L*(i0_prev - (Vin-Ke*omega)/R)*exp(-R/L*tON) - (Vin-Ke*omega)/R^2;
        di_max_dL = R/L^2*tON * (i0_prev - (Vin-Ke*omega)/R) * exp(-R/L*tON);

        t_sample = location_sample2 * dt - tON;
        i_sample2_hat = (i_max_hat + Ke*omega/R) * exp(-R/L*t_sample) - Ke*omega/R;
        di_sample2_di_max = exp(-R/L*t_sample);                            
        di_sample2_di0_prev = di_sample2_di_max * di_max_di0_prev;
        di_sample2_dR = (di_max_dR - Ke*omega/R^2)*exp(-R/L*t_sample) - t_sample/L*(i_max_hat + Ke*omega/R)*exp(-R/L*t_sample) + Ke*omega/R^2;
        di_sample2_dL = di_max_dL * exp(-R/L*t_sample) + R/L^2*t_sample*(i_max_hat + Ke*omega/R)*exp(-R/L*t_sample);
        
        z(2,1) = i_sample2;
        z_hat(2,1) = i_sample2_hat;    
        H(2,:) = [di_sample2_di0_prev, di_sample2_dR, di_sample2_dL];              
    else
        disp('Error: location_sample2 < duty which is current not handled');
    end   
    
    % Calculate Kalman gain
    S = H * P * H' + R_meas*eye(2);        
    K = P * H' / S;  % P_apriori * H' * inv(S)        
 
    X = X + K * (z - z_hat);    
    P = (eye(3) - K*H) * P;
        
    if (X(3) < 1e-6)
        X(3) = 1e-6;
    end        
    
    %% Prediction step  
    % Predict the correct i0 current to the next timestep
    i0_aposteriori = X(1);
    i_max = (i0_aposteriori - (Vin-Ke*omega)/R) * exp(-R/L*tON) + (Vin-Ke*omega)/R;        
    di_max_dR = (Vin-Ke*omega)/R^2*exp(-R/L*tON) - tON/L*(i0_aposteriori - (Vin-Ke*omega)/R)*exp(-R/L*tON) - (Vin-Ke*omega)/R^2;
    i0_next = (i_max + Ke*omega/R) * exp(-R/L*tOFF) - Ke*omega/R;
    
    i_avg = 1/dt * (...
        Vin/R*tON ...
      - Ke*omega/R*dt ...
      + L/R * (i0_aposteriori + Ke*omega/R) * (1-exp(-R/L*dt)) ...
      + L/R*Vin/R * (exp(-R/L*dt) - exp(-R/L*tOFF)) ...
    );
    
    X(1) = i0_next;

    % Determine model Jacobian (F)   -  OBS. This is not supposed to use/depend on apriori states!
    F = zeros(3,3);
    F(1,1) = exp(-R/L*dt); % di0_next / di0_aposteriori 
    F(1,2) = (di_max_dR - Ke*omega/R^2)*exp(-R/L*tOFF) - tOFF/L*(i_max + Ke*omega/R)*exp(-R/L*tOFF) + Ke*omega/R^2; % di0_next / dR
    F(1,3) = 0; % di0_next / dL
    F(2,1) = 0; % dR / di0_aposteriori
    F(2,2) = 1; % dR / dR
    F(2,3) = 0; % dR / dL
    F(3,1) = 0; % dR / di0_aposteriori
    F(3,2) = 0; % dR / dR    
    F(3,3) = 1; % dL / dL
    
    % Calculate apriori covariance of estimate error
    P = F * P * F';  
    
    % Include process covariance initially to speed up convergence
    if (P(1,1) > 0.25e-3)
        P = P + Q_proc;
    end
     
    %% Set output
    X_out = X;
    P_out = P;
end
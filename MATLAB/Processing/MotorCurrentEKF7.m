function [X_out, P_out, i_avg] = MotorCurrentEKF7(X, P, ...
    L, Ke, ... % constants/parameters
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
    
    % Setup covariance matrices
    Q_proc = diag([0.001, 0]); % process noise
    R_meas = 0.03; % measurement noise
    
    %% Update step
    % Form measurement and Jacobian matrix
    z = zeros(2, 1);
    z_hat = zeros(2, 1);
    H = zeros(2, 2);    
        
    if (location_sample1 > 0 && location_sample1 <= duty)
        % Sampled during ON-period
        t_sample = location_sample1 * dt;
        i_sample1_hat = (i0_prev - (Vin-Ke*omega)/R) * exp(-R/L*t_sample) + (Vin-Ke*omega)/R;                  
        di_sample1_di0_prev = exp(-R/L*t_sample);                      
        di_sample1_dR = (Vin-Ke*omega)/R^2*exp(-R/L*t_sample) - t_sample/L*(i0_prev - (Vin-Ke*omega)/R)*exp(-R/L*t_sample) - (Vin-Ke*omega)/R^2;        

        z(1,1) = i_sample1;
        z_hat(1,1) = i_sample1_hat;    
        H(1,:) = [di_sample1_di0_prev, di_sample1_dR];
    else
        disp('Error: location_sample1 > duty which is current not handled');
    end        

    if (location_sample2 > 0 && location_sample2 > duty)
        % Sampled during OFF-period        
        i_max_hat = (i0_prev - (Vin-Ke*omega)/R) * exp(-R/L*tON) + (Vin-Ke*omega)/R;        
        di_max_di0_prev = exp(-R/L*tON);
        di_max_dR = (Vin-Ke*omega)/R^2*exp(-R/L*tON) - tON/L*(i0_prev - (Vin-Ke*omega)/R)*exp(-R/L*tON) - (Vin-Ke*omega)/R^2;

        t_sample = location_sample2 * dt - tON;
        i_sample2_hat = (i_max_hat + Ke*omega/R) * exp(-R/L*t_sample) - Ke*omega/R;
        di_sample2_di_max = exp(-R/L*t_sample);                            
        di_sample2_di0_prev = di_sample2_di_max * di_max_di0_prev;
        di_sample2_dR = (di_max_dR - Ke*omega/R^2)*exp(-R/L*t_sample) - t_sample/L*(i_max_hat + Ke*omega/R)*exp(-R/L*t_sample) + Ke*omega/R^2;
               
        z(2,1) = i_sample2;
        z_hat(2,1) = i_sample2_hat;    
        H(2,:) = [di_sample2_di0_prev, di_sample2_dR];              
    else
        disp('Error: location_sample2 < duty which is current not handled');
    end       
    
    % Calculate Kalman gain
    S = H * P * H' + R_meas*eye(2);        
    K = P * H' / S;  % P_apriori * H' * inv(S)        
 
    X = X + K * (z - z_hat);    
    P = (eye(2) - K*H) * P;      

    %% Prediction step  
    % Predict the correct i0 current to the next timestep
    i0_aposteriori = X(1);
    R_aposteriori = X(2);
    i_max = (i0_aposteriori - (Vin-Ke*omega)/R_aposteriori) * exp(-R_aposteriori/L*tON) + (Vin-Ke*omega)/R_aposteriori;        
    di_max_dR = (Vin-Ke*omega)/R_aposteriori^2*exp(-R_aposteriori/L*tON) - tON/L*(i0_aposteriori - (Vin-Ke*omega)/R_aposteriori)*exp(-R_aposteriori/L*tON) - (Vin-Ke*omega)/R_aposteriori^2;
    i0_next = (i_max + Ke*omega/R_aposteriori) * exp(-R_aposteriori/L*tOFF) - Ke*omega/R_aposteriori;
    
    i_avg = 1/dt * (...
        Vin/R_aposteriori*tON ...
      - Ke*omega/R_aposteriori*dt ...
      + L/R_aposteriori * (i0_aposteriori + Ke*omega/R_aposteriori) * (1-exp(-R_aposteriori/L*dt)) ...
      + L/R_aposteriori*Vin/R_aposteriori * (exp(-R_aposteriori/L*dt) - exp(-R_aposteriori/L*tOFF)) ...
    );
    
    X(1) = i0_next;

    % Determine model Jacobian (F)   -  OBS. This is not supposed to use/depend on apriori states!
    F = zeros(2,2);
    F(1,1) = exp(-R_aposteriori/L*dt); % di0_next / di0_aposteriori 
    F(1,2) = (di_max_dR - Ke*omega/R_aposteriori^2)*exp(-R_aposteriori/L*tOFF) - tOFF/L*(i_max + Ke*omega/R_aposteriori)*exp(-R_aposteriori/L*tOFF) + Ke*omega/R_aposteriori^2; % di0_next / dR
    F(2,1) = 0; % dR / di0_aposteriori
    F(2,2) = 1; % dR / dR
    
    % Calculate apriori covariance of estimate error
    P = F * P * F' + Q_proc;        
     
    %% Set output
    X_out = X;
    P_out = P;
end
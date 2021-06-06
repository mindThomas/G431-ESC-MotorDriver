function [X_out, P_out] = MotorCurrentEKF(X, P_prev, ...
    R, L, Ke, ... % constants/parameters
    frequency, duty, Vin, dtheta_internal, ... % inputs (u)
    i_meas, adc_sample_time)  % measurements (z)

    % i_meas should be a matrix with;
    % n rows = number of current measurements within one PWM period
    % 3 columns, being:
    %  - current duty cycle percentage at sample instance
    %  - current sample
    
    % adc_sample_time specifies the sample time in seconds and is used to
    % model the noise covariance in the measurements the greater the sample
    % time is relative to the PWM period
    
    % Compute general parameters
    dt = 1/frequency;
    tON = duty * dt;
    tOFF = dt - tON;
    
    % Split state vector, X[k-1], into individual variables    
    Vin_R_prev = X(1); % Vin/R ratio posterior estimate from previous timestep
    i0_prev = X(2); % bottom current posterior estimate from previous timestep
    
    % Setup covariance matrices
    Q_proc = diag([0.01, 0.1]); % process noise
    R_meas = 0.02; % measurement noise
    
    %% Prediction step  
    X_apriori = zeros(2,1);
     
    % Prediction step - use the current models derived in OneNote
    Vin_R_apriori = Vin_R_prev; % assumed to stay constant, assuming that Vin and R is constant
    
    % Compute bottom/min current
    i0_apriori = Vin_R_prev * (exp(-R/L*tOFF)-exp(-R/L*dt))/(1-exp(-R/L*dt));           
    
%     i_apriori = 1/dt * (Vin/R*tON + L/R*i0_prev*(1-exp(-R/L*dt)) + L/R*Vin/R*exp(-R/L*dt) - L/R*Vin/R*exp(-R/L*tOFF)); % apriori estimate of average current
%     %i_apriori = 1/dt*Vin/R*tON + 1/dt*L/R*i0_prev*(1-exp(-R/L*dt)) + 1/dt*L/R*Vin/R*exp(-R/L*dt) - 1/dt*L/R*Vin/R*exp(-R/L*tOFF)
%       
%     i_max = (i0_prev - Vin/R)*exp(-R/L*tON) + Vin/R;
%     i_min = i_max * exp(-R/L*tOFF);
%     i0_apriori = i_min; % aprior estimate of the bottom current after current duty cycle
%     % i0_apriori = ((i0_prev - Vin/R)*exp(-R/L*tON) + Vin/R) * exp(-R/L*tOFF)
%     % i0_apriori = i0_prev*exp(-R/L*tON)*exp(-R/L*tOFF) - Vin/R*exp(-R/L*tON)*exp(-R/L*tOFF) + Vin/R*exp(-R/L*tOFF)
%     % i0_apriori = i0_prev*exp(-R/L*dt) - Vin/R*exp(-R/L*dt) + Vin/R*exp(-R/L*tOFF)
         
    X_apriori(1) = Vin_R_apriori;
    X_apriori(2) = i0_apriori;
    
    % Determine model Jacobian (F)   -  OBS. This is not supposed to use/depend on apriori states!
    F_prev = zeros(2,2);
    F_prev(1,1) = 1; % dVin_R_apriori / dVin_R_prev
    F_prev(1,2) = 0; % dVin_R_apriori / di0_prev
    F_prev(2,1) = (exp(-R/L*tOFF)-exp(-R/L*dt))/(1-exp(-R/L*dt)); % di0_apriori / dVin_R_prev
    F_prev(2,2) = 0; % di0_apriori / di0_prev
    
    % Calculate apriori covariance of estimate error
    P_apriori = F_prev * P_prev * F_prev' + Q_proc;     

    %% Update/correction step
    i_max = (i0_apriori - Vin_R_apriori)*exp(-R/L*tON) + Vin_R_apriori;
    i_min = i_max * exp(-R/L*tOFF);
    di_max_dVin_R_apriori = 1 - exp(-R/L*tON);
    di_max_di0_apriori = exp(-R/L*tON);
    
    % Form measurement and Jacobian matrix
    z = zeros(size(i_meas, 1), 1);
    z_hat = zeros(size(i_meas, 1), 1);
    H = zeros(size(i_meas, 1), 2);
    
    for (i = 1:size(i_meas, 1))        
        z_sample_duty = i_meas(i,1);
        z_current = i_meas(i,2);
        
        z(i,1) = z_current;
        
        if (z_sample_duty < duty)
            % Sampled during ON-period
            t_sample = z_sample_duty * dt;
            iON = (i0_apriori - Vin_R_apriori) * exp(-R/L*t_sample) + Vin_R_apriori;
            diON_dVin_R_apriori = 1 - exp(-R/L*t_sample);
            diON_di0_apriori = exp(-R/L*t_sample);
            
            z_hat(i,1) = iON;
            H(i,:) = [diON_dVin_R_apriori, diON_di0_apriori];
        else
            % Sampled during OFF-period
            t_sample = z_sample_duty * dt - tON;
            iOFF = i_max * exp(-R/L*t_sample);            
            diOFF_di_max = exp(-R/L*t_sample);                        
            diOFF_dVin_R_apriori = diOFF_di_max * di_max_dVin_R_apriori;
            diOFF_di0_apriori = diOFF_di_max * di_max_di0_apriori;
            
            z_hat(i,1) = iOFF;
            H(i,:) = [diOFF_dVin_R_apriori, diOFF_di0_apriori];
        end
    end    
    
    % Calculate Kalman gain
    S = H * P_apriori * H' + R_meas*eye(size(i_meas, 1));        
    K = P_apriori * H' / S;  % P_apriori * H' * inv(S)        
 
    X_aposteriori = X_apriori + K * (z - z_hat);    
    P_aposteriori = (eye(2) - K*H) * P_apriori;     
     
    %% Set output
    i_avg = duty * X_aposteriori(1);
    
    X_out = [X_aposteriori; i_avg];
    P_out = P_aposteriori;
    
%     %% Compute output based on prediction model
%     i0_aposteriori = X_aposteriori;
%     
%     % Prediction step - use the current models derived in OneNote
%     i_avg = 1/dt * (Vin_R*tON + L/R*i0_aposteriori*(1-exp(-R/L*dt)) + L/R*Vin_R*exp(-R/L*dt) - L/R*Vin_R*exp(-R/L*tOFF)); % apriori estimate of average current    
%     i_max = (i0_prev - Vin_R)*exp(-R/L*tON) + Vin_R;
%     i_min = i_max * exp(-R/L*tOFF);
%     i0_aposteriori_next = i_min; % aprior estimate of the bottom current after current duty cycle
%     
%     X_out = zeros(2,1);
%     X_out(1) = i_avg;
%     X_out(2) = i0_aposteriori_next;    
%     P_out = P_aposteriori;
end
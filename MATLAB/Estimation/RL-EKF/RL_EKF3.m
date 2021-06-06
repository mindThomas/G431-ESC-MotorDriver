function [X_out, P_out, i_avg] = RL_EKF3(X, P, ...
    Ke, ... % constants/parameters
    frequency, duty, Vin, omega, ... % inputs (u)
    i_sample1, location_sample1, ...
    i_sample2, location_sample2)  % measurements (z)

    % Compute general parameters
    dt = 1/frequency;
    tON = duty * dt;
    tOFF = dt - tON;
    
    % Split state vector, X[k-1], into individual variables            
    R = X(1);
    L = X(2);
    
    i_avg = 0;
    
    % Setup covariance matrices
    Q_proc = diag([0.000001, 0.000000001]); % process noise
    R_meas = 0.03; % measurement noise
    
    %% Update step
    % Form measurement and Jacobian matrix
    z = zeros(2, 1);
    z_hat = zeros(2, 1);
    H = zeros(2, 2);     
   
    if (location_sample1 > 0 && location_sample1 <= duty)
        % Sampled during ON-period
        t_sample = location_sample1 * dt;        
        i_sample1_hat = 1/R*( (Vin*(exp(-R/L*tOFF)-1) - 2*Ke*omega*exp(-R/L*dt))/(1-exp(-R/L*dt)) )*exp(-R/L*t_sample) + Vin/R - Ke*omega/R;
        di_sample1_dR = (Ke*omega)/R^2 - Vin/R^2 + (exp(-(R*t_sample)/L)*(Vin*(exp(-(R*tOFF)/L) - 1) - 2*Ke*omega*exp(-(R*dt)/L)))/(R^2*(exp(-(R*dt)/L) - 1)) + (exp(-(R*t_sample)/L)*((Vin*tOFF*exp(-(R*tOFF)/L))/L - (2*Ke*dt*omega*exp(-(R*dt)/L))/L))/(R*(exp(-(R*dt)/L) - 1)) + (t_sample*exp(-(R*t_sample)/L)*(Vin*(exp(-(R*tOFF)/L) - 1) - 2*Ke*omega*exp(-(R*dt)/L)))/(L*R*(exp(-(R*dt)/L) - 1)) - (dt*exp(-(R*dt)/L)*exp(-(R*t_sample)/L)*(Vin*(exp(-(R*tOFF)/L) - 1) - 2*Ke*omega*exp(-(R*dt)/L)))/(L*R*(exp(-(R*dt)/L) - 1)^2);
        di_sample1_dL = (dt*exp(-(R*dt)/L)*exp(-(R*t_sample)/L)*(Vin*(exp(-(R*tOFF)/L) - 1) - 2*Ke*omega*exp(-(R*dt)/L)))/(L^2*(exp(-(R*dt)/L) - 1)^2) - (t_sample*exp(-(R*t_sample)/L)*(Vin*(exp(-(R*tOFF)/L) - 1) - 2*Ke*omega*exp(-(R*dt)/L)))/(L^2*(exp(-(R*dt)/L) - 1)) - (exp(-(R*t_sample)/L)*((R*Vin*tOFF*exp(-(R*tOFF)/L))/L^2 - (2*Ke*R*dt*omega*exp(-(R*dt)/L))/L^2))/(R*(exp(-(R*dt)/L) - 1));
        %di_sample1_dR = (Vin-Ke*omega)/R^2*exp(-R/L*t_sample) - t_sample/L*(i0_prev - (Vin-Ke*omega)/R)*exp(-R/L*t_sample) - (Vin-Ke*omega)/R^2;
        %di_sample1_dL = R/L^2*t_sample * (i0_prev - (Vin-Ke*omega)/R) * exp(-R/L*t_sample);

        z(1,1) = i_sample1;
        z_hat(1,1) = i_sample1_hat;    
        H(1,:) = [di_sample1_dR, di_sample1_dL];
    else
        disp('Error: location_sample1 > duty which is current not handled');
    end
    
    if (location_sample2 > 0 && location_sample2 > duty)
        % Sampled during OFF-period               
        t_sample = location_sample2 * dt - tON;
        i_sample2_hat = Vin/R*exp(-R/L*t_sample)*( (1-exp(-R/L*tON))/(1-exp(-R/L*dt)) ) - Ke*omega/R*( (1-exp(-R/L*dt)+2*exp(-R/L*dt)*exp(-R/L*tON)*exp(-R/L*t_sample))/(1-exp(-R/L*dt)) );
        di_sample2_dR = (Vin*dt*exp(-(R*dt)/L)*exp(-(R*t_sample)/L)*(exp(-(R*tON)/L) - 1))/(L*R*(exp(-(R*dt)/L) - 1)^2) - (Ke*omega*(2*exp(-(R*dt)/L)*exp(-(R*tON)/L)*exp(-(R*t_sample)/L) - exp(-(R*dt)/L) + 1))/(R^2*(exp(-(R*dt)/L) - 1)) - (Ke*omega*((2*dt*exp(-(R*dt)/L)*exp(-(R*tON)/L)*exp(-(R*t_sample)/L))/L - (dt*exp(-(R*dt)/L))/L + (2*tON*exp(-(R*dt)/L)*exp(-(R*tON)/L)*exp(-(R*t_sample)/L))/L + (2*t_sample*exp(-(R*dt)/L)*exp(-(R*tON)/L)*exp(-(R*t_sample)/L))/L))/(R*(exp(-(R*dt)/L) - 1)) - (Vin*t_sample*exp(-(R*t_sample)/L)*(exp(-(R*tON)/L) - 1))/(L*R*(exp(-(R*dt)/L) - 1)) - (Vin*tON*exp(-(R*tON)/L)*exp(-(R*t_sample)/L))/(L*R*(exp(-(R*dt)/L) - 1)) - (Vin*exp(-(R*t_sample)/L)*(exp(-(R*tON)/L) - 1))/(R^2*(exp(-(R*dt)/L) - 1)) + (Ke*dt*omega*exp(-(R*dt)/L)*(2*exp(-(R*dt)/L)*exp(-(R*tON)/L)*exp(-(R*t_sample)/L) - exp(-(R*dt)/L) + 1))/(L*R*(exp(-(R*dt)/L) - 1)^2);
        di_sample2_dL = (Ke*omega*((2*R*dt*exp(-(R*dt)/L)*exp(-(R*tON)/L)*exp(-(R*t_sample)/L))/L^2 - (R*dt*exp(-(R*dt)/L))/L^2 + (2*R*tON*exp(-(R*dt)/L)*exp(-(R*tON)/L)*exp(-(R*t_sample)/L))/L^2 + (2*R*t_sample*exp(-(R*dt)/L)*exp(-(R*tON)/L)*exp(-(R*t_sample)/L))/L^2))/(R*(exp(-(R*dt)/L) - 1)) + (Vin*tON*exp(-(R*tON)/L)*exp(-(R*t_sample)/L))/(L^2*(exp(-(R*dt)/L) - 1)) + (Vin*t_sample*exp(-(R*t_sample)/L)*(exp(-(R*tON)/L) - 1))/(L^2*(exp(-(R*dt)/L) - 1)) - (Ke*dt*omega*exp(-(R*dt)/L)*(2*exp(-(R*dt)/L)*exp(-(R*tON)/L)*exp(-(R*t_sample)/L) - exp(-(R*dt)/L) + 1))/(L^2*(exp(-(R*dt)/L) - 1)^2) - (Vin*dt*exp(-(R*dt)/L)*exp(-(R*t_sample)/L)*(exp(-(R*tON)/L) - 1))/(L^2*(exp(-(R*dt)/L) - 1)^2);
        %di_sample2_dR = (di_max_dR - Ke*omega/R^2)*exp(-R/L*t_sample) - t_sample/L*(i_max_hat + Ke*omega/R)*exp(-R/L*t_sample) + Ke*omega/R^2;
        %di_sample2_dL = di_max_dL * exp(-R/L*t_sample) + R/L^2*t_sample*(i_max_hat + Ke*omega/R)*exp(-R/L*t_sample);
        
        z(2,1) = i_sample2;
        z_hat(2,1) = i_sample2_hat;    
        H(2,:) = [di_sample2_dR, di_sample2_dL];              
    else
        disp('Error: location_sample2 < duty which is current not handled');
    end   
    
    % Calculate Kalman gain
    S = H * P * H' + R_meas*eye(2);        
    K = P * H' / S;  % P_apriori * H' * inv(S)        
 
    X = X + K * (z - z_hat);    
    P = (eye(2) - K*H) * P;
        
    if (X(2) < 1e-6)
        X(2) = 1e-6;
    end        
    
    %% Prediction step      

    % Determine model Jacobian (F)   -  OBS. This is not supposed to use/depend on apriori states!
    F = zeros(2,2);    
    F(1,1) = 1; % dR / dR
    F(1,2) = 0; % dR / dL    
    F(2,1) = 0; % dL / dR    
    F(2,2) = 1; % dL / dL
    
    % Calculate apriori covariance of estimate error
    P = F * P * F';  
    
    % Include process covariance initially to speed up convergence
    if (P(1,1) > 0.25e-3)
        %P = P + Q_proc;
    end
     
    %% Set output
    X_out = X;
    P_out = P;
end
function [X_out, P_out] = MotorRL_EKF2(X, P_prev, ...
    Vbat, tdelta_high, ... % inputs (u)
    i_high)  %#codegen  

    % Split state vector, X[k-1], into individual variables    
    Rm = X(1);
    Lm = X(2);
    i0 = X(3);
    
    % Setup covariance matrices
    Q = diag([0.0001, 0.000000001, 0.1]); % process noise on constant parameter model
    R = 10 * eye(1); % measurement noise of high and low current
    
    % Measurement vector    
    z = [i_high];   
    
    %% Prediction step  
    X_apriori = single(zeros(3,1));
     
    % Prediction step - constant parameter model
    X_apriori = X;
    
    % Determine model Jacobian (F)   -  OBS. This is not supposed to use/depend on apriori states!
    F_prev = single(zeros(3,10));
    F_prev = eye(3);

    % Calculate apriori covariance of estimate error
    P_apriori = F_prev * P_prev * F_prev' + Q;           
    
    %% Update/correction step  
    z_hat_i_high = Vbat/Rm * (1 - (1-i0*Rm/Vbat) * exp(-Rm/Lm * tdelta_high)); % from getCurrentUp.m    
    
    di_high_dRm = (Vbat*((i0*exp(-(Rm*tdelta_high)/Lm))/Vbat - (tdelta_high*exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1))/Lm))/Rm - (Vbat*(exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1) + 1))/Rm^2;
    di_high_dLm = (Vbat*tdelta_high*exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1))/Lm^2;
    di_high_di0 = exp(-(Rm*tdelta_high)/Lm);
  
    z_hat = [z_hat_i_high];    
    H = [di_high_dRm, di_high_dLm, di_high_di0];

    % Calculate Kalman gain
    S = H * P_apriori * H' + R;        
    K = P_apriori * H' / S;  % P_apriori * H' * inv(S)        

    X_aposteriori = X_apriori + K * (z - z_hat);    
    P_aposteriori = (eye(3) - K*H) * P_apriori;     
     
    %% Send output
    X_out = X_aposteriori;
    P_out = P_aposteriori;
end
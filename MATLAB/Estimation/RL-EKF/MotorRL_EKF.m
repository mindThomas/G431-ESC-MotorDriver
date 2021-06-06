function [X_out, P_out] = MotorRL_EKF(X, P_prev, ...
    i0, Vbat, tdelta_high, tdelta_low, ... % inputs (u)
    i_high, i_low)  %#codegen  

    % Split state vector, X[k-1], into individual variables    
    Rm = X(1);
    Lm = X(2);
    
    % Setup covariance matrices
    Q = diag([0.0001, 0.000000001]); % process noise on constant parameter model
    R = 10 * eye(2); % measurement noise of high and low current
    
    % Measurement vector    
    z = [i_high; i_low];   
    
    %% Prediction step  
    X_apriori = single(zeros(2,1));
     
    % Prediction step - constant parameter model
    X_apriori = X;
    
    % Determine model Jacobian (F)   -  OBS. This is not supposed to use/depend on apriori states!
    F_prev = single(zeros(2,10));
    F_prev = eye(2);

    % Calculate apriori covariance of estimate error
    P_apriori = F_prev * P_prev * F_prev' + Q;           
    
    %% Update/correction step  
    z_hat_i_high = Vbat/Rm * (1 - (1-i0*Rm/Vbat) * exp(-Rm/Lm * tdelta_high)); % from getCurrentUp.m
    z_hat_i_low = z_hat_i_high * exp(-Rm/Lm * tdelta_low); % from getCurrentDown.m
    
    di_high_dRm = (Vbat*((i0*exp(-(Rm*tdelta_high)/Lm))/Vbat - (tdelta_high*exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1))/Lm))/Rm - (Vbat*(exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1) + 1))/Rm^2;
    di_high_dLm = (Vbat*tdelta_high*exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1))/Lm^2;
    
    %di_high_dRm = di_high_dRm * exp(-Rm/Lm * tdelta_low) + z_hat_i_high * d/dR (exp(-Rm/Lm * tdelta_low))
    %di_low_dRm = di_high_dRm * exp(-Rm/Lm * tdelta_low) + z_hat_i_high * (-tdelta_low/Lm) * exp(-Rm/Lm * tdelta_low);    
    %di_low_dLm = di_high_dLm * exp(-Rm/Lm * tdelta_low) + z_hat_i_high * (-Rm/(Lm^2)*tdelta_low) * exp(-Rm/Lm * tdelta_low);
    di_low_dRm = (Vbat*exp(-(Rm*tdelta_low)/Lm)*((i0*exp(-(Rm*tdelta_high)/Lm))/Vbat - (tdelta_high*exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1))/Lm))/Rm - (Vbat*exp(-(Rm*tdelta_low)/Lm)*(exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1) + 1))/Rm^2 - (Vbat*tdelta_low*exp(-(Rm*tdelta_low)/Lm)*(exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1) + 1))/(Lm*Rm);
    di_low_dLm = (Vbat*tdelta_low*exp(-(Rm*tdelta_low)/Lm)*(exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1) + 1))/Lm^2 + (Vbat*tdelta_high*exp(-(Rm*tdelta_low)/Lm)*exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1))/Lm^2;
    
    
    z_hat = [z_hat_i_high; z_hat_i_low];    
    H = [di_high_dRm, di_high_dLm;
         di_low_dRm, di_low_dLm];
     
    % Calculate Kalman gain
    S = H * P_apriori * H' + R;        
    K = P_apriori * H' / S;  % P_apriori * H' * inv(S)        

    X_aposteriori = X_apriori + K * (z - z_hat);    
    P_aposteriori = (eye(2) - K*H) * P_apriori;     
     
    %% Send output
    X_out = X_aposteriori;
    P_out = P_aposteriori;
end
% Current sampling is aligned with the center of the ON- and OFF-periods
% TODO!!!! Update the script below to do this!!!!
function [X_out, P_out] = MotorRL_EKF_CenterAligned(X, P_prev, ...
    i0, Vbat, tdelta_high, tdelta_low, toffset_low, toffset_high, ... % inputs (u)
    i_high, i_low)  %#codegen  

    % Split state vector, X[k-1], into individual variables    
    Rm = X(1);
    Lm = X(2);
    
    % Setup covariance matrices
    Q = 0.001 * diag([0.0001, 0.0000001]); % process noise on constant parameter model
    R = 0.1 * eye(2); % measurement noise of high and low current
    
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
    i_peak = Vbat/Rm * (1 - (1-i0*Rm/Vbat) * exp(-Rm/Lm * (tdelta_high - toffset_low)));  % from getCurrentUp.m
    z_hat_i_high = i_peak * exp(-Rm/Lm * toffset_high); % from getCurrentDown.m
    i_low = z_hat_i_high * exp(-Rm/Lm * tdelta_low); % from getCurrentDown.m
    z_hat_i_low = Vbat/Rm * (1 - (1-i_low*Rm/Vbat) * exp(-Rm/Lm * toffset_low)); % from getCurrentUp.m
    
    di_high_dRm = (Vbat*exp(-(Rm*toffset_high)/Lm)*((i0*exp(-(Rm*(tdelta_high - toffset_low))/Lm))/Vbat - (exp(-(Rm*(tdelta_high - toffset_low))/Lm)*((Rm*i0)/Vbat - 1)*(tdelta_high - toffset_low))/Lm))/Rm - (Vbat*exp(-(Rm*toffset_high)/Lm)*(exp(-(Rm*(tdelta_high - toffset_low))/Lm)*((Rm*i0)/Vbat - 1) + 1))/Rm^2 - (Vbat*toffset_high*exp(-(Rm*toffset_high)/Lm)*(exp(-(Rm*(tdelta_high - toffset_low))/Lm)*((Rm*i0)/Vbat - 1) + 1))/(Lm*Rm);
                 %(Vbat*((i0*exp(-(Rm*tdelta_high)/Lm))/Vbat - (tdelta_high*exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1))/Lm))/Rm - (Vbat*(exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1) + 1))/Rm^2;
    di_high_dLm = (Vbat*toffset_high*exp(-(Rm*toffset_high)/Lm)*(exp(-(Rm*(tdelta_high - toffset_low))/Lm)*((Rm*i0)/Vbat - 1) + 1))/Lm^2 + (Vbat*exp(-(Rm*toffset_high)/Lm)*exp(-(Rm*(tdelta_high - toffset_low))/Lm)*((Rm*i0)/Vbat - 1)*(tdelta_high - toffset_low))/Lm^2;
                 %(Vbat*tdelta_high*exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1))/Lm^2;
    
    %di_high_dRm = di_high_dRm * exp(-Rm/Lm * tdelta_low) + z_hat_i_high * d/dR (exp(-Rm/Lm * tdelta_low))
    %di_low_dRm = di_high_dRm * exp(-Rm/Lm * tdelta_low) + z_hat_i_high * (-tdelta_low/Lm) * exp(-Rm/Lm * tdelta_low);    
    %di_low_dLm = di_high_dLm * exp(-Rm/Lm * tdelta_low) + z_hat_i_high * (-Rm/(Lm^2)*tdelta_low) * exp(-Rm/Lm * tdelta_low);
    di_low_dRm = - (Vbat*(exp(-(Rm*toffset_low)/Lm)*(exp(-(Rm*tdelta_low)/Lm)*exp(-(Rm*toffset_high)/Lm)*(exp(-(Rm*(tdelta_high - toffset_low))/Lm)*((Rm*i0)/Vbat - 1) + 1) - 1) + 1))/Rm^2 - (Vbat*(exp(-(Rm*toffset_low)/Lm)*((tdelta_low*exp(-(Rm*tdelta_low)/Lm)*exp(-(Rm*toffset_high)/Lm)*(exp(-(Rm*(tdelta_high - toffset_low))/Lm)*((Rm*i0)/Vbat - 1) + 1))/Lm - exp(-(Rm*tdelta_low)/Lm)*exp(-(Rm*toffset_high)/Lm)*((i0*exp(-(Rm*(tdelta_high - toffset_low))/Lm))/Vbat - (exp(-(Rm*(tdelta_high - toffset_low))/Lm)*((Rm*i0)/Vbat - 1)*(tdelta_high - toffset_low))/Lm) + (toffset_high*exp(-(Rm*tdelta_low)/Lm)*exp(-(Rm*toffset_high)/Lm)*(exp(-(Rm*(tdelta_high - toffset_low))/Lm)*((Rm*i0)/Vbat - 1) + 1))/Lm) + (toffset_low*exp(-(Rm*toffset_low)/Lm)*(exp(-(Rm*tdelta_low)/Lm)*exp(-(Rm*toffset_high)/Lm)*(exp(-(Rm*(tdelta_high - toffset_low))/Lm)*((Rm*i0)/Vbat - 1) + 1) - 1))/Lm))/Rm;
                %(Vbat*exp(-(Rm*tdelta_low)/Lm)*((i0*exp(-(Rm*tdelta_high)/Lm))/Vbat - (tdelta_high*exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1))/Lm))/Rm - (Vbat*exp(-(Rm*tdelta_low)/Lm)*(exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1) + 1))/Rm^2 - (Vbat*tdelta_low*exp(-(Rm*tdelta_low)/Lm)*(exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1) + 1))/(Lm*Rm);
    di_low_dLm = (Vbat*(exp(-(Rm*toffset_low)/Lm)*((Rm*tdelta_low*exp(-(Rm*tdelta_low)/Lm)*exp(-(Rm*toffset_high)/Lm)*(exp(-(Rm*(tdelta_high - toffset_low))/Lm)*((Rm*i0)/Vbat - 1) + 1))/Lm^2 + (Rm*toffset_high*exp(-(Rm*tdelta_low)/Lm)*exp(-(Rm*toffset_high)/Lm)*(exp(-(Rm*(tdelta_high - toffset_low))/Lm)*((Rm*i0)/Vbat - 1) + 1))/Lm^2 + (Rm*exp(-(Rm*tdelta_low)/Lm)*exp(-(Rm*toffset_high)/Lm)*exp(-(Rm*(tdelta_high - toffset_low))/Lm)*((Rm*i0)/Vbat - 1)*(tdelta_high - toffset_low))/Lm^2) + (Rm*toffset_low*exp(-(Rm*toffset_low)/Lm)*(exp(-(Rm*tdelta_low)/Lm)*exp(-(Rm*toffset_high)/Lm)*(exp(-(Rm*(tdelta_high - toffset_low))/Lm)*((Rm*i0)/Vbat - 1) + 1) - 1))/Lm^2))/Rm;
                %(Vbat*tdelta_low*exp(-(Rm*tdelta_low)/Lm)*(exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1) + 1))/Lm^2 + (Vbat*tdelta_high*exp(-(Rm*tdelta_low)/Lm)*exp(-(Rm*tdelta_high)/Lm)*((Rm*i0)/Vbat - 1))/Lm^2;    
    
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